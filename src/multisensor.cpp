#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <mutex>
#include <vector>

// 点云类型定义
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class MultiDepthToPointcloud
{
private:
  // ROS句柄
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // 参数
  std::string target_frame_;
  std::vector<std::string> camera_topics_;
  std::vector<std::string> camera_info_topics_;
  double scale_factor_;
  
  // 发布者
  ros::Publisher cloud_pub_;
  
  // 多个相机的订阅器
  std::vector<ros::Subscriber> depth_subs_;
  std::vector<ros::Subscriber> camera_info_subs_;
  
  // 存储每个相机的相机信息和最新的深度图像
  std::vector<sensor_msgs::CameraInfo> camera_infos_;
  std::vector<cv_bridge::CvImageConstPtr> recent_depth_images_;
  
  // TF监听器
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  // 互斥锁，用于保护共享数据访问
  std::mutex mutex_;
  
  // 追踪哪些相机有数据
  std::vector<bool> camera_data_ready_;
  
  // 计时器，用于定期发布点云
  ros::Timer publish_timer_;

public:
  MultiDepthToPointcloud() 
    : pnh_("~"),
      tf_listener_(tf_buffer_)
  {
    // 获取参数
    pnh_.param<std::string>("target_frame", target_frame_, "map");
    pnh_.param<double>("scale_factor", scale_factor_, 0.001); // 默认毫米转米
    
    // 读取相机话题列表
    XmlRpc::XmlRpcValue camera_list;
    if (pnh_.getParam("cameras", camera_list)) {
      ROS_ASSERT(camera_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      
      for (int i = 0; i < camera_list.size(); i++) {
        XmlRpc::XmlRpcValue camera_item = camera_list[i];
        ROS_ASSERT(camera_item.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        ROS_ASSERT(camera_item.hasMember("depth_topic") && camera_item.hasMember("camera_info_topic"));
        
        std::string depth_topic = static_cast<std::string>(camera_item["depth_topic"]);
        std::string camera_info_topic = static_cast<std::string>(camera_item["camera_info_topic"]);
        
        camera_topics_.push_back(depth_topic);
        camera_info_topics_.push_back(camera_info_topic);
      }
    } else {
      ROS_ERROR("No cameras specified in the parameters. Using default.");
      camera_topics_.push_back("/camera/depth/image_raw");
      camera_info_topics_.push_back("/camera/depth/camera_info");
    }
    
    // 初始化数据结构
    int num_cameras = camera_topics_.size();
    camera_infos_.resize(num_cameras);
    recent_depth_images_.resize(num_cameras);
    camera_data_ready_.resize(num_cameras, false);
    
    // 创建点云发布者
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/combined_point_cloud", 1);
    
    // 为每个相机创建订阅器
    for (int i = 0; i < num_cameras; i++) {
      depth_subs_.push_back(
        nh_.subscribe<sensor_msgs::Image>(
          camera_topics_[i], 1, 
          boost::bind(&MultiDepthToPointcloud::depthCallback, this, _1, i)
        )
      );
      
      camera_info_subs_.push_back(
        nh_.subscribe<sensor_msgs::CameraInfo>(
          camera_info_topics_[i], 1, 
          boost::bind(&MultiDepthToPointcloud::cameraInfoCallback, this, _1, i)
        )
      );
      
      ROS_INFO_STREAM("Subscribed to depth topic: " << camera_topics_[i]);
      ROS_INFO_STREAM("Subscribed to camera info topic: " << camera_info_topics_[i]);
    }
    
    // 创建定时发布器 (10Hz)
    publish_timer_ = nh_.createTimer(
      ros::Duration(0.06), 
      &MultiDepthToPointcloud::publishTimerCallback, 
      this
    );
    
    ROS_INFO("Multi-depth to pointcloud node initialized");
    ROS_INFO_STREAM("Target frame: " << target_frame_);
    ROS_INFO_STREAM("Scale factor: " << scale_factor_);
  }
  
  void depthCallback(const sensor_msgs::Image::ConstPtr& msg, int camera_index)
  {
    try {
      // 将ROS图像消息转换为OpenCV图像
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      
      // 保存图像
      std::lock_guard<std::mutex> lock(mutex_);
      recent_depth_images_[camera_index] = cv_ptr;
      camera_data_ready_[camera_index] = true;
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("CV Bridge error: %s", e.what());
    }
  }
  
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg, int camera_index)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    camera_infos_[camera_index] = *msg;
  }
  
  void publishTimerCallback(const ros::TimerEvent& event)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 检查是否有足够的数据可用
    bool any_camera_ready = false;
    for (bool ready : camera_data_ready_) {
      if (ready) {
        any_camera_ready = true;
        break;
      }
    }
    
    if (!any_camera_ready) {
      return;  // 没有准备好的摄像机，继续等待
    }
    
    // 创建组合点云
    PointCloudT::Ptr combined_cloud(new PointCloudT);
    
    ros::Time cloud_time = ros::Time::now();
    
    // 处理每个相机的数据
    for (size_t i = 0; i < camera_topics_.size(); i++) {
      if (!camera_data_ready_[i] || recent_depth_images_[i] == nullptr) {
        continue;  // 跳过这个相机
      }
      
      // 获取该相机的点云
      PointCloudT::Ptr camera_cloud = depthImageToPointCloud(
        recent_depth_images_[i], 
        camera_infos_[i], 
        i
      );
      
      if (!camera_cloud) {
        continue;
      }
      
      // 设置点云头部信息
      pcl_conversions::toPCL(recent_depth_images_[i]->header.stamp, camera_cloud->header.stamp);
      camera_cloud->header.frame_id = recent_depth_images_[i]->header.frame_id;
      
      // 转换到目标坐标系
      PointCloudT::Ptr transformed_cloud(new PointCloudT);
      if (!transformPointCloud(camera_cloud, transformed_cloud, target_frame_)) {
        ROS_WARN_STREAM("Failed to transform point cloud from camera " << i);
        continue;
      }
      
      // 添加到组合点云
      *combined_cloud += *transformed_cloud;
    }
    
    // 发布组合点云
    if (combined_cloud->points.size() > 0) {
      sensor_msgs::PointCloud2 output_msg;
      pcl::toROSMsg(*combined_cloud, output_msg);
      output_msg.header.stamp = cloud_time;
      output_msg.header.frame_id = target_frame_;
      cloud_pub_.publish(output_msg);
      
      ROS_DEBUG("Published combined point cloud with %lu points", combined_cloud->points.size());
    }
  }
  
  PointCloudT::Ptr depthImageToPointCloud(
    const cv_bridge::CvImageConstPtr& depth_image, 
    const sensor_msgs::CameraInfo& camera_info,
    int camera_index)
  {
    if (!depth_image || depth_image->image.empty()) {
      return nullptr;
    }
    
    // 创建新的点云
    PointCloudT::Ptr cloud(new PointCloudT);
    
    // 获取相机参数
    float fx = camera_info.K[0];
    float fy = camera_info.K[4];
    float cx = camera_info.K[2];
    float cy = camera_info.K[5];
    
    if (fx == 0 || fy == 0) {
      ROS_WARN_STREAM("Invalid camera calibration for camera " << camera_index);
      return nullptr;
    }
    
    // 从深度图像生成点云
    const cv::Mat& depth_mat = depth_image->image;
    
    for (int v = 0; v < depth_mat.rows; v++) {
      for (int u = 0; u < depth_mat.cols; u++) {
        uint16_t depth_value = depth_mat.at<uint16_t>(v, u);
        
        // 过滤掉无效的深度值
        if (depth_value == 0) {
          continue;
        }
        
        // 转换为米
        float depth_meters = depth_value * scale_factor_;
        
        // 计算3D点
        PointT point;
        point.x = (u - cx) * depth_meters / fx;
        point.y = (v - cy) * depth_meters / fy;
        point.z = depth_meters;
        
        cloud->points.push_back(point);
      }
    }
    
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    
    return cloud;
  }
  
  bool transformPointCloud(
    const PointCloudT::Ptr& input_cloud, 
    PointCloudT::Ptr& output_cloud,
    const std::string& target_frame)
  {
    if (input_cloud->header.frame_id == target_frame) {
      // 如果已经在目标坐标系中，则直接复制
      *output_cloud = *input_cloud;
      return true;
    }
    
    try {
      // 查找从相机坐标系到目标坐标系的变换
      geometry_msgs::TransformStamped transform_stamped = 
        tf_buffer_.lookupTransform(
          target_frame,
          input_cloud->header.frame_id,
          ros::Time(0),
          ros::Duration(0.1)
        );
      
      // 将变换转换为Eigen矩阵
      Eigen::Isometry3d transform = tf2::transformToEigen(transform_stamped);
      
      // 变换点云
      output_cloud->points.resize(input_cloud->points.size());
      output_cloud->width = input_cloud->width;
      output_cloud->height = input_cloud->height;
      output_cloud->is_dense = input_cloud->is_dense;
      output_cloud->header = input_cloud->header;
      output_cloud->header.frame_id = target_frame;
      
      // 应用变换到每个点
      for (size_t i = 0; i < input_cloud->points.size(); i++) {
        const PointT& input_point = input_cloud->points[i];
        PointT& output_point = output_cloud->points[i];
        
        Eigen::Vector3d point(input_point.x, input_point.y, input_point.z);
        Eigen::Vector3d transformed_point = transform * point;
        
        output_point.x = transformed_point.x();
        output_point.y = transformed_point.y();
        output_point.z = transformed_point.z();
      }
      
      return true;
    }
    catch (tf2::TransformException& ex) {
      ROS_WARN_STREAM("Transform failure: " << ex.what());
      return false;
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_depth_to_pointcloud");
  
  MultiDepthToPointcloud converter;
  
  ros::spin();
  
  return 0;
}