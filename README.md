# Conventor Package

## 功能

Conventor是一个用于VL53L5(其他深度传感器应该也可以)数据处理的ROS包。应配合深度图像Topic使用，功能包括：

1. **深度图像转点云**：
    - 根据提供的深度图像和CameraInfo和TF信息，将多个传感器的深度信息转换成单一点云

2. **ROS功能**：
    - 通过ROS话题订阅深度图像和CameraInfo，发布点云数据，方便与其他ROS节点交互。
