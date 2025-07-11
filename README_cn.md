# stereo_extrinsic_calib

一个用于立体相机外参标定的ROS1包，通过RViz点击点实现。

## 功能简介
- 订阅RViz的`/clicked_point` (geometry_msgs/PointStamped)
- 收集用户点击的三维点
- 使用RANSAC拟合地面平面
- 计算相机到base_link的变换（旋转和平移）
- 输出ROS静态变换发布命令

## 依赖
- ROS（建议Noetic）
- Eigen3
- geometry_msgs
- tf
- roscpp

## 使用方法
1. 编译包：
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```
2. 启动RViz，使用“Publish Point”工具在地面点击至少3个点。
3. 运行标定节点：
   ```bash
   rosrun stereo_extrinsic_calib calib_node
   ```
4. 节点会在收到足够点后输出一条static_transform_publisher命令，复制并运行该命令即可发布base_link与camera_link之间的外参。

## 说明
- 点击点越多，标定越稳健。
- 节点会自动保证z轴朝上，并尽量对齐x轴。

## License
BSD
