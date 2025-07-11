# stereo_extrinsic_calib

A ROS1 package for stereo camera extrinsic calibration using clicked points in RViz.

## Features
- Subscribe to `/clicked_point` (geometry_msgs/PointStamped) from RViz
- Collect 3D points clicked by the user
- Use RANSAC to fit a ground plane from the clicked points
- Compute the transformation (rotation and translation) from camera to base_link
- Output a static transform publisher command for ROS

## Dependencies
- ROS (tested on Noetic)
- Eigen3
- geometry_msgs
- tf
- roscpp

## Usage
1. Build the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```
2. Launch RViz and publish points using the 'Publish Point' tool on the desired ground plane.
3. Run the calibration node:
   ```bash
   rosrun stereo_extrinsic_calib calib_node
   ```
4. After clicking at least 3 points, the node will print a static transform publisher command. Copy and use this command to publish the extrinsic calibration between `base_link` and `camera_link`.

## Notes
- The more points you click, the more robust the calibration.
- The node automatically orients the z-axis upward and aligns the x-axis as much as possible.

## License
BSD
