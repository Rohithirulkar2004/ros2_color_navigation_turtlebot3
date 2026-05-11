# ROS2 Color Navigation with TurtleBot3
ROS2 Humble + Turtlebot3 + OpenCV project for autonomous color-based navigation in Gazebo/Ignition Gazebo using computer vision and LIDAR

This project implements autonomous color-based navigation using TurtleBot3 in Gazebo/ Ignition Gazebo with ROS2 Humble and OpenCV.

The robot:
- Detects a green object using computer vision
- Rotates toward the object using a P-controller
- Moves toward the object
- Uses LiDAR for obstacle distance estimation and stopping

## Technologies Used
- ROS2 Humble
- TurtleBot3
- Gazebo / Ignition Gazebo
- OpenCV
- Python
- LiDAR
  
## Features
- Real-time green object detection
- ROS2 publisher/subscriber architecture
- OpenCV contour tracking
- P-controller based navigation
- LiDAR assisted stopping
- Gazebo simulation support
