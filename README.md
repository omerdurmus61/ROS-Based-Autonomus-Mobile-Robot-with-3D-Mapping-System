## Contents

1. [Introduction](#introduction)
2. [Mechanical Design and Manufacturing](#mechanical-design-and-manufacturing)
3. [System Architecture](#system-architecture)
4. [Installation and Usage](#installation-and-usage)
5. [Visualization](#visualization)
6. [Mapping](#mapping)
7. [Acknowledgment]((#acknowledgment))
8. [References](#references)
9. [Contact](#contact)
   
# ROS Based Autonomus Mobile Robot with 3D Mapping System
# TÜBİTAK 2209-B Industry Oriented Undergraduate Research Projects Support Program
## Introduction
This repository contains the implementation details, source code, and documentation for an autonomous differential drive mobile robot platform developed using ROS. The project includes a 3D mapping system utilizing a 3D LiDAR sensor mounted on a servo motor for high-resolution point cloud generation. For more details about the mapping system you can visit my [3D LiDAR Rotation System](https://github.com/omerdurmus61/3D-LiDAR-Rotation-System-Design?tab=readme-ov-file#system-architecture) project repository.

| ![LiDAR System](https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System/blob/master/images/physical_robot1.jpeg) | ![LiDAR in Action](https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System/blob/master/images/physical_robot2.jpeg) |
|------------------------------------|------------------------------------|

| ![LiDAR System 1](https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System/blob/master/images/robot_visualization2.gif) |
|--------------------------|


| ![LiDAR System](https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System/blob/master/images/motion_test1.gif) | ![LiDAR in Action](https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System/blob/master/images/motion_test2.gif) |
|------------------------------------|------------------------------------|

## Mechanical Design and Manufacturing
The electromechanical system was designed using Autodesk Fusion 360 and features a robust frame constructed from aluminum profiles. The design incorporates several components manufactured with a 3D printer, ensuring the system’s mechanical integrity and adaptability.

| ![LiDAR System](https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System/blob/master/images/CAD1.png) | ![LiDAR System](https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System/blob/master/images/CAD2.png) | ![LiDAR System](https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System/blob/master/images/CAD3.png) |
|--------------------------|--------------------------|--------------------------|


## System Architecture
This system illustrates the operation of a 3D LiDAR-enabled autonomous mobile robot controlled by a PC running ROS Melodic on Ubuntu 18.04. The PC handles tasks such as scanning, mapping, and navigation by processing LiDAR data and converting point clouds into usable information for a mapping algorithm that builds an environment map. A servo motor rotates the LiDAR sensor to expand its scanning coverage, sending position data back to the PC. The microcontroller manages motor control and speed regulation by processing encoder data, using a PID algorithm to adjust wheel speeds, and generating PWM signals to drive the motors. Additionally, the IMU provides acceleration and angular velocity data, which are combined with kinematic calculations and a Kalman filter to generate the robot's odometry information.

| ![LiDAR System 1](https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System/blob/master/images/flowchart.png) |
|--------------------------|
|  Flowchart               |

This RQT_GRAPH illustrates the ROS-based structure of a robotic system, showing the interaction between nodes, topics, and transform frames (TF). The /robot_marker and /platform_marker topics are used for visualizing the robot and platform state, while /LIDAR_marker and /LIDAR_MRS_marker represent the LiDAR sensor's position and status. The /robot_joy and /joy topics handle joystick inputs for manual control, which are translated into velocity commands via /teleop and published on /cmd_vel. The /mcu_imu_gps_servo node manages the servo motor, publishing its position through /servo_pos and /servo_pos_header, enabling LiDAR rotation. LiDAR data is initially published on /cloud and processed by the /PointCloud_transformer node to generate transformed point clouds on /transformed_cloud. The /tf topic integrates transform frames such as /base_link_to_laser and /map_to_odom for localization and mapping, while /odom_data_quat and /imu/data are fused via an EKF to provide accurate odometry on /robot_pose_ekf. The system ensures seamless communication between motion, sensor processing, and visualization components for autonomous navigation.

| ![LiDAR System 1](https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System/blob/master/rqt_graphs/nav_robot_lidar_rotation.png) |
|--------------------------|
|  RQT_GRAPH               |

This TF (Transform Frame) tree visualizes the hierarchical relationships and transformations between different coordinate frames in a ROS-based robot and 3D LiDAR Rotation System.
| ![LiDAR System](https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System/blob/master/images/tf%20tree.png) | ![LiDAR in Action](https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System/blob/master/images/frames3.png) |
|------------------------------------|------------------------------------|
|  TF Tree                           | Frames on Rviz                     |

## Installation and Usage
```bash
# Create a ROS workspace
mkdir ~/catkin_ws
cd ~/catkin_ws

# Clone the repository
git clone https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System.git

# Navigate to the workspace and build the project
catkin_make

# Source the workspace
source devel/setup.bash
```
This project includes a ROS bag file that contains the recorded odometry and LiDAR data from the robot. This bag file can be used to analyze and visualize the robot's movements and sensor readings, allowing users to test algorithms for mapping, localization, and navigation without requiring a physical robot. By replaying the bag file in a ROS environment, users can simulate the robot's behavior and evaluate the integration of odometry and LiDAR data for various applications.

rosbag file
[office1_transformed.bag]() 

Use rosbag terminal tool to replaying the bag file
```bash
rosbag play office1_transformed.bag 
```
Using nav_robot_visualization launch file you can visualize robot's movements and sensor readings
```bash
roslaunch lidar_rotation nav_robot_visualization.launch 
```

## Visualization
| ![LiDAR System](https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System/blob/master/images/odometry.gif) | ![LiDAR in Action](https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System/blob/master/images/robot_visualization.gif) |
|------------------------------------|------------------------------------|
| ![LiDAR System](https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System/blob/master/images/robot_visualization2.gif) | ![LiDAR in Action](https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System/blob/master/images/robot_visualization3.gif) |

## Mapping
| ![LiDAR System 1](https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System/blob/master/images/cloud_compare_animation.gif) |
|--------------------------|
|  Generated Point Cloud Map               |

| ![LiDAR System](https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System/blob/master/images/pcd%20map1.png) | ![LiDAR in Action](https://github.com/omerdurmus61/ROS-Based-Autonomus-Mobile-Robot-with-3D-Mapping-System/blob/master/images/pcd%20map2.png) |
|------------------------------------|------------------------------------|

## Acknowledgment
This project was supported by the **TÜBİTAK 2209-B Industry-Oriented Undergraduate Research Projects Support Program**.

## References
1. ROS Official Documentation: [http://wiki.ros.org](http://wiki.ros.org)  
  
2. TÜBİTAK 2209-B Industry-Oriented Undergraduate Research Projects Support Program  

3. Automatic Addison How to Set Up the ROS Navigation Stack on a Robot: [https://automaticaddison.com/how-to-set-up-the-ros-navigation-stack-on-a-robot/](https://automaticaddison.com/how-to-set-up-the-ros-navigation-stack-on-a-robot/)  
 
4. EKF Localization Reference: [https://wiki.ros.org/robot_pose_ekf](https://wiki.ros.org/robot_pose_ekf)  

5. MRS 1000 3D LiDAR Sensor Documentation: [Documantation](https://www.sick.com/tr/tr/catalog/ueruenler/lidar-ve-radar-sensoerleri/lidar-sensoerleri/mrs1000/mrs1104c-111011/p/p495044)  
 
      
## Contact
For any questions or feedback, please contact:
- Email: [omercandurmuss@gmail.com]
