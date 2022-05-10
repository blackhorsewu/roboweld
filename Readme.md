# RoboWeld
# Introduction
This package is intended to use a Universal Robots UR5 robotic arm to do autonomous Gas Metal Arc Welding (GMAW).
The UR5 is mounted on a 1m x 1.5m welding table. A Fronius Welding Machine is used as the power supply for the welding. The UR5 can use either a banded torch or a straight torch. A Keyence Laser Scanner LJ V7200 is also attached to the welding torch to scan the welding groove before and after the welding.
The system is developed on the ROS platform. It uses the RViz package of ROS to visualise the welding scene. It also tried to use the ROS package MoveIt! to plan and drive the UR5. 

The development was started back in 8 September 2020.
# Implementation details
Before anything can be done, it is necessary to set the scene in a URDF. In ROS, the URDF's are usually put in a package **..._support**. So, in this case there should be a package **roboweld_support**. In the folder **roboweld_support** then should have a folder **urdf**. ROS uses an XML macro language **xacro**. With xacro shorter and more readable XML files can by constructed by using macros that expand it to larger XML expressions.

**Realsense-ROS**
In base_realsense_node.cpp, there are some occurences of **find_if** did not have the std:: prefix. When compiled, the compiler will complain it has not been declared.

**libsocket**
This library is a must for the Keyence Laser Scanner LJ V7200 and must be installed. However, it cannot be put in this **roboweld** workspace because it is not compatible with catkin_make. Therefore, it is necessary to install it in the home directory before compiling roboweld. The source is also backed up in an external hard drive, because the website for it cannot be found anymore!

[libsocket](https://github.com/dermesser/libsocket)

**Keyence Laser Scanner LJ V7200**
There is a ROS driver for it - [keyence experimental](https://github.com/ros-industrial/keyence_experimental). There the driver proper **keyence_driver_node** which will startup the driver, provided the ip address of the scanner controller is supplied as a parameter. 

It is deceided not to touch the source of this node. Any usage of the scanner should be done by subscribing to the point cloud topic (**/profiles**) published by it. This will separate the development of this project and the updates to this node by the driver developer. 

# Change log
5 May 2022.
Added a moveit config with the name **stkey_moveit_config**. Here the **stkey** is to indicate that the Keyence Laser Scanner is straight as oppose to one going to construct with a tilted scanner.