# RoboWeld
# Introduction
This package is intended to use a Universal Robots UR5 robotic arm to do autonomous Gas Metal Arc Welding (GMAW).
The UR5 is mounted on a 1m x 1.5m welding table. A Fronius Welding Machine is used as the power supply for the welding. The UR5 can use either a banded torch or a straight torch. A Keyence Laser Scanner LJ V7200 is also attached to the welding torch to scan the welding groove before and after the welding.
The system is developed on the ROS platform. It uses the RViz package of ROS to visualise the welding scene. It also tried to use the ROS package MoveIt! to plan and drive the UR5. 

The development was started back in 8 September 2020.
# Implementation details
Before anything can be done, it is necessary to set the scene in a URDF. In ROS, the URDF's are usually put in a package **..._support**. So, in this case there should be a package **roboweld_support**. In the folder **roboweld_support** then should have a folder **urdf**. ROS uses an XML macro language **xacro**. With xacro shorter and more readable XML files can by constructed by using macros that expand it to larger XML expressions.

