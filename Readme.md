# RoboWeld
# Introduction
This package is intended to use a Universal Robots UR5 robotic arm to do semi-autonomous Gas Metal Arc Welding (GMAW).
The UR5 is mounted on a 1m x 1.5m welding table. A Fronius Welding Machine is used as the power supply for the welding.
The UR5 can use either a bended torch or a straight torch.
An Intel Realsense D435i camera is also attached to the welding torch holder to provide both Colour images and Point Clouds
of the welding scene.
Operators are exptected to move the End Effector of the UR5, in FreeDrive mode, manually to see the workpiece.
The system is developed on the ROS platform. It uses the RViz package of ROS to visualise the welding scene.

This change of approach was started on the 25th June, 2022.

# Implementation details
Before anything can be done, it is necessary to set the scene in a URDF. 
In ROS, the URDF's are usually put in a package **..._support**. 
So, in this case there should be a package **roboweld_support**. 
In the folder **roboweld_support** then should have a folder **urdf**. 
ROS uses an XML macro language **xacro**. With xacro shorter and more 
readable XML files can by constructed by using macros that expand it to 
larger XML expressions.

**Realsense-ROS**
In base_realsense_node.cpp, there are some occurences of **find_if** 
did not have the std:: prefix. When compiled, the compiler will complain 
it has not been declared.

