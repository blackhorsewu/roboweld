#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(-0.4247081, -0.1759199, 0.8204733, -0.3398512), tf::Vector3(0.05025, 0.05025, -0.11427)),
        ros::Time::now(),"ar_marker_2", "d435i_link"));
    r.sleep();
  }
}