#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>

#include <limits>

#include "boost/bind.hpp"
#include "boost/ref.hpp"

// Visualisation markers
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>

// Visual Tools
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <string>
#include <iostream>
#include <fstream>

#include <ctime>

using namespace std;

// local types
// typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2ConstPtr& ros_cloud)
{
   pub.publish(ros_cloud);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "concave_scanner");
  ros::NodeHandle nh, pnh("~");

  // ros parameters; they are needed for transformation
  std::string sensor_host;
  std::string scanner_frame;
  std::string world_frame;

  // Point Cloud topic
  std::string cloud_topic;
  
  cloud_topic = "profiles"; // The cloud published by the Keyence Driver
  world_frame = "world";
  scanner_frame = "lj_v7200_optical_frame";

  // set up profile cloud publisher
  // ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("Y_profiles", 100);
  pub = nh.advertise<sensor_msgs::PointCloud2>("Y_profiles", 1);

  /*
   * Listen for Point Cloud - profile from Laser Scanner
   */
  std::string topic = nh.resolveName(cloud_topic);
  ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic " << topic);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(topic, 1, callback);

  ros::spin();
  return 0;
}
