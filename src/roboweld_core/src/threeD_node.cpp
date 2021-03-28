#include <cmath>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

#include <ur_msgs/SetIO.h>
#include <ur_msgs/IOStates.h>

float x_filter_min = 0.13;
float x_filter_max = 1.35;

float y_filter_min = -0.50;
float y_filter_max = 0.50;

float z_filter_min = 0.0;
float z_filter_max = 1.25;

bool cloud_enable = false;

ros::Publisher pub; // it will be initialised in main

ros::ServiceClient srv_SetIO;

pcl::PointCloud<pcl::PointXYZRGB> pcl_cumulated_cloud; // cumulated cloud

/*
class RS_cloud // The class
{
  private:
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    std::string world_frame = "world";

  public:
    // Constructors
}
*/

void digital_in_callback(ur_msgs::IOStates iostates)
{
  // ROS_INFO("Hello, in callback: ");
  if (iostates.digital_in_states[1].state)
   cloud_enable = true;
  else
  {
    cloud_enable = false;
  }
  
}

void callback(const sensor_msgs::PointCloud2ConstPtr& rs_cloud)
{
  // Transform the Point Cloud from the camera frame to the World frame.
  tf::TransformListener listener;
  tf::StampedTransform stransform;

  if (cloud_enable)
  {
    ROS_INFO("Cloud Enabled.");
    try
    {
      // Wait for TF to give us the transformation
      listener.waitForTransform("world",
                                rs_cloud->header.frame_id,
                                ros::Time::now(),
                                ros::Duration(0.5));
      listener.lookupTransform ("world",
                                rs_cloud->header.frame_id,
                                ros::Time(0),
                                stransform);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    sensor_msgs::PointCloud2 transformed_cloud;

    // Actually doing the Transformation
    pcl_ros::transformPointCloud("world",
                                stransform,
                                *rs_cloud,
                                transformed_cloud);

    // Convert the ROS Point Cloud into a PCL Point Cloud
    // before PCL filters can be used on it.
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    pcl::fromROSMsg(transformed_cloud, cloud) ;

    // The conversion does not copy the Frame ID and we have to do it explicitly
    pcl_cumulated_cloud.header.frame_id = cloud.header.frame_id;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
                        cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB> (cloud));

    // Crop the Point Cloud to Just Above the Welding Table only
    // to focus our attention.

    pcl::PointCloud<pcl::PointXYZRGB> cropped_cloud;
    pcl::CropBox<pcl::PointXYZRGB> crop;

    crop.setInputCloud(cloud_ptr);

    Eigen::Vector4f min_point = Eigen::Vector4f(x_filter_min, y_filter_min, z_filter_min, 0);
    Eigen::Vector4f max_point = Eigen::Vector4f(x_filter_max, y_filter_max, z_filter_max, 0);

    crop.setMin(min_point);
    crop.setMax(max_point);

    crop.filter(cropped_cloud);

    ROS_INFO("The Cloud cropped.");

    // Concatinate the Local Cloud to the Cumulated Cloud
    // Only when we feel comfortable with what we see
    // Indicated by a Switch Connected to the UR5 Controller

    pcl_cumulated_cloud += cropped_cloud;

    // finish processing it then publish it
    pub.publish(pcl_cumulated_cloud);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "threeD_node");
  ros::NodeHandle nh, pnh("~");

  // set up profile cloud publisher for PCL point clouds
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("cumulated_cloud", 1);

  // Listen for Point Cloud - from d435i

  std::string rs_cloud; // Name of the topic to be subscribed
  std::string ur_data_topic;
  
  rs_cloud = "/d435i/depth/color/points"; // The cloud published by Realsense D435i
  ur_data_topic = "/ur_hardware_interface/io_states";

  std::string rs_cloud_topic = nh.resolveName(rs_cloud);
  // ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic: " << rs_cloud_topic);

  // ros::ServiceClient srv_SetIO = nh.serviceClient<ur_msgs::SetIO> ("ur_driver/set_io");

  // Visual tools
  /*
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().x() = 0.75;
  text_pose.translation().z() = 0.45;
  visual_tools.publishText(text_pose, "RoboWeld Groove Scanning", rvt::WHITE, rvt::XLARGE, false);
  visual_tools.trigger();
 
  char out_text[1500];
  Eigen::Isometry3d out_text_pose = Eigen::Isometry3d::Identity();

  if (write_Y_file)
  {
    OpenSurfaceFile();
  }
  */

  // Subscribe to the rs-cloud topic 
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(rs_cloud_topic, 1, callback);
  ros::Subscriber digit_sub = nh.subscribe<ur_msgs::IOStates>(ur_data_topic, 1, digital_in_callback);
  ROS_INFO_STREAM("Subscribed to IOStates: " << ur_data_topic);

  ros::spin(); // This will keep the whole thing in an infinite loop until ctl-C
  return 0;
}