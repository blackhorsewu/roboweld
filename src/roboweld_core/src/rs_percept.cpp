/**********************************************************************************
 *                The Chinese National Engineering Research Centre                *
 *                                Steel Construction                              *
 *                                (Hong Kong Branch)                              *
 *                                                                                *
 *                Department of Civil and Environmental Engineering               *
 *                        Hong Kong Polytechnic University                        *
 *                                                                                *
 *                                  Robotic Welding                               *
 *                                                                                *
 * Victor W H Wu                                                                  *
 * 1 January 2021.                                                                *
 *                                                                                *
 * RS Percept node for the Realsense camera to locating and identifying welding   *
 * groove in ROS.                                                                 *
 *                                                                                *
 **********************************************************************************/

/**********************************************************************************
 *                                                                                *
 * This node assumes that the Realsense2 camera package has already been          *
 * running so that it publishes the images and point clouds for use.              *
 * It also assumes the robot arm and the RViz are running with the tf for the     *
 * camera frames are readily available.                                           *
 *                                                                                *
 * It subscribes to point clouds from the Realsense D435i camera.                 *
 *                                                                                *
 **********************************************************************************/



#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

// PCL (Point Cloud Library) specific includes
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"

#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/bilateral.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "geometry_msgs/Point.h"
#include <vector>

// These include files are for Markers to be displayed in rviz
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>

// Visual Tools
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <string>

using namespace std;


/**********************************************************************************
 *                                                                                *
 * The Main of this package starts here.                                          *
 *                                                                                *
 **********************************************************************************/

int main(int argc, char *argv[])
{ // Begin of Main
  /*
   * INITIALIZE ROS NODE
   */
  ros::init(argc, argv, "rs_percept_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");

  /*
   * Set up parameters (Could be input from launch file/terminal)
   */

  /*
   * Parameters for the cloud topic and the reference frames
   */
  std::string cloud_topic, world_frame, camera_frame;

  cloud_topic = priv_nh_.param<std::string>("cloud_topic", "d435i/depth/color/points");
  world_frame = priv_nh_.param<std::string>("world_frame", "world");
  camera_frame = priv_nh_.param<std::string>("camera_frame", "d435i_depth_optical_frame");

  /*
   * we need to specify how much we want to see, ie how to crop the image in the camera 
   * frame. That is before transforming it to the World frame.
   */
  float x_filter_min, x_filter_max, y_filter_min, y_filter_max, 
        z_filter_min, z_filter_max;

  x_filter_min = priv_nh_.param<float>("x_filter_min", -0.20);
  x_filter_max = priv_nh_.param<float>("x_filter_max",  0.20);
  y_filter_min = priv_nh_.param<float>("y_filter_min",  0.0125);
  y_filter_max = priv_nh_.param<float>("y_filter_max",  0.02);
  z_filter_min = priv_nh_.param<float>("z_filter_min",  0.0);
  z_filter_max = priv_nh_.param<float>("z_filter_max",  0.25);

  // local types
  typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

/*
   * Setup publisher to publish ROS point clouds to RViz
   */
  ros::Publisher object_pub;
  object_pub = nh.advertise<sensor_msgs::PointCloud2>("object", 1);

 Cloud::Ptr transformed_pcl_cloud (new Cloud);
 transformed_pcl_cloud->header.frame_id = world_frame;
 // transformed_pcl_cloud->header.stamp=ros::Time::now();
 sensor_msgs::PointCloud2::Ptr transformed_ros_cloud (new sensor_msgs::PointCloud2);
 transformed_ros_cloud->header.frame_id =  world_frame;


 while (ros::ok())
 { // Begin of infinite loop

   /*
    * Listen for point cloud
    */
   std::string topic = nh.resolveName(cloud_topic);
   // ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic "<< topic);

   /*
    * recent_cloud is the raw ROS point cloud received from the camera
    */
   sensor_msgs::PointCloud2::ConstPtr recent_cloud =
                ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);

   /*
    * Convert Point Cloud from ROS format to PCL format
    */
   pcl::PointCloud<pcl::PointXYZ> cloud;

   // Now, cloud is the original cloud in pcl format.

   pcl::PointCloud<pcl::PointXYZ>::Ptr 
                        cloud_ptr(new pcl::PointCloud<pcl::PointXYZ> (cloud));

   pcl::fromROSMsg(*recent_cloud, *cloud_ptr);

   // from here on we are dealing with PCL point cloud

  /*********************************************************************************
   *                                                                               *
   *  PASSTHROUGH FILTER(S)                                                        *
   *                                                                               *
   *  Concentrate on the working area. Crop the image to focus on our intereset,   *
   *  that is the welding groove.                                                  *
   *  We are trying to crop the image from the perspective of the camera.          *
   *  The x direction is pointing to the right of the camera                       *
   *  The y direction is pointing downward of the camera                           *
   *  The z direction is pointing forward of the camera                            *
   *                                                                               *
   *********************************************************************************/

    //filter in x
    pcl::PointCloud<pcl::PointXYZ> xf_cloud, yf_cloud, zf_cloud;
    pcl::PassThrough<pcl::PointXYZ> pass_x;

    pass_x.setInputCloud(cloud_ptr);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_filter_min, x_filter_max);
    pass_x.filter(xf_cloud);

    //filter in y
    pcl::PointCloud<pcl::PointXYZ>::Ptr xf_cloud_ptr
                               (new pcl::PointCloud<pcl::PointXYZ>(xf_cloud));
    pcl::PassThrough<pcl::PointXYZ> pass_y;

    pass_y.setInputCloud(xf_cloud_ptr);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_filter_min, y_filter_max);
    pass_y.filter(yf_cloud);

    //filter in z 
    pcl::PointCloud<pcl::PointXYZ>::Ptr yf_cloud_ptr
                               (new pcl::PointCloud<pcl::PointXYZ>(yf_cloud));
    pcl::PassThrough<pcl::PointXYZ> pass_z;

    pass_z.setInputCloud(yf_cloud_ptr);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_filter_min, z_filter_max);
    pass_z.filter(zf_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr croped_cloud_ptr 
                              (new pcl::PointCloud<pcl::PointXYZ> (zf_cloud));

    // croped_cloud_ptr points to a pcl cloud
    // Needs to convert to ROS format before publishing
    sensor_msgs::PointCloud2::Ptr ros_cloud (new sensor_msgs::PointCloud2);
  
    pcl::toROSMsg(*croped_cloud_ptr, *ros_cloud);

   /*
    * After cropping the cloud, now we want to 
    * Transform PointCloud from Camera Frame to World Frame
    */
    
   tf::TransformListener listener;
   tf::StampedTransform  stransform;
   try
   {
     listener.waitForTransform(world_frame,
                               camera_frame,
                               ros::Time::now(),
                               ros::Duration(0.2));
     listener.lookupTransform (world_frame,
                               camera_frame,
                               ros::Time(0),
                               stransform);
   }
   catch (tf::TransformException ex)
   {
     ROS_ERROR("%s",ex.what());
   }
   sensor_msgs::PointCloud2::Ptr transformed_ros_cloud_local (new sensor_msgs::PointCloud2);
   Cloud::Ptr transformed_pcl_cloud_local (new Cloud);

   pcl_ros::transformPointCloud(world_frame,
                                stransform,
                                *ros_cloud,
                                *transformed_ros_cloud_local);

   pcl::fromROSMsg(*transformed_ros_cloud_local, *transformed_pcl_cloud_local);

   *transformed_pcl_cloud += *transformed_pcl_cloud_local;

   pcl::toROSMsg(*transformed_pcl_cloud, *transformed_ros_cloud);

    object_pub.publish(*transformed_ros_cloud);

  } // End of Infinite loop

  return 0;
} // End of Main
