#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>

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

// values LJ Navigator uses for out-of-range points (in meters)
const static double KEYENCE_INFINITE_DISTANCE_VALUE_SI = -999.9990 / 1e3;
const static double KEYENCE_INFINITE_DISTANCE_VALUE_SI2 = -999.9970 / 1e3;

ros::Publisher pub; // it will be initialised in main

pcl::PointCloud<pcl::PointXYZ> pcl_Y_cloud; // cumulated cloud

// needed for transformation from scanner frame to world frame
std::string sensor_host;
std::string scanner_frame;
std::string world_frame;

bool write_Y_file = false; // write the whole scan brief data to a file
bool write_X_file = false; // write data of each scan line to a file

ofstream Yfile;
ofstream Xfile;

int line_no = 0;
int file_no = 0;
  
int valid_begin, valid_end;

int maxDoubleDotj;

/* 
 * Average Differences
 */
void average_differences(pcl::PointCloud<pcl::PointXYZ> pointcloud)
{
  /* code here */
  int w = 15; // Number of points to take Average
  int m = w/2; // Half of no. of average points
  int j = 1;

  int cloudSize = pointcloud.size();

  double sumz = 0.0;
  double sumdavgz = 0.0;
  double sumdavgdavgz = 0.0;

  double Z[cloudSize], avgz[cloudSize], avgdavgz[cloudSize], avgddavgz[cloudSize];
  double davgz[cloudSize], davgdavgz[cloudSize];

  double minDoubleDot1Z = 0.0;
  double minDoubleDot2Z = 0.0;
  double maxDoubleDotZ = 0.0;

  double zz;

  bool first = true;
  bool second = false;
  bool third = false;

  for (int i = 0; i < (cloudSize - 50); ++i)
  {
    zz = pointcloud[i].z;
    if ((zz != KEYENCE_INFINITE_DISTANCE_VALUE_SI) && (zz != KEYENCE_INFINITE_DISTANCE_VALUE_SI2)
          && (zz != std::numeric_limits<double>::infinity()))
    {   // then this is a piece of normal data
      if (first) { first = false; second = true; }
      else if (second) 
      { 
        second = false;
        valid_begin = i+1;
      }
      else
      {
        if (j < (w - m)) // from 1 to m
        {                                                          //////////////// j = 1 to m /////
          Z[j] = zz;
          sumz += Z[j];                          // cumulate sumz from Z[1] to Z[m]
          if (write_X_file)
          {
            Xfile << j << ", " << zz << ", \n";   // write only j and Z
          }
        }
        else if ((j >= (w - m)) && (j < w))                         ///////////////// j = m+1 to w-1 ////
        {
          Z[j] = zz;    
          sumz += Z[j];                          // carry on cumulate sumz but do NOT write them out
        }
        else if (j == w)                                            ////////////////// j = w //////
        {
          Z[j] = zz;
          sumz += Z[j];                          // sumz cumulated from Z[1] to Z[w]
          avgz[j-m] = sumz / w;                  // average and put into avgz[j-m]
          if (write_X_file)
          {
            Xfile << (j-m) << ", " << Z[j-m] << ", " << avgz[j-m] << ", \n"; // WRITE Z[j-m] and avgz[j-m] to file
          }
          sumz -= Z[(j-w)+1];                    // sumz subtract Z[(j-w)+1], the first cumulated value
        }
        else if ((j > w) && (j <= (w+m)))                          //////////////////  j = w+1 to w+m //// 
        {
          Z[j] = zz;
          sumz += Z[j];                          // cumulate sumz from Z[w+1] to Z[w+m]
          avgz[j-m] = sumz / w;                  // average at j-m
          davgz[j-m] = avgz[j-m] - avgz[(j-m)-1];// difference between averages of Z[j-m] and [(j-m)-1]
          sumdavgz += davgz[j-m];                // cumulate the difference
          if (write_X_file)
          {
            Xfile << (j-m) << ", " << Z[j-m] << ", " << avgz[j-m] << ", " << davgz[j-m] << ", \n";
          }
          sumz -= Z[(j-w)+1];                    // sumz subtract Z[(j-w)+1], the first cumulated value
        } 
        else if (j == ((w+m)+1))                                   /////////////////// j = (w+m)+1
        {
          Z[j] = zz;
          sumz += Z[j];                          // sumz cumulated from Z[w+1] to Z[(w+m)+1]
          avgz[j-m] = sumz / w;                  // average at j-m
          davgz[j-m] = avgz[j-m] - avgz[(j-m)-1];// difference between averages of Z[j-m] and Z[(j-m)-1]
          sumdavgz += davgz[j-m];                // cumulate the difference
          sumz -= Z[(j-w)+1];                    // sumz subtract Z[(j-w)+1], the first cumulated value
        }
        else if ((j > ((w+m)+1)) && (j < (((w+m)+m)+1) ))          ///////////////// j = ((w+m)+1)+1 to (((w+m)+m)+1)
        {
          Z[j] = zz;
          sumz += Z[j];                          // cumulate sumz from Z[((w+m)+1)+1] to Z[(w+m)+m]
          avgz[j-m] = sumz/w;                    // average avgz[11] / j=17; avgz[13]
          davgz[j-m] = avgz[j-m] - avgz[(j-m)-1];// difference between averages of Z[j-m] and Z[(j-m)-1]
          sumdavgz += davgz[j-m];                // cumulate the difference
          sumz -= Z[(j-w)+1];                    // sumz subtract Z[(j-w)+1], the first cumulated value
        }
        else if (j==(((w+m)+m)+1))                                  ////////////////// j = ((w+m)+m)+1
        {
          Z[j] = zz;
          sumz += Z[j];                          // sum Z[10] to Z[18]
          avgz[j-m] = sumz/w;                    // avgz[14] = sumz / 9
          davgz[j-m] = avgz[j-m] - avgz[(j-m)-1];// davgz[14]=avgz[14]-avgz[13]
          sumdavgz += davgz[j-m];                // sumdavgz += davgz[14] ; sumdavgz[6] to sumdavgz[14]
          avgdavgz[(j-m)-m] = sumdavgz / w;      // avgdavgz[10]
          if (write_X_file)
          {
            Xfile << ((j-m)-m) << ", " << Z[(j-m)-m] << ", " << avgz[(j-m)-m] << ", " << davgz[(j-m)-m] << ", "  << avgdavgz[(j-m)-m] << "\n";
          }
          sumdavgz -= davgz[((j-w)-m)+1];        //  Need to subtact davgz[6]; j=18, w=9, m=4; j-w-m+1 = 6
          sumz -= Z[(j-w)+1];                    // subtract Z[10]
        }
        else if ((j>(((w+m)+m)+1)) && (j<(((((w+m)+m)+m)+1)+1)))
        {
          Z[j] = zz;
          sumz += Z[j];
          avgz[j-m] = sumz/w;
          davgz[j-m] = avgz[j-m] - avgz[j-m-1];
          sumdavgz += davgz[j-m];
          avgdavgz[(j-m)-m] = sumdavgz / w;
          davgdavgz[(j-m)-m] = avgdavgz[(j-m)-m] - avgdavgz[((j-m)-m)-1];
          sumdavgdavgz += davgdavgz[(j-m)-m];
          if (write_X_file)
          {
            Xfile << ((j-m)-m) << ", " << Z[(j-m)-m] << ", " << avgz[(j-m)-m] << ", " << davgz[(j-m)-m] << ", "  << avgdavgz[(j-m)-m] << 
                  ", " << davgdavgz[(j-m)-m] << "\n ";
          }
          sumdavgz -= davgz[((j-w)-m)+1];
          sumz -= Z[(j-w)+1];
        }
        else if ((j>=((((((w+m)+m)+m)+1)+1))) && (j < ((((((w+m)+m)+m)+m)+1)+1)))
        {
          Z[j] = zz;
          sumz += Z[j];
          avgz[j-m] = sumz/w;
          davgz[j-m] = avgz[j-m] - avgz[j-m-1];
          sumdavgz += davgz[j-m];
          avgdavgz[(j-m)-m] = sumdavgz / w;
          davgdavgz[(j-m)-m] = avgdavgz[(j-m)-m] - avgdavgz[((j-m)-m)-1];
          sumdavgdavgz += davgdavgz[(j-m)-m];
          sumdavgz -= davgz[((j-w)-m)+1];
          sumz -= Z[(j-w)+1];
        }
        else if (j >= ((((((w+m)+m)+m)+m)+1)+1))
        {
          Z[j] = zz;
          sumz += Z[j];
          avgz[j-m] = sumz/w;
          davgz[j-m] = avgz[j-m] - avgz[j-m-1];
          sumdavgz += davgz[j-m];
          avgdavgz[(j-m)-m] = sumdavgz / w;
          davgdavgz[(j-m)-m] = avgdavgz[(j-m)-m] - avgdavgz[((j-m)-m)-1];
          sumdavgdavgz += davgdavgz[(j-m)-m];
          avgddavgz[((j-m)-m)-m] = sumdavgdavgz / w;
          if (avgddavgz[((j-m)-m)-m] > maxDoubleDotZ)
          {
            maxDoubleDotZ = avgddavgz[((j-m)-m)-m];
            maxDoubleDotj = ((j-m)-m)-m;
          }
          if (write_X_file)
          {
            Xfile << (((j-m)-m)-m) << ", " << Z[((j-m)-m)-m] << ", " << avgz[((j-m)-m)-m] << ", " << davgz[((j-m)-m)-m] << ", "  << avgdavgz[((j-m)-m)-m] << 
                  ", " << davgdavgz[((j-m)-m)-m] << ", " << avgddavgz[((j-m)-m)-m] << "\n";
          }
          sumdavgdavgz -= davgdavgz[(((j-w)-m)-m)+1];
          sumdavgz -= davgz[((j-w)-m)+1];
          sumz -= Z[(j-w)+1];
        }
        j++;
      }
    }
    else if (i >= (cloudSize - 50))
    {
      valid_end = i;
    }
  } // end of First Loop through the Scan Line
  if (write_X_file)
  {
    Xfile.close();
    file_no++;
  }

}

/*
double width(pcl::PointCloud<pcl::PointXYZ> pointcloud)
{
  bool first = true;
  bool second = false;
  bool third = false;

  int minDoubleDot1i, minDoubleDot2i;

  for (int i = valid_begin; i < valid_end; ++i)
  { // Second Loop through the Scan Line
    zz = pointcloud[i].z;

    if ((zz != KEYENCE_INFINITE_DISTANCE_VALUE_SI) && (zz != KEYENCE_INFINITE_DISTANCE_VALUE_SI2)
          && (zz != std::numeric_limits<double>::infinity()))
    {   // then this is a piece of normal data
      if (first) // from 1st onward
      {
        first = false;
        second = true;
      } 
      else if (second)
      {
        second = false;
        third = true;
      } 
      else if (third)
      {
        third = false;
      }
      else
      {
        if ((i < (maxDoubleDotj-10)) && (avgddavgz[i] < minDoubleDot1Z))
        {
          minDoubleDot1Z = avgddavgz[i];
          minDoubleDot1i = i;
        }
        if ((i > (maxDoubleDotj+10)) && (avgddavgz[i] < minDoubleDot2Z))
        {
          minDoubleDot2Z = avgddavgz[i];
          minDoubleDot2i = i;
        }
      }
    }
  } // End of Second Loop through the Scan Line
}
*/

/*
 * Process a scan line.
 */
void process_line(pcl::PointCloud<pcl::PointXYZ> pointcloud)
{
  /* Pass through the line 3 times */
  /* 1. work out the averaged differences between points */
  average_differences(pointcloud);
  /* 2. find the width of the Groove */
  // double groove_width = width(pointcloud);
  /* 3. work out the cross-section area of the groove */
}

/*
 * This handles one scan line.
 */
void callback(const sensor_msgs::PointCloud2ConstPtr& ros_cloud)
{
  // The publisher is initialised in main
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

  // Only ros cloud can be transformed!
  tf::TransformListener listener;
  tf::StampedTransform  stransform;
  try
  {
    listener.waitForTransform(world_frame,
                              ros_cloud->header.frame_id,
                              ros::Time::now(),
                              ros::Duration(0.25));
    listener.lookupTransform (world_frame,
                              ros_cloud->header.frame_id,
                              ros::Time(0),
                              stransform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }

  sensor_msgs::PointCloud2 transformed_ros_cloud;
  pcl_ros::transformPointCloud("world",
                               stransform,
                               *ros_cloud,
                               transformed_ros_cloud);
  /*
   * pcl cloud is used because the operator += cannot work with ros cloud!
   */
  pcl::fromROSMsg(transformed_ros_cloud, pcl_cloud);
  pcl_Y_cloud.header.frame_id = pcl_cloud.header.frame_id; // cannot be done away with; must keep
  pcl_Y_cloud += pcl_cloud;
  pub.publish(pcl_Y_cloud);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "concave_scanner");
  ros::NodeHandle nh, pnh("~");

  // Point Cloud topic
  std::string cloud_topic;
  
  cloud_topic = "profiles"; // The cloud published by the Keyence Driver
  world_frame = "world";
  scanner_frame = "lj_v7200_optical_frame";

  // set up profile cloud publisher for PCL point clouds
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("Y_profiles", 1);

  /*
   * Listen for Point Cloud - profile from Laser Scanner
   */
  std::string topic = nh.resolveName(cloud_topic);
  ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic " << topic);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(topic, 1, callback);

  ros::spin();
  return 0;
}
