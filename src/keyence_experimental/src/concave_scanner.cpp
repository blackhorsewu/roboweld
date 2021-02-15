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
// #include <iostream>
// #include <fstream>

#include <ctime>

using namespace std;

// values LJ Navigator uses for out-of-range points (in meters)
const static double KEYENCE_INFINITE_DISTANCE_VALUE_SI = -999.9990 / 1e3;
const static double KEYENCE_INFINITE_DISTANCE_VALUE_SI2 = -999.9970 / 1e3;

// default values for parameters
const static std::string DEFAULT_FRAME_ID = "sensor_optical_frame";
const static std::string DEFAULT_WORLD_FRAME = "world";

// local types
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

// Global variable
// double global_x_increment;

// Prototype for function that handles the cross section of the groove; returns the cross
// section area.
// double cross_section(Cloud local_pc, geometry_msgs::Point line_list); // Input is the local point cloud in PCL format

/**
 * @brief Given a @e a, and a @e b, both are vector of 3 doubles, 
 * return the dot product.
 */
double dot(geometry_msgs::Point a, geometry_msgs::Point b)
{
  return(a.x*b.x + a.y*b.y + a.z*b.z);
}

/**
 * @brief Given a @e a, and a @e b, find the length between them
 */
double length(geometry_msgs::Point a, geometry_msgs::Point b)
{
  return(sqrt(pow((b.x - a.x), 2) + pow((b.y - a.y), 2) + pow((b.z - a.z), 2)));
}

/**
 * @brief Given a @e a, and a @e b, find the mid-point
 */
geometry_msgs::Point mid(geometry_msgs::Point a, geometry_msgs::Point b)
{
  geometry_msgs::Point mid_pt;

  mid_pt.x = (b.x + a.x)/2.0;
  mid_pt.y = (b.y + a.y)/2.0;
  mid_pt.z = (b.z + a.z)/2.0;

  return(mid_pt);
}

/**
 * @brief Given a @e a, a @e b, and a @e p, find the projection of p on line a b
 */
geometry_msgs::Point proj(geometry_msgs::Point a, geometry_msgs::Point b, geometry_msgs::Point p)
{
  geometry_msgs::Point ap, ab;
  geometry_msgs::Point result;
  double ratio;
  
  ap.x = p.x-a.x;  ab.x = b.x-a.x;
  ap.y = p.y-a.y;  ab.y = b.y-a.y;
  ap.z = p.z-a.z;  ab.z = b.z-a.z;
  
  ratio = dot(ap, ab)/dot(ab, ab);

  result.x = a.x + ratio*ab.x;
  result.y = a.y + ratio*ab.y;
  result.z = a.z + ratio*ab.z;

  return result;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "concave_scanner");
  ros::NodeHandle nh, pnh("~");

  // ros parameters; they are needed for transformation
  std::string sensor_host;
  std::string scanner_frame;
  std::string world_frame;
  double sample_rate;


  double x, y, z;
  double first_y = 0.0;
  double lastY = 0.0;
  double TopZ = 0.0;
  double BotZ = 0.0;
  double thickness = 0.0;
  double volume = 0.0;
  double scanned_length = 0.0;

  pnh.param<std::string>("frame_id", scanner_frame, DEFAULT_FRAME_ID);
  pnh.param<std::string>("world_frame", world_frame, DEFAULT_WORLD_FRAME);

  // Point Cloud topic
  std::string cloud_topic;
  
  cloud_topic = "profiles";
  world_frame = "world";
  scanner_frame = "lj_v7200_optical_frame";

  // Cloud::Ptr pc_msg(new Cloud); 
  // I renamed it as line_cloud but it is a ROS cloud

  // Another point cloud transformed into the world frame
  // Cloud::Ptr transformed_pc_msg(new Cloud);
  Cloud::Ptr transformed_pcl_Y_cloud(new Cloud);

  // setup point Marker message
  visualization_msgs::MarkerArray fillers;
  /*
  fillers.header.frame_id = world_frame;
  fillers.header.stamp = ros::Time::now();
  fillers.ns = "fillers";
  fillers.action = visualization_msgs::Marker::ADD;
  fillers.pose.orientation.w = 1.0; // Quarternion
  fillers.id = 0;
  fillers.type = visualization_msgs::Marker::LINE_LIST;
  fillers.scale.x = 0.0001; // so the line is shown as of 0.1mm wide
  fillers.color.r = 1;   // in red
  fillers.color.a = 1;   // 
  */
  geometry_msgs::Point points;

  // set up fillers Marker publisher
  ros::Publisher fillers_pub = nh.advertise<visualization_msgs::MarkerArray>("fillers", 400);

  
  /*
  pc_msg->header.frame_id = frame_id;
  pc_msg->is_dense = false; // cloud could have NaNs
  // message is essentially a line-strip of points
  pc_msg->height = 1;
  */

  transformed_pcl_Y_cloud->header.frame_id = world_frame;
  transformed_pcl_Y_cloud->is_dense = false;

  // set up profile cloud publisher
  ros::Publisher pub = nh.advertise<Cloud>("Y_profiles", 100);

  bool write_Y_file = false; // write the whole scan brief data to a file
  bool write_X_file = false; // write data of each scan line to a file

  bool active_flag = false;
  int line_no = 0;
  int file_no = 0;
  int filler_id = 0;
  int marker_id = 1;
  
  string file_name;
  ofstream Yfile;
  /*
  time_t now = time(0);
  tm *ltm = localtime(&now);
  char name_text[1200];
  */
  while (ros::ok())
  {
    // double cross_section_area = 0.0;
    bool first_line = true;

    // Visual tools
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().x() = 0.75;
    text_pose.translation().z() = 0.45;
    visual_tools.publishText(text_pose, "RoboWeld Groove Scanning", rvt::WHITE, rvt::XLARGE, false);
    visual_tools.trigger();

    Eigen::Isometry3d arrow_pose;
    // rotate along X axis by 45 degrees
    arrow_pose = Eigen::AngleAxisd(-M_PI/4, Eigen::Vector3d::UnitY());

    // Set Marker Lifetime
    // visual_tools.setLifetime(0.05);
    char out_text[1500];
    Eigen::Isometry3d out_text_pose = Eigen::Isometry3d::Identity();

    if (write_Y_file)
    {
      file_name = "/home/victor/Data/Volume/2021/February/volume.csv";
      Yfile.open(file_name);
      Yfile << "Width, Slice , Area, Plate , 1st Y, Y, Scanned Length, Volume\n";
    }

    // ros::Rate sleeper (100);
    // Main loop
    // sleeper.reset();
    while (ros::ok())
    { // BEGIN Main Loop
      // sleeper.sleep();
      ros::spinOnce();

      /*
        * Listen for Point Cloud - profile from Laser Scanner
        */
      std::string topic = nh.resolveName(cloud_topic);
      // ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic " << topic);

      /*
        * line_cloud is the raw point cloud received from the Laser Scanner
        */
      sensor_msgs::PointCloud2::ConstPtr line_cloud // this cloud is just a line
        = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);

      // ROS_INFO("Received a point cloud... ");
      // pub.publish(line_cloud);
      /*
        * Transform the point cloud from Laser Scanner Frame to World Frame
        */
      tf::TransformListener listener;
      tf::StampedTransform stransform;

      sensor_msgs::PointCloud2 transformed_line_cloud;
      Cloud::Ptr transformed_pcl_line (new Cloud);

      //  sensor_msgs::PointCloud2 cloud_in; // Now, I call it line_cloud

      try
      {
        listener.waitForTransform(world_frame,
                                  scanner_frame,
                                  ros::Time::now(),
                                  ros::Duration(0.6)); // Frequency of scan ** It seems to be working fine when 1.0 //

        listener.lookupTransform (world_frame,
                                  scanner_frame,
                                  ros::Time(0),
                                  stransform);
      }
      catch(tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }

      pcl_ros::transformPointCloud(world_frame,
                                   stransform,
                                   *line_cloud,
                                   transformed_line_cloud);

      pcl::fromROSMsg (transformed_line_cloud, *transformed_pcl_line);

      x = stransform.getOrigin().x();
      y = stransform.getOrigin().y();
      z = stransform.getOrigin().z();

      /***********************************************************************************************
       * cross_section_area = cross_section(*transformed_pcl_line, points);                      *
       ***********************************************************************************************/
      Cloud pointcloud = *transformed_pcl_line;
      double area = 0.0;
      double zz, lastZ;
      double dotZ, lastDotZ;
      double doubleDotZ; // lastDoubleDiffZ;
      double height, Height;
      int valid_begin = 0;
      int minDoubleDot1i, minDoubleDot2i; // maxDoubleDoti;
      int cloudSize = pointcloud.size();
      bool first = true;
      bool second = false;
      bool third = false;
      ofstream Xfile;

      // Line list pose
      // geometry_msgs::Point p;
  
      using namespace std;
      int w = 15;     // Number of points to take Average 
      int m = w / 2;  // Half of no. of average points
      int j = 1;
      int maxDoubleDotj;
      double sumz = 0.0;
      double sumdavgz = 0.0;
      double sumdavgdavgz = 0.0;
      double Z[cloudSize], avgz[cloudSize], avgdavgz[cloudSize], avgddavgz[cloudSize];
      double davgz[cloudSize], davgdavgz[cloudSize];

      if (write_X_file)
      {
        file_name = "/home/victor/Data/profile/2021/February/profile_" + std::to_string(file_no) + ".csv";
        Xfile.open(file_name);
        Xfile << "J, Z, Average Z, D Avg Z, Avg D Avg Z, D Avg D Avg Z, Avg DD Avg Z\n";
      }

      /********************************************************************************************
       * Going through the scanned line data for the FIRST time.                                  *
       * Should be able to work out the Gradient and the Derivative of the Gradient, and also the *
       * Maximum value of the Derivative of the Gradient, that is the deepest point of the Groove.*
       ********************************************************************************************/
      double minDoubleDot1Z = 0.0;
      double minDoubleDot2Z = 0.0;
      double maxDoubleDotZ = 0.0;

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
      } // end of First Loop through the Scan Line
      if (write_X_file)
      {
        Xfile.close();
        file_no++;
      }

      // ROS_INFO_STREAM("X step: " << global_x_increment * 1e3 << " mm");
      ROS_INFO_STREAM("Valid at: " << valid_begin << "; 1st y: " << first_y * 1e3);
      ROS_INFO_STREAM("Max DD position: " << maxDoubleDotj << " File no.: " << file_no - 1);

      /******************************************************************************************** 
       * Go through the data points the SECOND time. Next find the LEFT and RIGHT Min double d    *
       ********************************************************************************************/
      for (int i = valid_begin; i < (cloudSize - 50); ++i)
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

      ROS_INFO_STREAM("Left Min DD position: " << minDoubleDot1i);
      ROS_INFO_STREAM("Right Min DD position: " << minDoubleDot2i);

      /********************************************************************************************
       * After the three turning points are found, go through the data points THIRD time!         *
       * (1) Work out the cross-section area                                                      *
       * (2) Fill in the Marker details in preparation to publish the markers but NOT until the   *
       *     first line is finished then publish the fillers for the whole line outside the loop  *
       ********************************************************************************************/
      geometry_msgs::Point a, b, Top, Bot, Mid;
      int begin = minDoubleDot1i + valid_begin;
      int end = minDoubleDot2i + valid_begin;
      double last_zB; // if inf then use last zB
      double base; // in metre
      double lastX, xIncrement;
      // height;

      a.x = pointcloud[begin].x;
      a.y = pointcloud[begin].y;
      a.z = pointcloud[begin].z;

      b.x = pointcloud[end].x;
      b.y = pointcloud[end].y;
      b.z = pointcloud[end].z;


      base = length(a, b);

      Bot.x = pointcloud[maxDoubleDotj + valid_begin].x;
      Bot.y = pointcloud[maxDoubleDotj + valid_begin].y;
      Bot.z = pointcloud[maxDoubleDotj + valid_begin].z;

      y = Bot.y; // This is the y on the bottom of the steel plate Groove
      BotZ = Bot.z;
      TopZ = proj(a, b, Bot).z;
      if (first_y == 0.0)
      {
        first_y = y;
      }

      // y is known before entering into the Loop Unless this is the first line
      if (line_no != 0)
      {
        thickness = (y - lastY) * cos(0.471233898038469); // Slice thickness is in Metre; it has slanted by 0.47 radian
        scanned_length += thickness;
      }

      lastX = a.x;
      for (int i = begin; i <= end; ++i)
      { // Third Loop through the Scan Line
        // Work out the cross-section area regardless if this is the first line
        Bot.x = pointcloud[i].x;
        Bot.y = pointcloud[i].y;
        Bot.z = pointcloud[i].z;

        xIncrement = abs(Bot.x - lastX);
        lastX = Bot.x;

        if ((Bot.z == KEYENCE_INFINITE_DISTANCE_VALUE_SI) || (Bot.z == KEYENCE_INFINITE_DISTANCE_VALUE_SI2)
              || (Bot.z == std::numeric_limits<double>::infinity()))
        { // if zB is inf then use last zB
          Bot.z = last_zB;
        }
        else // keep zB as last zB
        {
          last_zB = Bot.z;
            
          Top = proj(a, b, Bot); // The top point is the Bottom project onto the top line ab.
          height = length(Top, Bot);

          area += height * xIncrement; // both height and x_increment are in Metre, therefore area in Metre square

          if (((line_no != 0) && (height != 0.0) && (thickness != 0.0)) &&
             ((Bot.z != KEYENCE_INFINITE_DISTANCE_VALUE_SI) && (Bot.z != KEYENCE_INFINITE_DISTANCE_VALUE_SI2)
              && (Bot.z != std::numeric_limits<double>::infinity())))
          { 
            visualization_msgs::Marker filler;
            // Fill the Markers fixed parts first
            filler.header.frame_id = world_frame;
            filler.ns = "fillers";
            filler.action = visualization_msgs::Marker::ADD;
            filler.pose.orientation.x = stransform.getRotation().x();
            filler.pose.orientation.y = stransform.getRotation().y();
            filler.pose.orientation.z = stransform.getRotation().z();
            filler.pose.orientation.w = stransform.getRotation().w();
            filler.type = visualization_msgs::Marker::CUBE;
            filler.color.r = 1;
            filler.color.a = 1;
            filler.id = marker_id++;

            filler.scale.x = 0.0001; // that is 0.1 mm
            filler.scale.y = thickness;
            filler.scale.z = height;

            // location for the centre of the column
            Mid = mid(Top, Bot);
            filler.pose.position.x = Mid.x;
            filler.pose.position.y = Mid.y - thickness/2.0;
            filler.pose.position.z = Mid.z;

            fillers.markers.push_back(filler);
            // fillers_pub.publish(filler);
          }
        }
      } // End of Third Loop through the Scan Line

      // Publish the culmulated point cloud.
      *transformed_pcl_Y_cloud += *transformed_pcl_line;

      area = area * cos(0.471233898038469);
      volume += area * thickness;
      ROS_INFO_STREAM("Width: " << base * 1e3 << " mm");
      ROS_INFO_STREAM("Slice Thickness: " << thickness * 1e3 << " mm");
      ROS_INFO_STREAM("Scanned Length: " << scanned_length * 1e3 << " mm");
      ROS_INFO_STREAM("Y: " << y * 1e3);
      ROS_INFO_STREAM("TopZ: " << TopZ*1e3 << " BotZ: " << BotZ*1e3 << " Plate: " << (TopZ - BotZ) * 1e3);
      ROS_INFO_STREAM("Area: " << area * 1e6 << " mm2");
      ROS_INFO_STREAM("Volume: " << volume * 1e9 << " mm3\n");

      if (write_Y_file)
      {
        Yfile << base * 1e3 << ", " << thickness * 1e3 << ", " << area * 1e6 << ", " << (TopZ-BotZ) * 1e3 << ", " << first_y * 1e3 << ", " 
              << y * 1e3 << ", " << scanned_length * 1e3 << ", " << volume * 1e9 << "\n";
      }

      int n = sprintf(out_text, 
                  "Cross Section Area: %5.2f mm2\nThickness of slice: %5.2f mm\nScanned length: %5.2f mm\nVolume of Groove: %5.2f mm3\n",
                  area * 1e6,
                  thickness * 1e3,
                  scanned_length * 1e3,
                  volume * 1e9
                 );

      out_text_pose.translation().x() = x + 0.15;
      out_text_pose.translation().y() = y;
      out_text_pose.translation().z() = z + 0.1;

      visual_tools.publishText(out_text_pose, 
                               out_text, 
                               rvt::YELLOW,
                               rvt::LARGE,
                               1
                              );
      visual_tools.trigger();  /************ Switching the display ON and OFF here. *****************/

      // publish pointcloud
      pub.publish(transformed_pcl_Y_cloud);
      line_no++;
      lastY = y;
    } // finish scanning a good line
    fillers_pub.publish(fillers);
  } // END main loop

  if (write_Y_file)
  {
    Yfile.close();
  }
  return 0;
}
