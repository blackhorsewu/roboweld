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

// values LJ Navigator uses for out-of-range points (in meters)
const static double KEYENCE_INFINITE_DISTANCE_VALUE_SI = -999.9990 / 1e3;
const static double KEYENCE_INFINITE_DISTANCE_VALUE_SI2 = -999.9970 / 1e3;

// local types
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

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

/**
 * @brief Given a @e mon, find a Month text.
 */
char *month_text(int mon)
{
  switch (mon)
  {
    case 0:
      return("Jan");
    case 1:
      return("Feb");
    case 2:
      return("Mar");
    case 3:
      return("Apr");
    case 4:
      return("May");
    case 5:
      return("Jun");
    case 6:
      return("Jul");
    case 7:
      return("Aug");
    case 8:
      return("Sep");
    case 9:
      return("Oct");
    case 10:
      return("Nov");
    case 11:
      return("Dec");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "concave_scanner");
  ros::NodeHandle nh, pnh("~");

  // ros parameters; they are needed for transformation
  std::string sensor_host;
  std::string scanner_frame;
  std::string world_frame;

  double x, y, z;
  double first_y = 0.0;
  double lastY = 0.0;
  double TopZ = 0.0;
  double BotZ = 0.0;
  double thickness = 0.0;
  double volume = 0.0;
  double scanned_length = 0.0;

  // Point Cloud topic
  std::string cloud_topic;
  
  cloud_topic = "profiles"; // The cloud published by the Keyence Driver
  world_frame = "world";
  scanner_frame = "lj_v7200_optical_frame";

  Cloud::Ptr transformed_pcl_Y_cloud(new Cloud);

  // transformed_pcl_Y_cloud->header.frame_id = world_frame;
  // transformed_pcl_Y_cloud->is_dense = false;

  // set up profile cloud publisher
  ros::Publisher pub = nh.advertise<Cloud>("Y_profiles", 100);

  bool write_Y_file = true; // write the whole scan brief data to a file
  bool write_X_file = false; // write data of each scan line to a file

  ofstream Yfile;
  ofstream Xfile;

  int line_no = 0;
  int file_no = 0;
  
  // Visual tools
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
    time_t now = time(0);
    tm *ltm = localtime(&now);

    char name_text[100];
    char day_text[10];
    char hour_text[10];
    char min_text[20];
    int n;

    string file_name;

    sprintf(name_text, "/home/victor/Data/Volume/%d/", 1900+ltm->tm_year);
    if ((n = mkdir(name_text, 0777)) == 0)
    {
      ROS_INFO_STREAM("Folder: " << name_text << " was created.");
    }
    strcat(name_text, month_text(ltm->tm_mon));
    if ((n = mkdir(name_text, 0777)) == 0)
    {
      ROS_INFO_STREAM("Folder: " << name_text << " was created.");
    }
    sprintf(day_text, "/%d/", ltm->tm_mday);
    strcat(name_text, day_text);
    if ((n = mkdir(name_text, 0777)) == 0)
    {
      ROS_INFO_STREAM("Folder: " << name_text << " was created.");
    }
    sprintf(hour_text, "%d/", ltm->tm_hour);
    strcat(name_text, hour_text);
    if ((n = mkdir(name_text, 0777)) == 0)
    {
      ROS_INFO_STREAM("Folder: " << name_text << " was created.");
    }
    sprintf(min_text, "%d-volume.csv", ltm->tm_min);
    ROS_INFO_STREAM("Name text: " << name_text);
    strcat(name_text, min_text);
    ROS_INFO_STREAM("File: " << name_text << " will be created.");
    Yfile.open(name_text);
    ROS_INFO_STREAM("File: " << name_text << " was created.");
    Yfile << "Width, Slice , Area, Plate , 1st Y, Y, Scanned Length, Volume\n";
  }

  /*
   * Listen for Point Cloud - profile from Laser Scanner
   */
  std::string topic = nh.resolveName(cloud_topic);
  ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic " << topic);

  while (ros::ok())
  { // BEGIN Main Loop
    // ros::spin();

    sensor_msgs::PointCloud2::ConstPtr line_cloud (new sensor_msgs::PointCloud2);

    line_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);

    /*
     * Transform the point cloud from Laser Scanner Frame to World Frame
     */
    tf::TransformListener listener;
    tf::StampedTransform stransform;

    sensor_msgs::PointCloud2 transformed_line_cloud;
    Cloud transformed_pcl_line;

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

    pcl::fromROSMsg (transformed_line_cloud, transformed_pcl_line);

    x = stransform.getOrigin().x();
    y = stransform.getOrigin().y();
    z = stransform.getOrigin().z();

    /***********************************************************************************************
     * cross_section_area = cross_section(*transformed_pcl_line, points);                      *
     ***********************************************************************************************/
    Cloud pointcloud = transformed_pcl_line;
    double area = 0.0;
    double zz;
    double height;
    int valid_begin = 0;
    int minDoubleDot1i, minDoubleDot2i; // maxDoubleDoti;
    int cloudSize = pointcloud.size();
    bool first = true;
    bool second = false;
    bool third = false;

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
      string file_name;
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
    ROS_INFO_STREAM("Max DD position: " << maxDoubleDotj << " Line no.: " << line_no);

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

    BotZ = Bot.z;
    TopZ = proj(a, b, Bot).z;
    if (first_y == 0.0)
    {
      first_y = y;
    }

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
          /* Do not publish the Fillers */
        }
      }
    } // End of Third Loop through the Scan Line

    // Publish the culmulated point cloud.
    // *transformed_pcl_Y_cloud += *transformed_pcl_line;

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
    // pub.publish(transformed_pcl_Y_cloud);
    pub.publish(transformed_pcl_line);
    line_no++;
    lastY = y;
  } // END Main Loop
  if (write_Y_file)
  {
    Yfile.close();
  }
  return 0;
} // End of Main
