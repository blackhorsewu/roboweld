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

bool write_Y_file = false; // write the whole scan brief data to a file
bool write_X_file = true; // write data of each scan line to a file

double first_y = 0.0;
double lastY = 0.0;
double y = 0.0;
double scanned_length = 0.0;
double volume = 0.0;

ofstream Yfile;
ofstream Xfile;

int line_no = 0;
int file_no = 0;

pcl::PointCloud<pcl::PointXYZ> pcl_Y_cloud; // cumulated cloud

// Given a, b, find the length between them
double length(geometry_msgs::Point a, geometry_msgs::Point b)
{
  return(sqrt(pow((b.x - a.x), 2) + pow((b.y - a.y), 2) + pow((b.z - a.z), 2)));
}

// Given a, b, both are vector of 3 doubles, return the DOT product.
double dot(geometry_msgs::Point a, geometry_msgs::Point b)
{
  return(a.x*b.x + a.y*b.y + a.z*b.z);
}

// Given a, b, and p, find the projection of p on line ab.
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

// Given a mon return the name of the month.
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

void OpenSurfaceFile()
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
  Yfile << "Width, Slice , Area, 1st Y, Y, Scanned Length, Volume\n";
}

void OpenLineFile()
{
  time_t now = time(0);
  tm *ltm = localtime(&now);

  char name_text[100];
  char day_text[10];
  char hour_text[10];
  char min_text[20];
  int n;

  string file_name;

  sprintf(name_text, "/home/victor/Data/Profile/%d/", 1900+ltm->tm_year);
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
  sprintf(min_text, "%d-profile_%d.csv", ltm->tm_min, file_no);
  // ROS_INFO_STREAM("Name text: " << name_text);
  strcat(name_text, min_text);
  // ROS_INFO_STREAM("File: " << name_text << " will be created.");
  Xfile.open(name_text);
  // ROS_INFO_STREAM("File: " << name_text << " was created.");
  Xfile << "J, Z, Average Z, D Avg Z, Avg D Avg Z, D Avg D Avg Z, Avg DD Avg Z\n";
}

class Line_cloud  // The class
{
  private:
    // private attribute
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    double minDoubleDot1Z = 0.0;
    double minDoubleDot2Z = 0.0;
    double maxDoubleDotZ = 0.0;

    int minDoubleDot1i, minDoubleDot2i;
    int valid_begin, valid_end = 0;

    int maxDoubleDotj;

    std::string world_frame = "world";


    double avgddavgz[]; // flexible array member must be at end of class!
  public:
    // Constructor
    Line_cloud(const sensor_msgs::PointCloud2ConstPtr& ros_cloud) 
    {
      // Only ros cloud can be transformed!
      tf::TransformListener listener;
      tf::StampedTransform  stransform;
      try
      {
        listener.waitForTransform(world_frame,
                                  ros_cloud->header.frame_id,
                                  ros::Time::now(),
                                  ros::Duration(0.3));
        listener.lookupTransform (world_frame,
                                  ros_cloud->header.frame_id,
                                  ros::Time(0),
                                  stransform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
      }

      y = stransform.getOrigin().y();

      sensor_msgs::PointCloud2 transformed_ros_cloud;
      pcl_ros::transformPointCloud("world",
                                  stransform,
                                  *ros_cloud,
                                  transformed_ros_cloud);
      pcl::fromROSMsg(transformed_ros_cloud, pcl_cloud);
      pcl_Y_cloud.header.frame_id = pcl_cloud.header.frame_id; // cannot be done away with; must keep
      // cumulate the line cloud into a profile cloud
      pcl_Y_cloud += pcl_cloud;
      // Pass through the line cloud 3 times
      // 1. work out the averaged differences between points
      average_differences();
      // 2. find the width of the Groove
      // width();
      // 3. work out the cross-section area of the groove
      // area();
      line_no++;
      lastY = y;
    }

    // Getter
    pcl::PointCloud<pcl::PointXYZ> get_pcl_cloud()
    {
      return pcl_cloud;
    }
     
    // Average Differences
    void average_differences()
    {
      int w = 15; // Number of points to take Average
      int m = w/2; // Half of no. of average points
      int j = 1;

      int cloudSize = pcl_cloud.size();

      double sumz = 0.0;
      double sumdavgz = 0.0;
      double sumdavgdavgz = 0.0;

      double Z[cloudSize], avgz[cloudSize], avgdavgz[cloudSize], avgddavgz[cloudSize];
      double davgz[cloudSize], davgdavgz[cloudSize];

      double zz;

      bool first = true;
      bool second = false;

      if (write_X_file)
      {
        OpenLineFile();
      }

      // for (int i = 0; i < (cloudSize - 50); ++i)
      for (int i = 0; i < cloudSize; ++i)
      {
        zz = pcl_cloud[i].z;
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
              /*
              if (avgddavgz[((j-m)-m)-m] > maxDoubleDotZ)
              {
                maxDoubleDotZ = avgddavgz[((j-m)-m)-m];
                maxDoubleDotj = ((j-m)-m)-m;
              }
              */
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
        if ((i >= (cloudSize - 50)) && (valid_end == 0) && 
            ((zz == KEYENCE_INFINITE_DISTANCE_VALUE_SI) || (zz == KEYENCE_INFINITE_DISTANCE_VALUE_SI2)
              || (zz == std::numeric_limits<double>::infinity())))
        {
          valid_end = i;
        }
      } // end of First Loop through the Scan Line
      if (write_X_file)
      {
        Xfile.close();
        file_no++;
      }
      // ROS_INFO_STREAM("Valid at: " << valid_begin << "; 1st y: " << first_y * 1e3);
      // ROS_INFO_STREAM("Max DD position: " << maxDoubleDotj << " Line no.: " << line_no);
    }

    void width()
    {
      bool first = true;
      bool second = false;
      bool third = false;

      double zz;
      double width;

      for (int i = valid_begin; i < valid_end; ++i)
      { // Second Loop through the Scan Line
        zz = pcl_cloud[i].z;

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

    void area()
    {
      geometry_msgs::Point a, b, Top, Bot, Mid;
      int begin = minDoubleDot1i + valid_begin;
      int end = minDoubleDot2i + valid_begin;
      double last_zB; // if inf then use last zB
      double base; // in metre
      double lastX, xIncrement;
      double TopZ = 0.0;
//      double BotZ = 0.0;
      double thickness = 0.0;
      double height = 0.0;
      double lastHeight = height;
      double area = 0.0;

      a.x = pcl_cloud[begin].x;
      a.y = pcl_cloud[begin].y;
      a.z = pcl_cloud[begin].z;

      b.x = pcl_cloud[end].x;
      b.y = pcl_cloud[end].y;
      b.z = pcl_cloud[end].z;

      base = length(a, b);
/*
      Bot.x = pcl_cloud[maxDoubleDotj + valid_begin].x;
      Bot.y = pcl_cloud[maxDoubleDotj + valid_begin].y;
      Bot.z = pcl_cloud[maxDoubleDotj + valid_begin].z;

      BotZ = Bot.z; */
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
        Bot.x = pcl_cloud[i].x;
        Bot.y = pcl_cloud[i].y;
        Bot.z = pcl_cloud[i].z;

        xIncrement = abs(Bot.x - lastX);
        lastX = Bot.x;

        if ((Bot.z == KEYENCE_INFINITE_DISTANCE_VALUE_SI) || (Bot.z == KEYENCE_INFINITE_DISTANCE_VALUE_SI2)
              || (Bot.z == std::numeric_limits<double>::infinity()))
        { // if zB is inf then use last zB
//          ROS_INFO("Bot.z is NaN!");
          Bot.z = last_zB;
        }
        else // keep zB as last zB
        {
//          last_zB = Bot.z;

          Top = proj(a, b, Bot); // The top point is the Bottom project onto the top line ab.
          height = length(Top, Bot);

          if (isnan(height))
          {
//            ROS_INFO("height is NaN!");
            height = lastHeight;
            exit;
          }

          area += height * xIncrement; // both height and x_increment are in Metre, therefore area in Metre square

          if (((line_no != 0) && (height != 0.0) && (thickness != 0.0)) &&
              ((Bot.z != KEYENCE_INFINITE_DISTANCE_VALUE_SI) && (Bot.z != KEYENCE_INFINITE_DISTANCE_VALUE_SI2)
              && (Bot.z != std::numeric_limits<double>::infinity())))
          {
            /* Do not publish the Fillers */
          }
        }
        lastHeight = height;
        last_zB = Bot.z;
      } // End of Third Loop through the Scan Line
      area = area * cos(0.471233898038469);
      volume += area * thickness;
      // ROS_INFO_STREAM("Width: " << base * 1e3 << " mm");
      // ROS_INFO_STREAM("Slice Thickness: " << thickness * 1e3 << " mm");
      // ROS_INFO_STREAM("Scanned Length: " << scanned_length * 1e3 << " mm");
      // ROS_INFO_STREAM("Y: " << y * 1e3);
      // ROS_INFO_STREAM("TopZ: " << TopZ*1e3 << " BotZ: " << BotZ*1e3 << " Plate: " << (TopZ - BotZ) * 1e3);
      // ROS_INFO_STREAM("Area: " << area * 1e6 << " mm2");
      // ROS_INFO_STREAM("Volume: " << volume * 1e9 << " mm3\n");
      if (write_Y_file)
      {
        Yfile << base * 1e3 << ", " << thickness * 1e3 << ", " << area * 1e6 << ", " << first_y * 1e3 << ", " 
              << y * 1e3 << ", " << scanned_length * 1e3 << ", " << volume * 1e9 << "\n";
      }
    }
};

// This handles one scan line.
void callback(const sensor_msgs::PointCloud2ConstPtr& ros_cloud)
{
  // Create an object of Line_cloud (this will call the constructor)
  Line_cloud line_cloud(ros_cloud);

  // The publisher is initialised in main
  pub.publish(pcl_Y_cloud);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "concave_scanner");
  ros::NodeHandle nh, pnh("~");

  // set up profile cloud publisher for PCL point clouds
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("Y_profiles", 1);

  // Listen for Point Cloud - profile from Laser Scanner

  std::string cloud_topic; // Name of the topic to be subscribed
  
  cloud_topic = "profiles"; // The cloud published by the Keyence Driver

  std::string topic = nh.resolveName(cloud_topic);
  ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic: " << topic);

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
    OpenSurfaceFile();
  }
  // Subscribe to the "profile" topic 
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(topic, 1, callback);

  ros::spin(); // This will keep the whole thing in an infinite loop until ctl-C
  return 0;
}
