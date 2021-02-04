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

#include <keyence/impl/keyence_exception.h>
#include <keyence/impl/keyence_tcp_client.h>
#include <keyence/impl/messages/high_speed_single_profile.h>
#include <keyence/impl/messages/change_program.h>
#include <keyence/impl/messages/get_setting.h>
#include <keyence/impl/settings_defs.h>

#include "boost/bind.hpp"
#include "boost/ref.hpp"

#include "keyence_experimental/ChangeProgram.h"

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

// keyence protocol / profile related defines
static const std::string KEYENCE_DEFAULT_TCP_PORT = "24691";
static const std::string KEYENCE_DEFAULT_TCP_PORT_HS = "24692";

// values LJ Navigator uses for out-of-range points (in meters)
const static double KEYENCE_INFINITE_DISTANCE_VALUE_SI = -999.9990 / 1e3;
const static double KEYENCE_INFINITE_DISTANCE_VALUE_SI2 = -999.9970 / 1e3;

// default values for parameters
const static std::string DEFAULT_FRAME_ID = "sensor_optical_frame";
const static std::string DEFAULT_WORLD_FRAME = "world";

// local types
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

// Global variable
double global_x_increment;

// Prototype for function that converts a given profile to
// a PCL point cloud
int unpackProfileToPointCloud(const keyence::ProfileInformation& info,
                              const std::vector<int32_t>& points, Cloud& msg, bool cnv_inf_pts);

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
 * @brief Given a @e client, makes a request to figure out if the given @e program
 * is continuously triggered or not.
 */
bool isProgramContinuouslyTriggered(keyence::TcpClient& client, uint8_t program)
{
  uint8_t level = keyence::setting::write_area;
  uint8_t type = keyence::setting::program::programType(program);
  uint8_t category = keyence::setting::program::TriggerMode::category;
  uint8_t item = keyence::setting::program::TriggerMode::item;

  keyence::command::GetSetting::Request req (level, type, category, item, 0, 0, 0, 0);
  auto resp = client.sendReceive(req);

  if (!resp.good())
  {
    throw keyence::KeyenceException("Controller responded but errored on request");
  }

  uint8_t result_id = resp.body.data[0];

  if (result_id == keyence::setting::program::TriggerMode::continuous_trigger)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @brief Get's the active program number by making a settings-read request and pulling
 * the number out of the response header.
 */
uint8_t getActiveProgramNumber(keyence::TcpClient& client)
{
  uint8_t level = keyence::setting::write_area;
  uint8_t type = keyence::setting::program::programType(0);
  uint8_t category = keyence::setting::program::TriggerMode::category;
  uint8_t item = keyence::setting::program::TriggerMode::item;

  keyence::command::GetSetting::Request req (level, type, category, item, 0, 0, 0, 0);
  auto resp = client.sendReceive(req);

  return resp.header.active_program_no;
}

/**
 * @brief Get's the commanded sampling frequency of the auto-trigger setting
 * for the given program. Ranges from 10 Hz - 64 kHz. This driver can currently
 * do only about 200 Hz. The sensor can only do about 1 kHz over TCP/IP with
 * its high speed mode.
 */
double getProgramSamplingRate(keyence::TcpClient& client, uint8_t program)
{
  uint8_t level = keyence::setting::write_area;
  uint8_t type = keyence::setting::program::programType(program);
  uint8_t category = keyence::setting::program::SamplingPeriod::category;
  uint8_t item = keyence::setting::program::SamplingPeriod::item;

  keyence::command::GetSetting::Request req (level, type, category, item, 0, 0, 0, 0);
  auto resp = client.sendReceive(req);

  if (!resp.good())
  {
    throw keyence::KeyenceException("Controller responded but errored on request");
  }

  uint8_t result_id = resp.body.data[0];

  switch (result_id)
  {
  case keyence::setting::program::SamplingPeriod::freq_10hz:
    return 10.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_20hz:
    return 20.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_50hz:
    return 50.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_100hz:
    return 100.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_200hz:
    return 200.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_500hz:
    return 500.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_1000hz:
    return 1000.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_2000hz:
    return 2000.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_4000hz:
    return 4000.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_4130hz:
    return 4130.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_8000hz:
    return 8000.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_16000hz:
    return 16000.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_32000hz:
    return 32000.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_64000hz:
    return 64000.0;
    break;
  default:
    throw keyence::KeyenceException("Received unrecognized frequency code: " + std::to_string(result_id));
  }
}

/**
 * @brief Services external ROS requests to change the active program. Will reset
 * activity flag and sampling rate according to the settings of the new program.
 */
bool changeProgramCallback(keyence_experimental::ChangeProgram::Request& req,
                           keyence_experimental::ChangeProgram::Response& res,
                           keyence::TcpClient& client, bool& active_flag,
                           ros::Rate& rate)
{
  if (req.program_no > keyence::setting::max_program_index)
  {
    ROS_ERROR("Rejecting program change request: Commanded program index %hhu is out of"
              " the valid range %hhu to %hhu", req.program_no, keyence::setting::min_program_index,
              keyence::setting::max_program_index);
    res.code = res.ERROR_OUT_OF_RANGE;
    return true;
  }

  ROS_INFO("Attempting to change keyence program to %hhd", req.program_no);

  try
  {
    keyence::command::ChangeProgram::Request cmd(req.program_no);
    keyence::Client::Response<keyence::command::ChangeProgram::Request> resp =
        client.sendReceive(cmd);

    if (resp.good())
    {
      active_flag = isProgramContinuouslyTriggered(client, req.program_no);
      double sample_rate = getProgramSamplingRate(client, req.program_no);
      rate = ros::Rate(sample_rate);

      ROS_INFO("Program successfully changed to %hhu with sample freq of %f and"
               " continuous sampling mode = %hhu", req.program_no, sample_rate,
               static_cast<uint8_t>(active_flag));

      if (sample_rate > 200.0)
      {
        ROS_WARN("Note that when sampling above 200Hz this driver will likely not keep up");
      }

      res.code = res.SUCCESS;
      return true;
    }
    else
    {
      res.code = res.ERROR_OTHER;
    }

  } catch (const keyence::KeyenceException& e) // TODO: The exception safety here is suspect
  {                                            // We could fail on the setting reads after a
                                               // successful program change.
    ROS_ERROR("Keyence threw exception while processing program change request: %s",
              e.what());
    res.code = res.ERROR_OTHER;
    return true;
  }

  return false;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyence_lj_driver");
  ros::NodeHandle nh, pnh("~");

  // ros parameters
  std::string sensor_host;
  std::string frame_id;
  std::string world_frame;
  std::string sensor_port;
  double sample_rate;

  double x, y, z;
  double first_y = 0.0;
  double lastY = 0.0;
  double TopZ = 0.0;
  double BotZ = 0.0;
  double thickness = 0.0;
  double volume = 0.0;
  double scanned_length = 0.0;

  // check required parameters
  if (!pnh.hasParam("controller_ip"))
  {
    ROS_FATAL("Parameter 'controller_ip' missing. Cannot continue.");
    return -1;
  }

  pnh.getParam("controller_ip", sensor_host);
  pnh.param<std::string>("controller_port", sensor_port, KEYENCE_DEFAULT_TCP_PORT);
  pnh.param<std::string>("frame_id", frame_id, DEFAULT_FRAME_ID);
  pnh.param<std::string>("world_frame", world_frame, DEFAULT_WORLD_FRAME);

  ROS_INFO("Attempting to connect to %s (TCP %s); expecting a single head attached to port A.",
           sensor_host.c_str(), sensor_port.c_str());

  // setup point cloud message (we reuse single one)
  // TODO: this won't work with nodelets
  Cloud::Ptr pc_msg(new Cloud);

  // Another point cloud transformed into the world frame
  Cloud::Ptr transformed_pc_msg(new Cloud);

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

  pc_msg->header.frame_id = frame_id;
  pc_msg->is_dense = false; // cloud could have NaNs
  // message is essentially a line-strip of points
  pc_msg->height = 1;

  transformed_pc_msg->header.frame_id = world_frame;
  transformed_pc_msg->is_dense = false;

  // set up profile cloud publisher
  ros::Publisher pub = nh.advertise<Cloud>("profiles", 100);

  bool write_Y_file = true; // write the whole scan brief data to a file
  bool active_flag = true;
  int line_no = 0;
  int file_no = 0;
  int filler_id = 0;
  int marker_id = 1;
  
  string file_name;
  ofstream Yfile;
  time_t now = time(0);
  tm *ltm = localtime(&now);
  char name_text[1200];

  while (ros::ok())
  {
    try
    {
      // Establish communications
      keyence::TcpClient keyence(sensor_host, sensor_port);

      auto active_program = getActiveProgramNumber(keyence);
      active_flag = isProgramContinuouslyTriggered(keyence, active_program);
      sample_rate = getProgramSamplingRate(keyence, active_program);

      ROS_INFO("Connection established to Keyence controller.");
      ROS_INFO("Beginning with program %hhu with a sampling rate of %f",
               active_program, sample_rate);
      if (!active_flag)
      {
        ROS_INFO("Note that program %hhu is not configured for continuous sampling",
                 active_program);
      }

      ros::Rate sleeper (sample_rate);

      ros::ServiceServer program_server =
          nh.advertiseService<keyence_experimental::ChangeProgram::Request,
                              keyence_experimental::ChangeProgram::Response>(
              "change_program", boost::bind(changeProgramCallback, _1, _2, boost::ref(keyence),
                                            boost::ref(active_flag), boost::ref(sleeper)));

      ROS_INFO("Keyence connection established");
      ROS_INFO("Attempting to publish at %.2f Hz.", sample_rate);

      sensor_msgs::PointCloud2 transformed_cloud;
      Cloud::Ptr transformed_pc_msg_local (new Cloud);

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
        int n = sprintf( name_text, "/home/victor/roboweld/Data/Volume/2021/February/%d-%d-%d-volume.csv",
                          // 1900 + ltm->tm_year,
                          // 1 + ltm->tm_mon,
                          ltm->tm_mday,
                          ltm->tm_hour,
                          ltm->tm_min
                       );
        /*
        int n = sprintf(name_text, "/home/victor/roboweld/Data/%d/%d/%d/%d-%d/volume.csv",
                                   1900 + ltm->tm_year,
                                   1 + ltm->tm_mon,
                                   ltm->tm_mday,
                                   5 + ltm->tm_hour,
                                   30 + ltm->tm_min
                                   ); */
        // file_name = ;
        // file_name += 
        // file_name += "";
        // Yfile.open(file_name);


        std::ios_base::iostate exceptionMask = Yfile.exceptions() | std::ios::failbit;
        Yfile.exceptions(exceptionMask);
        try
        {
          Yfile.open(name_text);
          cout << name_text << "\n";
        }
        catch (std::ios_base::failure& e)
        {
          std::cerr << e.what() << '\n';
          cout << "fail to open file: " << name_text;
        }
        /*
        if (Yfile.is_open())
        {
          cout << name_text << "\n";
        }
        else
        {
          cout << "fail to open file: " << name_text;
        }
        */
        Yfile << "Width, Slice , Area, Plate , 1st Y, Y, Scanned Length, Volume\n";
      }

      // Main loop
      sleeper.reset();
      while (ros::ok())
      { // Main Loop
        sleeper.sleep();
        ros::spinOnce();

        // avoid interacting with sensor if there are no publishers
        // TODO: maybe we should actually always poll sensor, but just not
        //       publish anything (although unpacking + publishing is cheap)
        if (pub.getNumSubscribers() == 0)
        {
          ROS_INFO_THROTTLE(60, "No (more) subscribers. Not polling sensor.");
          continue;
        }

        if (!active_flag)
        {
          ROS_INFO_THROTTLE(60, "Sensor is disabled via the 'activity flag'.");
          continue;
        }

        // unpack profile data
        keyence::command::SingleProfile::Request req;
        auto resp = keyence.sendReceive(req);

        // Check for success
        if (!resp.good())
        {
          ROS_DEBUG_STREAM("Error code in header:\n" << resp.header);
        }
        else
        { // Good Point Cloud
          // convert to pointcloud
          pc_msg->points.clear();
          unpackProfileToPointCloud(resp.body.profile_info, resp.body.profile_points, *pc_msg, true);

          // Transform the pointcloud from Keyence Optical Frame to World Frame.
          // Make sure the pc_msg.header.frame_id is changed to world as well.
          tf::TransformListener listener;
          tf::StampedTransform  stransform;
          tf2_ros::Buffer tf_buffer;

          sensor_msgs::PointCloud2 cloud_in;

          try
          {
            listener.waitForTransform(world_frame,
                                      frame_id,
                                      ros::Time::now(),
                                      ros::Duration(0.6)); // Frequency of scan ** It seems to be working fine when 1.0 //

            listener.lookupTransform (world_frame,
                                      frame_id,
                                      ros::Time(0),
                                      stransform);

          }
          catch(tf::TransformException ex)
          {
              ROS_ERROR("%s", ex.what());
          }

          pcl::toROSMsg(*pc_msg, cloud_in);

          pcl_ros::transformPointCloud(world_frame,
                                       stransform,
                                       cloud_in,
                                       transformed_cloud);

          pcl::fromROSMsg (transformed_cloud, *transformed_pc_msg_local);

          x = stransform.getOrigin().x();
          y = stransform.getOrigin().y();
          z = stransform.getOrigin().z();

/***********************************************************************************************
          cross_section_area = cross_section(*transformed_pc_msg_local, points);
************************************************************************************************/
          Cloud pointcloud = *transformed_pc_msg_local;
          double area = 0.0;
          double zz, lastZ;
          double dotZ, lastDotZ;
          double doubleDotZ; // lastDoubleDiffZ;
          double height, Height;
          int valid_begin = 0;
          int minDoubleDot1j, minDoubleDot2j; // maxDoubleDoti;
          int cloudSize = pointcloud.size();
          bool first = true;
          bool second = false;
          bool third = false;
          bool write_X_file = true; // write data of each scan line to a file
          ofstream Xfile;

          // Line list pose
          // geometry_msgs::Point p;
  
          using namespace std;
          int w = 15;     // Number of points to take Average 
          int m = w / 2;  // Half of no. of average points
          int j = 1;
          int maxDoubleDot1j;
          int maxDoubleDot2j;
          double sumz = 0.0;
          double sumdavgz = 0.0;
          double sumdavgdavgz = 0.0;
          double Z[cloudSize], avgz[cloudSize], avgdavgz[cloudSize], avgddavgz[cloudSize];
          double davgz[cloudSize], davgdavgz[cloudSize];

          if (write_X_file)
          {
            int n = sprintf(name_text, "/home/victor/roboweld/Data/Profile/2021/February/%d-%d-%d-profile_%d.csv",
                             // 1900 + ltm->tm_year,
                             // 1 + ltm->tm_mon,
                             ltm->tm_mday,
                             ltm->tm_hour,
                             ltm->tm_min,
                             file_no
                           );
            // file_name = "profile_" + std::to_string(file_no) + ".csv";
            Xfile.open(name_text);
            cout << name_text << "\n";
            Xfile << "J, Z, Average Z, D Avg Z, Avg D Avg Z, D Avg D Avg Z, Avg DD Avg Z\n";
          }

/*********************************************************************************************
 * Going through the scanned line data for the FIRST time.                                   *
 * Should be able to work out the Gradient and the Derivative of the Gradient.               *
 * This node is to scan a groove already has gone through a first pass of welding. The shape *
 * of the groove is changed and we cannot find a Maximum of the Derivative of the Gradient.  *
 * We can only assuming the groove is in the middle of the scan. Then we can divide the scan *
 * into left and right parts along the middle. Then find the Maximum and Minimum on both     *
 * sides.                                                                                    *
 *********************************************************************************************/
          double minDoubleDot1Z = 0.0;
          double minDoubleDot2Z = 0.0;
          double maxDoubleDot1Z = 0.0;
          double maxDoubleDot2Z = 0.0;

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
                  if ((((j-m)-m)-m) < (cloudSize/2)) // on the left hand side
                  {
                    if (avgddavgz[((j-m)-m)-m] > maxDoubleDot1Z)
                    {
                      maxDoubleDot1Z = avgddavgz[((j-m)-m)-m];  
                      maxDoubleDot1j = ((j-m)-m)-m;
                    }
                    if (avgddavgz[((j-m)-m)-m] < minDoubleDot1Z)
                    {
                      minDoubleDot1Z = avgddavgz[((j-m)-m)-m];  
                      minDoubleDot1j = ((j-m)-m)-m;
                    }
                  }
                  if ((((j-m)-m)-m) >= (cloudSize/2)) // on the left hand side
                  {
                    if (avgddavgz[((j-m)-m)-m] > maxDoubleDot2Z)
                    {
                      maxDoubleDot2Z = avgddavgz[((j-m)-m)-m];  
                      maxDoubleDot2j = ((j-m)-m)-m;
                    }
                    if (avgddavgz[((j-m)-m)-m] < minDoubleDot2Z)
                    {
                      minDoubleDot2Z = avgddavgz[((j-m)-m)-m];  
                      minDoubleDot2j = ((j-m)-m)-m;
                    }
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

  ROS_INFO_STREAM("X step: " << global_x_increment * 1e3 << " mm");
  ROS_INFO_STREAM("Valid at: " << valid_begin << "; 1st y: " << first_y * 1e3);
  ROS_INFO_STREAM("Left Max DD position: " << maxDoubleDot1j << " Line no.: " << line_no);
  ROS_INFO_STREAM("Right Max DD position: " << maxDoubleDot2j);

/******************************************************************************************** 
 * Go through the data points the SECOND time. Next find the LEFT and RIGHT Min double d    *
 ********************************************************************************************
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
          } // End of Second Loop through the Scan Line */

  ROS_INFO_STREAM("Left Min DD position: " << minDoubleDot1j);
  ROS_INFO_STREAM("Right Min DD position: " << minDoubleDot2j);
  
/********************************************************************************************
 * After the FOUR turning points are found, go through the data points SECOND time!         *
 * (1) Work out the cross-section area                                                      *
 * (2) Fill in the Marker details in preparation to publish the markers but NOT until the   *
 *     first line is finished then publish the fillers for the whole line outside the loop  *
 ********************************************************************************************/
          geometry_msgs::Point a, b, Top, Bot, Mid;
          int begin = minDoubleDot1j + valid_begin;
          int end = minDoubleDot2j + valid_begin;
          double last_zB; // if inf then use last zB
          double base; // in metre
          // height;

          a.x = pointcloud[begin].x;
          a.y = pointcloud[begin].y;
          a.z = pointcloud[begin].z;

          b.x = pointcloud[end].x;
          b.y = pointcloud[end].y;
          b.z = pointcloud[end].z;


          base = length(a, b);

          Top.x = pointcloud[begin].x;
          Top.y = pointcloud[begin].y;
          Top.z = pointcloud[begin].z;

          y = Top.y; // This is the y on the bottom of the steel plate Groove
          // BotZ = Bot.z;
          // TopZ = proj(a, b, Bot).z;
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

          for (int i = begin; i <= end; ++i)
          { // Third Loop through the Scan Line
            // Work out the cross-section area regardless if this is the first line
            Bot.x = pointcloud[i].x;
            Bot.y = pointcloud[i].y;
            Bot.z = pointcloud[i].z;

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

              area += height * global_x_increment; // both height and x_increment are in Metre, therefore area in Metre square

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

                filler.scale.x = 0.0001; // that is 
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

          *transformed_pc_msg += *transformed_pc_msg_local;

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
          pub.publish(transformed_pc_msg);
          line_no++;
          lastY = y;
        } // finish scanning a good line
        fillers_pub.publish(fillers);
      } // end main loop

      if (write_Y_file)
      {
        Yfile.close();
      }

    }
    catch (const keyence::KeyenceException& ex)
    {
      ROS_ERROR_STREAM("Keyence Exception Caught:\n" << ex.what());
      ROS_ERROR_STREAM("Attempting reconnection after pause");
      ros::Duration(1.0).sleep();
    }
  }
  return 0;
}

int unpackProfileToPointCloud(const keyence::ProfileInformation& info,
                              const std::vector<int32_t>& points, Cloud& msg, bool cnv_inf_pts)
{
  // TODO: get proper timestamp from somewhere
  // pcl header stamps are in microseconds
  msg.header.stamp = ros::Time::now().toNSec() / 1e3;
  msg.width = info.num_profiles;
  cnv_inf_pts = true;

  double x = 0., y = 0., z = 0.;

  msg.points.reserve(info.num_profiles);

  global_x_increment = keyence::unitsToMeters(info.x_increment); // Raw x increment in Metre

  // add points
  for (int i = 0; i < static_cast<int>(points.size()); ++i)
  {
    // convert profile points to SI units (meters)
    x = keyence::unitsToMeters(info.x_start + i * info.x_increment);
    y = 0.0;
    z = keyence::unitsToMeters(static_cast<int>(info.data_unit) * points[i]);

    // filter out 'infinite distance' points
    // REP-117: http://www.ros.org/reps/rep-0117.html
    //  "out of range detections will be represented by +Inf."
    if (points[i] == KEYENCE_INVALID_DATA_VALUE)
    {
      if (cnv_inf_pts)
        z = std::numeric_limits<double>::infinity();
      else
        z = KEYENCE_INFINITE_DISTANCE_VALUE_SI;
    }

    // device returns two different values that are supposed to be interpreted
    // as out-of-range or 'infinite'. This is the second
    if (points[i] == KEYENCE_DEAD_ZONE_DATA_VALUE)
    {
      if (cnv_inf_pts)
        z = std::numeric_limits<double>::infinity();
      else
        z = KEYENCE_INFINITE_DISTANCE_VALUE_SI2;
    }

    msg.points.push_back(pcl::PointXYZ(x, y, z));
  }

  return 0;
}
