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

  double x, y, lastY, z, thickness;
  double volume = 0.0;

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
  visualization_msgs::Marker fillers;

  fillers.header.frame_id = world_frame;
  fillers.ns = "fillers";
  fillers.action = visualization_msgs::Marker::ADD;
  fillers.pose.orientation.w = 1.0; // Quarternion
  fillers.id = 0;
  fillers.type = visualization_msgs::Marker::CUBE;
  fillers.scale.x = 0.0001; // so the line is shown as of 0.1mm wide
  fillers.color.r = 1;   // in red
  fillers.color.a = 1;   // 

  geometry_msgs::Point points;

  // set up fillers Marker publisher
  ros::Publisher fillers_pub = nh.advertise<visualization_msgs::Marker>("fillers", 400);

  pc_msg->header.frame_id = frame_id;
  pc_msg->is_dense = false; // cloud could have NaNs
  // message is essentially a line-strip of points
  pc_msg->height = 1;

  transformed_pc_msg->header.frame_id = world_frame;
  transformed_pc_msg->is_dense = false;

  // set up profile cloud publisher
  ros::Publisher pub = nh.advertise<Cloud>("profiles", 100);

  bool active_flag = true;
  int line_no = 0;
  int file_no = 1;
  
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
      char out_text[100];
      Eigen::Isometry3d out_text_pose = Eigen::Isometry3d::Identity();

      // int line_no = 0;

      // Main loop
      sleeper.reset();
      while (ros::ok())
      {
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
        {
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
                                      ros::Duration(1.5)); // *********************************************//

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

          // y = transformed_cloud.points[0].y;

          pcl::fromROSMsg (transformed_cloud, *transformed_pc_msg_local);

/***********************************************************************************************
          cross_section_area = cross_section(*transformed_pc_msg_local, points);
************************************************************************************************/
  Cloud pointcloud = *transformed_pc_msg_local;
  double area = 0.0;
  double zz, lastZ;
  double dotZ, lastDotZ;
  double doubleDotZ; // lastDoubleDiffZ;
  double minDoubleDot1Z;
  double minDoubleDot2Z;
  double maxDoubleDotZ;
  double height, Height;
  int minDoubleDot1i, minDoubleDot2i, maxDoubleDoti;
  int cloudSize = pointcloud.size();
  // bool writeToFile = false;
  bool first = true;
  bool second = false;
  bool third = false;


  // Line list pose
  // geometry_msgs::Point p;
  
  ROS_INFO_STREAM("The x step size: " << global_x_increment << "mm\n");

  using namespace std;

  // Find the maximum double d first
  for (int i = 0; i < (cloudSize - 50); ++i)
  {
    zz = pointcloud[i].z;

    if ((zz != KEYENCE_INFINITE_DISTANCE_VALUE_SI) && (zz != KEYENCE_INFINITE_DISTANCE_VALUE_SI2)
          && (zz != std::numeric_limits<double>::infinity()))
    {   // then this is a piece of normal data
      if (first) // from 1st onward
      {
        first = false;
        lastZ = zz;
        second = true;
      } 
      else if (second)
      {
        second = false;
        dotZ = zz - lastZ;
        lastDotZ = dotZ;
        lastZ = zz;
        third = true;
      } 
      else if (third)
      {
        third = false;
        dotZ = zz - lastZ;
        doubleDotZ = dotZ - lastDotZ;
        lastZ = zz;
        lastDotZ = dotZ;
        maxDoubleDotZ = -1; maxDoubleDoti = i;
      }
      else
      {
        dotZ = zz - lastZ;
        doubleDotZ = dotZ - lastDotZ;
        lastZ = zz;
        lastDotZ = dotZ;

        if (i <= 30) // ignore the beginning
        {
          maxDoubleDotZ = doubleDotZ; maxDoubleDoti = i;
        }
        else
        {
          if (doubleDotZ > maxDoubleDotZ)
          {
            maxDoubleDotZ = doubleDotZ;
            maxDoubleDoti = i;
          }
        }
        
      }

    }
  }

  ROS_INFO_STREAM("Finished finding the Max Double d: " << maxDoubleDotZ * 1e3 << " at position: " << maxDoubleDoti);

  // Next find the left and right Min double d 
  for (int i = 0; i < (cloudSize - 50); ++i)
  {
    zz = pointcloud[i].z;

    if ((zz != KEYENCE_INFINITE_DISTANCE_VALUE_SI) && (zz != KEYENCE_INFINITE_DISTANCE_VALUE_SI2)
          && (zz != std::numeric_limits<double>::infinity()))
    {   // then this is a piece of normal data
      if (first) // from 1st onward
      {
        first = false;
        lastZ = zz;
        second = true;
      } 
      else if (second)
      {
        second = false;
        dotZ = zz - lastZ;
        lastDotZ = dotZ;
        lastZ = zz;
        third = true;
      } 
      else if (third)
      {
        third = false;
        dotZ = zz - lastZ;
        doubleDotZ = dotZ - lastDotZ;
        lastZ = zz;
        lastDotZ = dotZ;
      }
      else
      {
        dotZ = zz - lastZ;
        doubleDotZ = dotZ - lastDotZ;
        lastZ = zz;
        lastDotZ = dotZ;

        if (i <= 30) // ignore the initial 30 points
        {
          minDoubleDot1Z = doubleDotZ; minDoubleDot1i = i;
          minDoubleDot2Z = doubleDotZ; minDoubleDot2i = i;
        }
        else
        {
          if ((i < (maxDoubleDoti - 10)) && (doubleDotZ < minDoubleDot1Z))
          {
            minDoubleDot1Z = doubleDotZ;
            minDoubleDot1i = i;
          }
          if ((i > maxDoubleDoti) && (doubleDotZ < minDoubleDot2Z))
          {
            minDoubleDot2Z = doubleDotZ;
            minDoubleDot2i = i;
          }
        }
      }

    }
  }

  ROS_INFO_STREAM("Finished finding the Left Miin Double d: " << minDoubleDot1Z * 1e3 << " at position: " << minDoubleDot1i);
  ROS_INFO_STREAM("Finished finding the Right Miin Double d: " << minDoubleDot2Z * 1e3 << " at position: " << minDoubleDot2i);
  
  double z1 = pointcloud[minDoubleDot1i].z;
  double z2 = pointcloud[minDoubleDot2i].z;
  double zT; // z at top
  double zB; // z at bottom
  double dh; // difference in height
  int begin, end, base;
  string file_name;

  begin = minDoubleDot1i;
  end = minDoubleDot2i;
  base = end - begin;

  if (base < 280) // a special case for debugging
  {
    ROS_INFO("Base less than 28");
    ofstream myfile;
    file_name = "profile_" + std::to_string(file_no) + ".csv";
    myfile.open(file_name);
    for (int i = 0; i < (cloudSize - 50); ++i)
    {
      myfile << pointcloud[i].z << ", \n";
    }
    myfile.close();
    file_no++;
  }

  // After the three turning points are found, publish the fillers in this loop
  for (int i = begin; i <= end; ++i)
  {
    zB = pointcloud[i].z; // Bottom
    if (z2 > z1)
    {
      dh = z2 - z1;
    }
    else
    {
      dh = z1 - z2;
    }
    zT = (z1 + (((double)(i - begin)/base) * dh)); // Top

    if (zB > zT) // 
    {
      height = zB - zT;

      /*
      ROS_INFO_STREAM("zB: " << pointcloud[i].z);
      ROS_INFO_STREAM("i: " << i << " begin: " << begin << " (i - begin): " << (i - begin));
      ROS_INFO_STREAM("base: " << base << " (i-begin)/base: " << (double)(i-begin)/base);
      ROS_INFO_STREAM("z2: " << z2 << " z1: " << z1 << " dh: " << dh);
      ROS_INFO_STREAM("[(i-begin)/base]*(z2-z1): " << ((double)(i-begin)/base)*(z2-z1));
      ROS_INFO_STREAM("z1: " << z1 << " zT=z1 + ^: " << z1+((double)(i-begin)/base)*(z2-z1));
      */
    }
    else
    {
      height = zT - zB;
    }
    area += height * global_x_increment;

    fillers.pose.position.x = pointcloud[i].x;
    fillers.pose.position.y = pointcloud[i].y;
    fillers.pose.position.z = zB + (height/2);

    fillers.scale.z = height;
    // fillers.points.push_back(p);
    
    if (fillers.scale.x == 0.0)
    {
      ROS_INFO_STREAM("Fillers.scale.x was 0.0 and Line: " << line_no);
    } 
    else if (fillers.scale.y == 0.0)
    {
      ROS_INFO_STREAM("Fillers.scale.y was 0.0 and Line: "<< line_no);
    } else if (fillers.scale.z == 0.0)
    {
      ROS_INFO_STREAM("Fillers.scale.z was 0.0 and Line: " << line_no);
      // cout << "Something is wrong!\n";
      // string reply;
      // getline(cin, reply);
    }
    else
    {
      fillers_pub.publish(fillers);
      fillers.id++;
    }


  }

  Height = pointcloud[maxDoubleDoti].z;

  ROS_INFO_STREAM("The Base is: " << base/10 << "mm\n");
  // ROS_INFO_STREAM("The Height is: " << Height * 1e3 << "mm\n");
  ROS_INFO_STREAM("The cross section area: " << area <<"mm2\n");

          x = stransform.getOrigin().x();
          y = stransform.getOrigin().y();
          z = stransform.getOrigin().z();
          
          if (first_line)
          { 
            lastY = y; 
            first_line = false;
          }
          else
          { // Not first line
            ROS_INFO_STREAM("y: " << y * 1e3 << "mm; lastY: " << lastY * 1e3 << "mm\n");

            thickness = (y - lastY);
            ROS_INFO_STREAM("Thickness: " << thickness * 1e3 << "mm\n");

            volume += area * thickness;

            *transformed_pc_msg += *transformed_pc_msg_local;
          }

          lastY = y;

          fillers.scale.y = thickness; // so the line is shown as of 0.1mm wide


          // fillers.points = points;
          // fillers_pub.publish(fillers);

          ROS_INFO_STREAM("Volume: " << volume << "mm3\n");

          int n = sprintf(out_text, 
                      "Cross Section Area: %5.2f mm2\nThickness of slice: %5.2f mm\nVolume of Groove: %5.2f mm3\n",
                      area,
                      thickness,
                      volume
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
          visual_tools.trigger();

          // publish pointcloud
          pub.publish(transformed_pc_msg);
        }
      } // end main loop
    }
    catch (const keyence::KeyenceException& ex)
    {
      ROS_ERROR_STREAM("Keyence Exception Caught:\n" << ex.what());
      ROS_ERROR_STREAM("Attempting reconnection after pause");
      ros::Duration(1.0).sleep();
    }
    line_no++;
  }
  line_no++;
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

  global_x_increment = keyence::unitsToMeters(info.x_increment) * 1e3;

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
