#include <ros/ros.h>
#include "roboweld_core/PlanCartesianPath.h"

#include <ur5_demo_descartes/ur5_robot_model.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_planner/sparse_planner.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/joint_trajectory_pt.h>
#include <descartes_utilities/ros_conversions.h>
#include <eigen_conversions/eigen_msg.h>

// For inserting objects into the planning scene
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <cmath>

ros::Publisher way_pts_axes_pub;

std::vector<double> getCurrentJointState(const std::string& topic)
{
  sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage<sensor_msgs::JointState>(topic, ros::Duration(0.0));
  if (!state) throw std::runtime_error("Joint state message capture failed");
  return state->position;
}

/*
 ************** Publish Way Points Axes *****************************
 */
static void pubWayPtsAxes(const EigenSTL::vector_Isometry3d& poses)
{
  // The axes of the poses
  visualization_msgs::Marker x_axis, y_axis, z_axis;
  visualization_msgs::MarkerArray markers_msg;

  x_axis.header.frame_id = "world";
  x_axis.header.stamp = ros::Time::now();
  x_axis.ns = "x_axis";
  x_axis.id = 0;
  x_axis.type = visualization_msgs::Marker::LINE_LIST;
  x_axis.action = visualization_msgs::Marker::ADD;
  x_axis.pose.orientation.w = 1.0;
  x_axis.color.r = 1;
  x_axis.color.g = 0;
  x_axis.color.b = 0;
  x_axis.color.a = 1;
  x_axis.scale.x = 0.01;
//  x_axis.points.push_back(m.pose.pose.position);

  y_axis.header.frame_id = "world";
  y_axis.header.stamp = ros::Time::now();
  y_axis.ns = "y_axis";
  y_axis.id = 1;
  y_axis.type = visualization_msgs::Marker::LINE_LIST;
  y_axis.action = visualization_msgs::Marker::ADD;
  y_axis.pose.orientation.w = 1.0;
  y_axis.color.r = 0;
  y_axis.color.g = 1;
  y_axis.color.b = 0;
  y_axis.color.a = 1;
  y_axis.scale.x = 0.01;
//  y_axis.points.push_back(m.pose.pose.position);

  z_axis.header.frame_id = "world";
  z_axis.header.stamp = ros::Time::now();
  z_axis.ns = "z_axis";
  z_axis.id = 2;
  z_axis.type = visualization_msgs::Marker::LINE_LIST;
  z_axis.action = visualization_msgs::Marker::ADD;
  z_axis.pose.orientation.w = 1.0;
  z_axis.color.r = 0;
  z_axis.color.g = 0;
  z_axis.color.b = 1;
  z_axis.color.a = 1;
  z_axis.scale.x = 0.01;
//  z_axis.points.push_back(m.pose.pose.position);

  // create axes markers
  z_axis.points.reserve(2*poses.size());
  y_axis.points.reserve(2*poses.size());
  x_axis.points.reserve(2*poses.size());

  geometry_msgs::Point start_point, x_end_point, y_end_point, z_end_point;

  for (unsigned int i = 0; i < poses.size(); i++)
  {
    const Eigen::Isometry3d& pose = poses[i];

    // All 3 axes start from the same point
    tf::pointEigenToMsg(pose.translation(), start_point);

    Eigen::Isometry3d move_along_x = pose * Eigen::Translation3d(0.1, 0, 0);
    tf::pointEigenToMsg(move_along_x.translation(), x_end_point);
    x_axis.points.push_back(start_point);
    x_axis.points.push_back(x_end_point);

    Eigen::Isometry3d move_along_y = pose * Eigen::Translation3d(0, 0.1, 0);
    tf::pointEigenToMsg(move_along_y.translation(), y_end_point);
    y_axis.points.push_back(start_point);
    y_axis.points.push_back(y_end_point);

    Eigen::Isometry3d move_along_z = pose * Eigen::Translation3d(0, 0, 0.1);
    tf::pointEigenToMsg(move_along_z.translation(), z_end_point);
    z_axis.points.push_back(start_point);
    z_axis.points.push_back(z_end_point);
  }

  markers_msg.markers.push_back(x_axis);
  markers_msg.markers.push_back(y_axis);
  markers_msg.markers.push_back(z_axis);

  way_pts_axes_pub.publish(markers_msg);

//  visual_pub.publish(marker);
 
}

/*****************************************************************************/

/*
 *************************** Make Line **********************************
 */
EigenSTL::vector_Isometry3d makeLine(const Eigen::Vector3d& start,
                                     const Eigen::Vector3d& stop,
                                     double ds,
                                     double degree,
                                     int axis)
{
  EigenSTL::vector_Isometry3d line;
  
  const Eigen::Vector3d travel = stop - start;
  const int steps = std::floor(travel.norm() / ds);

  // Linear interpolation
  for (int i = 0; i < steps; ++i)
  {
    double ratio = static_cast<float>(i) / steps;
    Eigen::Vector3d position = start + ratio * travel;
/*
    Eigen::Isometry3d tr;
    tr = Eigen::Translation3d(position);
    line.push_back( tr );
*/

    Eigen::Isometry3d pose;
    pose = Eigen::Translation3d(position);
    switch (axis)
    {
      case 1:
        pose *= Eigen::AngleAxisd((M_PI*(degree/180.0)), Eigen::Vector3d::UnitX());
        break;
      case 2:
        pose *= Eigen::AngleAxisd((M_PI*(degree/180.0)), Eigen::Vector3d::UnitY());
        break;
      case 3:
        pose *= Eigen::AngleAxisd((M_PI*(degree/180.0)), Eigen::Vector3d::UnitZ());
        break;
      case 4:
        // Do not rotate about any axis at all
        break;
    }
    line.push_back(pose);

  }

  return line;
}

/*
 *********************************** Make Arc ***********************************
 */

EigenSTL::vector_Isometry3d makeArc(double start, // in degrees
                                    double stop, // in degrees
                                    double radius,
                                    int steps) 
{
  EigenSTL::vector_Isometry3d arc;
  

  double travel = ((stop - start)/180.0) * M_PI; // in radians
  const double intervals = travel / steps;

  // Circular interpolation
  for (int i = 0; i < steps; ++i)
  {
    double angle = (start/180.0) + (intervals * i) ; // radians

    Eigen::Vector3d position =
         Eigen::Vector3d(radius * cos(angle), radius * sin(angle), 0.0);

    Eigen::Isometry3d pose;
    pose = Eigen::Translation3d(position);

    pose *= Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
    pose *= Eigen::AngleAxisd((M_PI * -0.1), Eigen::Vector3d::UnitY());
    arc.push_back(pose);
  }

  pubWayPtsAxes(arc);

  return arc;
}

/*
 ********************************** Class Cartesian Planner ************************
 */

class CartesianPlanner
{
public:

  CartesianPlanner(ros::NodeHandle& nh)
  {
    // first init descartes
    if (!initDescartes())
      throw std::runtime_error("There was an issue initializing Descartes");
    else
      ROS_INFO("Descartes initialised.");

    // init services
    server_ = nh.advertiseService("plan_path", &CartesianPlanner::planPath, this);
  }

/************************************************************************************/

  bool initDescartes()
  {
    // Create a robot model
    model_ = boost::make_shared<ur5_demo_descartes::UR5RobotModel>();
    
    // Define the relevant "frames"
    const std::string robot_description = "robot_description";
    const std::string group_name = "manipulator";
    const std::string world_frame = "world"; // Frame in which tool poses are expressed
    const std::string tcp_frame = "tool0";

    // Using the desired frames, let's initialize Descartes
    if (!model_->initialize(robot_description, group_name, world_frame, tcp_frame))
    {
      ROS_WARN("Descartes RobotModel failed to initialize");
      return false;
    }
    else
      ROS_INFO("Descartes RobotModel initialised.");

    model_->setCheckCollisions(true);  // Turn on collision checking.

    if (!planner_.initialize(model_))
    {
      ROS_WARN("Descartes Planner failed to initialize");
      return false;
    }
    else
    {
      ROS_INFO("Descartes Planner initialised.");
      return true;
    }
  }

/******************************************************************************************/

  bool planPath(roboweld_core::PlanCartesianPathRequest& req,
                roboweld_core::PlanCartesianPathResponse& res)
  {
    ROS_INFO("Recieved cartesian planning request");

    // Step 1: Generate path poses
    EigenSTL::vector_Isometry3d tool_poses = makeToolPoses();
    
    // Step 2: Translate that path by the input reference pose and convert to "Descartes points"
    std::vector<descartes_core::TrajectoryPtPtr> 
        path = makeDescartesTrajectory(req.pose, tool_poses);

    // Step 3: Tell Descartes to start at the "current" robot position
    std::vector<double> start_joints = getCurrentJointState("joint_states");
    descartes_core::TrajectoryPtPtr pt (new
                 descartes_trajectory::JointTrajectoryPt(start_joints));
    path.front() = pt;

    // Step 4: Plan with descartes
    ROS_INFO("Going to call planPath.");
    if (!planner_.planPath(path))
    {
      ROS_ERROR("Could not solve for a valid path");
      return false;
    }
    else
      ROS_INFO("A valid path solved.");

    std::vector<descartes_core::TrajectoryPtPtr> result;
    if (!planner_.getPath(result))
    {
      ROS_ERROR("Could not retrieve path");
      return false;
    }
    else
      ROS_INFO("Planned Path retrieved.");

    // Step 5: Convert the output trajectory into a ROS-formatted message
    res.trajectory.header.stamp = ros::Time::now();
    res.trajectory.header.frame_id = "world";
    res.trajectory.joint_names = getJointNames();
    const static double default_joint_vel = 0.03; // rad/s
/*    const static double default_joint_vel = 1.0; // rad/s original was 1.0 */
    if (!descartes_utilities::toRosJointPoints(*model_,
                                          result,
                                          default_joint_vel,
                                          res.trajectory.points
                                          ))
    {
      ROS_ERROR("Unable to convert Descartes trajectory to joint points");
      return false;
    }
    else
    {
      ROS_INFO("Descartes trajectory converted to Joint Points.");
    }
    return true;
  }

/**********************************************************************************/

  EigenSTL::vector_Isometry3d makeToolPoses()
  {
    EigenSTL::vector_Isometry3d path;

    // We assume that our path is centered at (0, 0, 0), so let's define the
    // corners of the AR marker

    const double side_length = 0.15; // All units are in meters (M)
    const double half_side = side_length / 2.0;
    const double step_size = 0.025;

    Eigen::Vector3d top_left (-half_side, half_side, 0);
    Eigen::Vector3d bot_left (-half_side, -half_side, 0);
    Eigen::Vector3d bot_right (half_side, -half_side, 0);
    Eigen::Vector3d top_right (half_side, half_side, 0);

    // Descartes requires you to guide it in how dense the points should be,
    // so you have to do your own "discretization".
    // NOTE that the makeLine function will create a sequence of points inclusive
    // of the start and exclusive of finish point, i.e. line = [start, stop)
    
    // TODO: Add the rest of the cartesian path
//    auto segment1 = makeLine(top_left, bot_left, step_size, 90, 4);
//    auto segment2 = makeLine(bot_left, bot_right, step_size, -90, 4);
    auto segment3 = makeLine(bot_right, top_right, step_size, -90, 4);
//    auto segment4 = makeLine(top_right, top_left, step_size,30, 2);

//    path.insert(path.end(), segment1.begin(), segment1.end());
//    path.insert(path.end(), segment2.begin(), segment2.end());
    path.insert(path.end(), segment3.begin(), segment3.end());
//    path.insert(path.end(), segment4.begin(), segment4.end());


//    return(makeArc(0.0, 90, 0.07, 20));

    return path;
  }

/********************************************************************************/

  std::vector<descartes_core::TrajectoryPtPtr>
  makeDescartesTrajectory(const geometry_msgs::Pose& reference,
                           const EigenSTL::vector_Isometry3d& path)
  {
    std::vector<descartes_core::TrajectoryPtPtr> descartes_path; // return value

    Eigen::Isometry3d ref;
    tf::poseMsgToEigen(reference, ref);

/**/

geometry_msgs::Pose pose;
int i=0;

/**/

    for (auto& point : path)
    {

      // Turn 45 degree about the Y axis
//      my_pt = point * Eigen::AngleAxisd((M_PI/4.0), Eigen::Vector3d::UnitY());

      // TODO: make a Descartes "cartesian" point with some kind of constraints
      descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(ref * point);
      descartes_path.push_back(pt);
/**/

tf::poseEigenToMsg((ref * point), pose);
ROS_INFO_STREAM("pose " << i++ << ": " << pose);

/**/
    }
    return descartes_path;
  }

/******************************************************************************/

  descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(
                                     const Eigen::Isometry3d& pose)
  {
    using namespace descartes_core;
    using namespace descartes_trajectory;

    return TrajectoryPtPtr( new AxialSymmetricPt(pose,
                                                 M_PI/180.0,
                                                 AxialSymmetricPt::Z_AXIS
                                                )
                          );
  }

/*********************************************************************************/

  // HELPER
  std::vector<std::string> getJointNames()
  {
    std::vector<std::string> names;
    nh_.getParam("controller_joint_names", names);
    return names;
  }

/************************** Variable Declarations **********************************/

  boost::shared_ptr<ur5_demo_descartes::UR5RobotModel> model_;
  descartes_planner::DensePlanner planner_;
  ros::ServiceServer server_;
  ros::NodeHandle nh_;

/*
  Visualization

  namespace rvt = rviz_visual_tools;
  visual_tools.deleteAllMarkers();
*/
};

/***********************************************************************************/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "descartes_node");

  ros::NodeHandle nh;

  way_pts_axes_pub = nh.advertise<visualization_msgs::MarkerArray>("way_pts_axes",1);

  CartesianPlanner planner (nh);

  ROS_INFO("Cartesian planning node starting");
  ros::spin();
}
