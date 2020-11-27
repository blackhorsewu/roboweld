/**
 **  The MAIN of the "motion_node"
 **/

/**
 **  Victor W H Wu
 **
 **  Chinese National Engineering Research Centre for
 **  Steel Construction (Hong Kong Brach)
 **
 **  28 September 2018.
 **
 **  3 July 2019.
 **  Modified by Victor W H Wu
 **
 **  8 July 2019.
 **  Trying to orient the torch such that it is perpendicular to the workpiece when viewing
 **  from the side (i.e. looking in the Y direction).
 **
 **  11 July 2019.
 **  Trying to tidy up the code and trying to find out why it missed the first few points
 **  of the trajectory.
 **/

// Core ros functionality like ros::init and spin
#include <ros/ros.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

// Includes the descartes robot model we will be using
#include <descartes_moveit/ikfast_moveit_state_adapter.h>

// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>

// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>

// Includes the utility function for converting to trajectory_msgs::JointTrajectory's
#include <descartes_utilities/ros_conversions.h>

// Includes moveit for moving "home"
#include <moveit/move_group_interface/move_group_interface.h>

// Includes for ur msgs IO
#include <ur_msgs/SetIO.h>
#include <ur_msgs/IOStates.h>

using namespace std;


/*
 * Forward Declarations for two functions
 */


/**
 * Makes a Simple trajectory, just a straight line on the y axis, for the robot to follow.
 */
std::vector<descartes_core::TrajectoryPtPtr> makePath(ros::NodeHandle& nh);

/**
 * Sends a ROS trajectory to the robot controller
 */
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory,
                       ros::NodeHandle& nh);

/*
 * The Main program body
 */

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "cartesian_node");
  ros::NodeHandle nh;

  // Since we're not calling ros::spin() and doing the planning in a callback, 
  // but rather just handling this inline, we need to create an async spinner if
  // our publishers are to work. Note that many MoveIt components will also not
  // work without an active spinner and Descartes uses moveit for its "groups"
  // and "scene" descriptions
  ros::AsyncSpinner spinner (1);
  spinner.start();

  // 1. First thing first, let's create a kinematic model of the robot. In Descartes,
  // this is used to do things like forward kinematics (joints -> pose), inverse
  // kinematics (pose -> many joints), and collision checking.

  // All of the existing planners (as of Nov 2017) have been designed with the idea
  // that you have "closed form" kinematics. This means that the default solvers in
  // MoveIt (KDL) will NOT WORK WELL. I encourage you to produce an ikfast model 
  // for your robot (see MoveIt tutorial) or use the OPW kinematics package if you
  // have a spherical wrist industrial robot. See http://docs.ros.org/kinetic/api
  // /moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html 

  // This package assumes that the move group you are using is pointing to an IKFast
  // kinematics plugin in its kinematics.yaml file. By default, it assumes that the
  // underlying kinematics are from 'base_link' to 'tool0'. If you have renamed
  // these, please set the 'ikfast_base_frame' and 'ikfast_tool_frame' parameter 
  // (not in the private namespace) to the base and tool frame used to generate the
  // IKFast model.
  descartes_core::RobotModelPtr model
                                (new descartes_moveit::IkFastMoveitStateAdapter());

  // Name of description on parameter server. Typically just "robot_description". 
  // Used to initialize moveit model.
  const std::string robot_description = "robot_description";
  // const std::string robot_description = "weldingtable";

  // name of the kinematic group you defined when running MoveitSetupAssistant. 
  // For many industrial robots this will be "manipulator".
  const std::string group_name = "manipulator";

  // Name of frame in which you are expressing poses. Typically "world_frame" or
  // "base_link".
  const std::string world_frame = "/world";

  // tool center point frame (name of link associated with tool). The robot's 
  // flange is typically "tool0" but yours could be anything. We typically have
  // our tool's positive Z-axis point outward from the grinder, welder, etc.
  const std::string tcp_frame = "tool0";

  // Before you can use a model, you must call initialize. This will load robot
  // models and sanity check the model.
  if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
  {
    ROS_INFO("Could not initialize robot model");
    return -1;
  }
  else
  {
    ROS_INFO("Robot Model Initialized.");
  }

  model->setCheckCollisions(true); // Let's turn on collision checking.

  // 2. The next thing to do is to generate a path for the robot to follow. The
  // description of this path is one of the cool things about Descartes. The
  // source of this path is where this library ties into your application: it could
  // come from CAD or from surfaces that were "scanned".

  // Make the path by calling a helper function. See makePath()'s definition for
  // more discussion about paths.
  std::vector<descartes_core::TrajectoryPtPtr> points = makePath(nh);

  // then Go allZeros, the Initial position the manipulator assumes
/*    moveit::planning_interface::MoveGroupInterface 
                               move_group("manipulator");

    move_group.setNamedTarget("allZeros");
    move_group.move();
*/

  // 3. Now we create a planner that can fuse your kinematic world with the points
  // you want to move the robot along. There are a couple of planners now. 
  // DensePlanner is the naive, brute force approach to solving the trajectory. 
  // SparsePlanner may be faster for some problems (especially very dense ones),
  // but has recieved less overall testing and evaluation.
  descartes_planner::DensePlanner planner;

  // Like the model, you also need to call initialize on the planner
  if (!planner.initialize(model))
  {
    ROS_ERROR("Failed to initialize planner");
    return -2;
  }
  else
  {
    ROS_INFO("Dense Planner Initialized.");
  }

  // 4. Now, for the planning itself. This typically happens in two steps. First,
  // call planPath(). This function takes your input trajectory and expands it into
  // a large kinematic "graph". Failures at this point indicate that the input path
  // may not have solutions at a given point (because of reach/collision) or has
  // two points with no way to connect them.
  if (!planner.planPath(points))
  {
    ROS_ERROR("Could not solve for a valid path");
    return -3;
  }
  else
  {
    ROS_INFO("A valid path solved.");
  }

  // After expanding the graph, we now call 'getPath()' which searches the graph 
  // for a minimum cost path and returns the result. Failures here (assuming
  // planPath was good) indicate that your path has solutions at every waypoint
  // but constraints prevent a solution through the whole path. Usually this
  // means a singularity is hanging out in the middle of your path: the robot can
  // solve all the points but not in the same arm configuration.
  std::vector<descartes_core::TrajectoryPtPtr> result;
  if (!planner.getPath(result))
  {
    ROS_ERROR("Could not retrieve path");
    return -4;
  }
  else
  {
    ROS_INFO("Planned Path retrieved.");
  }

  // 5. Translate the result into something that you can execute. In ROS land, 
  // this means that we turn the result into a trajectory_msgs::JointTrajectory
  // that's executed through a control_msgs::FollowJointTrajectoryAction. If you
  // have your own execution interface, you can get joint values out of the
  // results in the same way.

  // get joint names - this could be from the robot model, or from the parameter
  // server.
  std::vector<std::string> names;
  nh.getParam("controller_joint_names", names);

  // Create a JointTrajectory
  trajectory_msgs::JointTrajectory joint_solution;
  joint_solution.joint_names = names;

  // Define a default velocity. Descartes points without specified timing will
  // use this value to limit the fastest moving joint. This usually effects the
  // first point in your path the most.
  const static double default_joint_vel = 0.01; // rad/s
  if (!descartes_utilities::toRosJointPoints(*model, result, default_joint_vel,
                                             joint_solution.points, 3.0))
  {
    ROS_ERROR("Unable to convert Descartes trajectory to joint points");
    return -5;
  }
  else
  {
    ROS_INFO("Descartes trajectory converted to Joint Points.");
  }

  // 6. Send the ROS trajectory to the robot for execution

  // Now try to execute the Joint Trajectory
  if (!executeTrajectory(joint_solution, nh))
  {
    ROS_ERROR("Could not execute trajectory!");
    return -6;
  }
  else
  {
    ROS_INFO("Trajectory executed successfully.");
  }

  // we should delete the Trajectory after execution. Victor Wu 8 Oct 2018.
  // clear(joint_solution.points);
  joint_solution.points.clear();

  // Wait till user kills the process (Control-C)
  ROS_INFO("Done!");
  return 0;
}

descartes_core::TrajectoryPtPtr makeCartesianPoint
                                (const Eigen::Affine3d& pose, double dt)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new 
                CartTrajectoryPt( TolerancedFrame(pose), TimingConstraint(dt)) );
}

descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint
                                (const Eigen::Affine3d& pose, double dt)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  return TrajectoryPtPtr( new 
                    AxialSymmetricPt(pose, M_PI / 12.0, AxialSymmetricPt::Y_AXIS, 
                    TimingConstraint(dt)) );
}

std::vector<descartes_core::TrajectoryPtPtr> makePath(ros::NodeHandle& nh)
{
  // In Descartes, trajectories are composed of "points". Each point describes what
  // joint positions of the robot can satisfy it. You can have a "joint point" for
  // which only a single solution is acceptable. You might have a fully defined
  // cartesian point for which many (8 or 16) different robot configurations might
  // work. You could allow extra tolerances in any of these and even more points
  // satisfy the constraints.

  const static double time_between_points = 0.15; // keep the time between points

  ROS_INFO("Just start Making Path.");

  // Wait for way points published by perception node

  std::string topic = "wayPoints";

  /*
   * Wait for wayPoints and wait for user to confirm before doing anything
   */

  string reply = "";

  visualization_msgs::Marker::ConstPtr mk_points =
          ros::topic::waitForMessage<visualization_msgs::Marker>(topic, nh);

  cout << "Is this the path you want?\n";
  cout << "1: Original, 2: Downsampled, 3: Cropped, 4: Plane, 5: No plane, "
       << "6: cluster(s)\n";

  getline(cin, reply);

  while (reply != "g" )
  {
    visualization_msgs::Marker::ConstPtr mk_points =
            ros::topic::waitForMessage<visualization_msgs::Marker>(topic, nh);

    cout << "Is this the path you want?\n";
    cout << "1: Original, 2: Downsampled, 3: Cropped, 4: Plane, 5: No plane, "
         << "6: cluster(s)\n";

    getline(cin, reply);
  }


  EigenSTL::vector_Affine3d pattern_poses;


  for (unsigned int i = 0; i < (*mk_points).points.size(); ++i)
  {
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();

    // Add 0.033 m (or 3.3cm) to z, make sure the torch is clear of the workpiece
    // before the robot arm is calibrated.
    pose.translation() = Eigen::Vector3d((*mk_points).points[i].x-0.0035,
                                         (*mk_points).points[i].y,
                                         (*mk_points).points[i].z+0.02);

    // I now let the Y axis be free to move and 
    // keep the Z axis 90 degree
    // and the X asis is Pi - 1.0
    pose *= Eigen::AngleAxisd((M_PI-0.65), Eigen::Vector3d::UnitX());
    pose *= Eigen::AngleAxisd(2.0, Eigen::Vector3d::UnitZ()); 
    pattern_poses.push_back(pose);
  }

  ROS_INFO("Start making Cartesian Points.");
  std::vector<descartes_core::TrajectoryPtPtr> result;
  for (const auto& pose : pattern_poses)
  {
    // This creates a trajectory that searches around the tool Y and let the
    // robot move in that null space

    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint
                             (pose, time_between_points);

    // This creates a trajectory that is rigid. The tool cannot float and must
    // be at exactly this point.

/*    descartes_core::TrajectoryPtPtr pt = makeCartesianPoint
                            (pose, time_between_points);
*/
    result.push_back(pt);
  }

  ROS_INFO("Finish making Cartesian Points.");

  // Note that we could also add a joint point representing the starting location
  // of the robot, or a joint point representing the desired end pose of the robot
  // to the front and back of the vector respectively.

  return result;
}

bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory,
                       ros::NodeHandle& nh)
{

  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    /* for ROS Control
                    ac ("/vel_based_pos_traj_controller/follow_joint_trajectory", true);
     */
    /* for Simulation
                    ac ("joint_trajectory_action", true);
     */
    /* for Real */
                      ac ("follow_joint_trajectory", true);
     /* */

  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }
  else
  {
    ROS_INFO("Connected to the action server.");
  }

//  trajectory_msgs::JointTrajectory joint_solution;
/*
  control_msgs::FollowJointTrajectoryGoal goal1;
  goal1.trajectory.points.push_back(trajectory.points[0]);
  goal1.trajectory.joint_names = trajectory.joint_names;
  goal1.goal_time_tolerance = ros::Duration(1.0);
ROS_INFO_STREAM("goal1.trajectory.points.size: " << goal1.trajectory.points.size());
*/
  int result;
/*
  ros::ServiceClient srv_SetIO = nh.serviceClient<ur_msgs::SetIO>
                                                ("ur_driver/set_io");

  ur_msgs::SetIO srv;
*/

  // Before doing that we should move the arm to the starting position first
  // then switch on the Laser Torch

  // Now go to the starting point first
/*
  if (result = (ac.sendGoalAndWait(goal1) ==
                actionlib::SimpleClientGoalState::SUCCEEDED))
  {

    // Create a Service Client to switch ON and Off the Laser Torch
      ROS_INFO("Here 1.");


    // Set the Digital Output #0 to ON, that is 24V output
    srv.request.fun = 1;
    srv.request.pin = 0;
    srv.request.state = 1.0; // Set DO 0 to ON
    if (srv_SetIO.call(srv))
    {
      ROS_INFO("True: Switched Torch ON");
    }
    else
    {
      ROS_INFO("False: Failed to switch Torch ON");
    }

  }
  else
  {
    ROS_INFO("Failed to execute Goal1 !");
  }
*/
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;

  // When to start the trajectory: 1s from now
  // added by Victor W H Wu on 26 July 2019.
//  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(3.0);

 ROS_INFO_STREAM("time from start for point zero: " << goal.trajectory.points[0].time_from_start << "\n");
  goal.trajectory.points[0].time_from_start = ros::Duration(3.0);
 ROS_INFO_STREAM("After time from start for point zero: " << goal.trajectory.points[0].time_from_start << "\n");

  goal.goal_time_tolerance = ros::Duration(1.0);

  if (result = 
               (ac.sendGoalAndWait(goal) ==
                actionlib::SimpleClientGoalState::SUCCEEDED)
     )
  {
/*
    // After the execution succeeded Switch off the Torch
    srv.request.state = 0.0; // Set DO 0 to OFF
    if (srv_SetIO.call(srv))
    {
      ROS_INFO("True: Switched Torch OFF");
    }
    else
    {
      ROS_INFO("False: Failed to switch Torch OFF");
    }
*/

    printf("Execution State: %s\n", ac.getState().toString().c_str());

    // then Go Home

    moveit::planning_interface::MoveGroupInterface 
                               move_group("manipulator");

    move_group.setNamedTarget("allZeros");
    move_group.move();

    return result;
  }
}
