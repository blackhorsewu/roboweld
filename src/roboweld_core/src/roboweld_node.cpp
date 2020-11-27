#include <ros/ros.h>
#include <roboweld_core/LocalizePart.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <roboweld_core/PlanCartesianPath.h>

#include <eigen_conversions/eigen_msg.h>
#include <vector>

using namespace std;

class ScanNPlan
{
public:
/* For simulation on RViz only 
  ScanNPlan(ros::NodeHandle& nh) : ac_("joint_trajectory_action", true)
*/

/* For using the real hardware UR5 */
  ScanNPlan(ros::NodeHandle& nh) : 
        ac_("scaled_vel_joint_traj_controller/follow_joint_trajectory", true)

  {
    vision_client_ = nh.serviceClient<roboweld_core::LocalizePart>("localize_part");
    cartesian_client_ = nh.serviceClient<roboweld_core::PlanCartesianPath>("plan_path");
  }

  void start(const std::string& base_frame, ros::NodeHandle& nh)
  {
    // Localize the part
    ROS_INFO("Attempting to localize part");
    roboweld_core::LocalizePart srv;
    srv.request.base_frame = base_frame;
//    geometry_msgs::Pose resp;
    ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);

    if (!vision_client_.call(srv))
    {
      ROS_ERROR("Could not localize part");
      return;
    }
    ROS_INFO_STREAM("part localized: " << srv.response);

    // Call this location the move_target
    geometry_msgs::Pose move_target = srv.response.pose;

/*
    // Instead of moving to the centre of the workpiece
    // move to the starting point of the workpiece
    move_target.position.x += 0.15/2.0; // assuming the workpiece is a square of 0.15m
    move_target.position.y += 0.15/2.0;
*/
    // lift the end effector up from the workpiece by 5mm
    move_target.position.z += 0.003;

//    resp = srv.response.pose;

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    // Plan for robot to move to part
    move_group.setPoseReferenceFrame(base_frame);

/*
    // Move to "up" pose first
    move_group.setNamedTarget("up");
    move_group.move();
*/

    // Move the End Effector to the centre of the workpiece first
    move_group.setPoseTarget(move_target);
    move_group.move();

    cout << "Enter when ready...\n";

    string reply = "";
    getline(cin, reply);

    /******** Plan cartesian path **********
     ***** plan use move_target as reference! *****
     ***** therefore do not change move_target ! *****
     */
    roboweld_core::PlanCartesianPath cartesian_srv;
    cartesian_srv.request.pose = move_target; 

    ROS_INFO("Start to plan path...");
    if (!cartesian_client_.call(cartesian_srv))
    {
      ROS_ERROR("Could not plan for path");
      return;
    }

    ROS_INFO("Got cart path.");
    control_msgs::FollowJointTrajectoryGoal goal, dummy_goal;
/*
 * Create a single point trajectory at the starting point.
 * Move the end effector to it first. Then switch on the welding machine and wait
 * a bit of time before start moving along the whole trajectory.


    dummy_goal.trajectory.points.push_back(cartesian_srv.response.trajectory.points[0]);
    dummy_goal.trajectory.joint_names = cartesian_srv.response.trajectory.joint_names;
    dummy_goal.goal_time_tolerance = ros::Duration(1.0);
    ROS_INFO_STREAM("Dummy goal size: " << dummy_goal.trajectory.points.size());
    
    // move to the starting point first
    int result;
    if (result = (ac_.sendGoalAndWait(dummy_goal) ==
                  actionlib::SimpleClientGoalState::SUCCEEDED))
      {
        ROS_INFO("Moved to the first point");
      }
    else
      {
        ROS_INFO("Failed to move to the first point");
      }
    ac_.waitForResult();

    cout << "Enter when ready...\n";

//    string reply = "";
    getline(cin, reply);
 */

    goal.trajectory = cartesian_srv.response.trajectory;

    // Find the number of joints (it should always be 6).
    auto n_joints = goal.trajectory.points.front().positions.size();

    ROS_INFO_STREAM("No. of joints: " << n_joints);

    // Print out the time from start for each of the waypoints along the path
    for (auto i = 0; i < goal.trajectory.points.size(); i++)
      ROS_INFO_STREAM("point " << i << " : time: "
                      << goal.trajectory.points[i].time_from_start);

    // Work out the angular velocity of each joint for each waypoint
    // For each of the joints
    for (auto i = 0; i < n_joints; ++i)
    {
      ROS_INFO_STREAM("Joint: " << i );
      for (auto j = 1; j < goal.trajectory.points.size() - 1; j++)
      {
        // For each point in a given joint
        ROS_INFO_STREAM("Joint: " << i << " point: " << j);
        // Find the difference of joint angles between the next position and the last position
        double delta_theta = - goal.trajectory.points[j - 1].positions[i]
                             + goal.trajectory.points[j + 1].positions[i];
        // Find the difference of time from start between the next and last point
        double delta_time = - goal.trajectory.points[j - 1].time_from_start.toSec()
                            + goal.trajectory.points[j + 1].time_from_start.toSec();
        // Work out the angular velocity by dividing the angle with the time
        double v = delta_theta / delta_time;
        // Use that as the angular velocity of the current point for this joint
        goal.trajectory.points[j].velocities[i] = v;
      }
    }

    cout << "Enter when ready...\n";

//    string reply = "";
    getline(cin, reply);


/*  Finish added code */

    // Execute descartes-planned path directly (bypassing MoveIt)
    ac_.sendGoal(goal);
    ac_.waitForResult();
    ROS_INFO("Done");
  }

private:
  // Planning components
  ros::ServiceClient vision_client_;
  ros::ServiceClient cartesian_client_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roboweld_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_node_handle("~");
  ros::AsyncSpinner async_spinner (1);
  async_spinner.start();

  ROS_INFO("ScanNPlan node has been initialized");

  std::string base_frame;
  // parameter name, string object reference, default value
  private_node_handle.param<std::string>("base_frame", base_frame, "world");

  ScanNPlan app(nh);

  ros::Duration(0.5).sleep();  // wait for the class to initialize
  app.start(base_frame, nh);

  // ros::waitForShutdown();
}
