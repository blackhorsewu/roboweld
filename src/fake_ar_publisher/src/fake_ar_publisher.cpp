#include <ros/ros.h>
#include <fake_ar_publisher/ARMarker.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

ros::Publisher ar_pub;
ros::Publisher visual_pub;
/*
ros::Publisher x_axis_pub;
ros::Publisher y_axis_pub;
ros::Publisher z_axis_pub;
*/
ros::Publisher axes_pub;

static std::string& camera_frame_name()
{
  static std::string camera_frame;
  return camera_frame;
} 

// Singleton Instance of Object Position
static geometry_msgs::Pose& pose()
{
  static geometry_msgs::Pose pose;
  return pose;
}

// Given a marker w/ pose data, publish an RViz visualization
// You'll need to add a "Marker" visualizer in RVIZ AND define
// the "camera_frame" TF frame somewhere to see it.
static void pubVisualMarker(const fake_ar_publisher::ARMarker& m)
{
  const double width = 0.15;
  const double thickness = 0.005;
  
  // The fake AR Marker itself
  visualization_msgs::Marker marker;
  marker.header.frame_id = m.header.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "ar_marker_visual";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = m.pose.pose;
  marker.pose.position.z -= thickness / 2.0;
  marker.scale.x = 0.14;
  marker.scale.y = 0.14;
  marker.scale.z = thickness;
  marker.color.a = 1.0;
  marker.color.b = 1.0;

  // The axes of the fake AR Marker
  visualization_msgs::Marker x_axis, y_axis, z_axis;
  visualization_msgs::MarkerArray markers_msg;

  x_axis.header.frame_id = m.header.frame_id;
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
  x_axis.points.push_back(m.pose.pose.position);

  y_axis.header.frame_id = m.header.frame_id;
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
  y_axis.points.push_back(m.pose.pose.position);

  z_axis.header.frame_id = m.header.frame_id;
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
  z_axis.points.push_back(m.pose.pose.position);

  geometry_msgs::Point x_end_point, y_end_point, z_end_point;

  Eigen::Isometry3d eigen_pose = Eigen::Isometry3d::Identity();

  tf::poseMsgToEigen(m.pose.pose, eigen_pose);


  Eigen::Isometry3d move_along_x = eigen_pose * Eigen::Translation3d(0.1, 0, 0);
  tf::pointEigenToMsg(move_along_x.translation(), x_end_point);
  x_axis.points.push_back(x_end_point);

  Eigen::Isometry3d move_along_y = eigen_pose * Eigen::Translation3d(0, 0.1, 0);
  tf::pointEigenToMsg(move_along_y.translation(), y_end_point);
  y_axis.points.push_back(y_end_point);

  Eigen::Isometry3d move_along_z = eigen_pose * Eigen::Translation3d(0, 0, 0.1);
  tf::pointEigenToMsg(move_along_z.translation(), z_end_point);
  z_axis.points.push_back(z_end_point);

  markers_msg.markers.push_back(x_axis);
  markers_msg.markers.push_back(y_axis);
  markers_msg.markers.push_back(z_axis);

  axes_pub.publish(markers_msg);

  visual_pub.publish(marker);
}

void pubCallback(const ros::TimerEvent&)
{
  geometry_msgs::Pose p = pose();
  fake_ar_publisher::ARMarker m;
  m.header.frame_id = camera_frame_name();
  m.header.stamp = ros::Time::now();
  m.pose.pose = p;

  ar_pub.publish(m);
  
  pubVisualMarker(m); // visualize the marker
}

int main(int argc, char **argv)
{

  tf2::Quaternion q_orig, q_rot, q_new;

  // Set up ROS.
  ros::init(argc, argv, "fake_ar_publisher");
  ros::NodeHandle nh, pnh ("~");
  ar_pub = nh.advertise<fake_ar_publisher::ARMarker>("ar_pose_marker", 1);

  visual_pub = nh.advertise<visualization_msgs::Marker>("ar_pose_visual", 1);

  axes_pub = nh.advertise<visualization_msgs::MarkerArray>("axes",1);

  // init pose
  pose().orientation.w = 1.0; // facing straight up

  // Get the original orientation of 'commanded_pose'
  tf2::convert(pose().orientation , q_orig);


/*


//  double r=0, p=0, y=3.1416;  // Rotate the previous pose by 180* about Z
  double r=3.1416, p=0, y=0;  // Rotate the previous pose by 180* about X
  q_rot.setRPY(r, p, y);

  q_new = q_rot*q_orig;  // Calculate the new orientation
  q_new.normalize();

  q_orig = q_new;

  r=1.5708; p=0; y=0; // Rotate the previous pose by 90 degree about X
  q_rot.setRPY(r, p, y);
  q_new = q_rot*q_orig;
  q_new.normalize();

  double r=0, p=0, y=-0.7854; // Rotate the previous pose by 45 degree about Z
  q_rot.setRPY(r, p, y);
  q_new = q_rot*q_orig;
  q_new.normalize();

  // Stuff the new rotation back into the pose.
  // This requires conversion into a msg type
  tf2::convert(q_new, pose().orientation);

  Do not change the orientation
*/
/*
  pnh.param<double>("x_pos", pose().position.x, 0.65);
  pnh.param<double>("y_pos", pose().position.y, -0.2535);
  pnh.param<double>("z_pos", pose().position.z, 1.1465);


  pnh.param<double>("x_pos", pose().position.x, -0.65);
  pnh.param<double>("y_pos", pose().position.y, 0.15);
  pnh.param<double>("z_pos", pose().position.z, 1.1465);
*/
  pnh.param<double>("x_pos", pose().position.x, -0.20);
  pnh.param<double>("y_pos", pose().position.y, -0.30);
//  pnh.param<double>("z_pos", pose().position.z, 0.9515);
  pnh.param<double>("z_pos", pose().position.z, 0.9465);


  pnh.param<std::string>("camera_frame", camera_frame_name(),
                         "kinect2_rgb_optical_frame");

  ROS_INFO("Starting simulated ARMarker publisher");  
  ros::Timer t = nh.createTimer(ros::Duration(0.1), pubCallback, false, true);
  ros::spin();
}
