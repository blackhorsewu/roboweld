/**********************************************************************************
 *                The Chinese National Engineering Research Centre                *
 *                                Steel Construction                              *
 *                                (Hong Kong Branch)                              *
 *                                                                                *
 *                Department of Civil and Environmental Engineering               *
 *                        Hong Kong Polytechnic University                        *
 *                                                                                *
 *                                  Robotic Welding                               *
 *                                                                                *
 * Victor W H Wu                                                                  *
 * 21 June 2019.                                                                  *
 *                                                                                *
 * Perception node for locating and identifying welding groove in ROS.            *
 *                                                                                *
 * Version: 2.2                                                                   *
 *   A clean up version and try to merge with the bigger package of welding.      *
 *                                                                                *
 *                                                                                *
 **********************************************************************************/

/**********************************************************************************
 *                                                                                *
 * This node assumes that the Kinect One v2 to ROS bridge has already been        *
 * running so that it publishes the images and point clouds for use.              *
 * It also assumes the robot arm and the RViz are running with the tf for the     *
 * camera frames are readily available.                                           *
 *                                                                                *
 * It subscribes to point clouds from the Kinect v2 RGBD  (Red Green Blue, in     *
 * other words Colour and D for Depth) camera.                                    *
 * The first thing it will do with the point cloud will be to transform it to the *
 * "world" reference frame. Then it will be down sampled to a manageable size/    *
 * resolution. In this process any noise in the form of NAN (Not a Number) will   *
 * be filtered. Kinect v2 has a fairly wide angle of view and can see quite a lot *
 * besides the table of the robotic arm and the workpieces. So, we will crop the  *
 * point cloud and concentrate to the area of interest. Since the workpiece will  *
 * be place on the table, we will make sure that we only process objects above    *
 * the table. The first assumption at this point of the project is that the       *
 * workpiece is flat and we are going to do butt joints only, for the time being. *
 * So, we can use the PCL (Point Cloud Library) to segment the plane that         *
 * represent the flat workpiece. Once we got the plane we can take it away. The   *
 * groove will be left behind. The next thing is to use this point cloud of the   *
 * groove to generate the side points for ROS to plan and execute the path for    *
 * the UR3 robotic arm to follow. That belongs to another package - cartesian     *
 * node.                                                                          *
 *                                                                                *
 **********************************************************************************/



#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

// PCL (Point Cloud Library) specific includes
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"

#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/bilateral.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "geometry_msgs/Point.h"
#include <vector>

// These include files are for Markers to be displayed in rviz
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>

// Visual Tools
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <string>

using namespace std;

/**********************************************************************************
 *                                                                                *
 * compareX and compareY are newly added                                          *
 *                                                                                *
 **********************************************************************************/


bool compareY(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
  return(p1.y < p2.y);
}

bool compareX(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  return(p1.x < p2.x);
}

/**********************************************************************************
 *                                                                                *
 * The Main of this package starts here.                                          *
 *                                                                                *
 **********************************************************************************/

int main(int argc, char *argv[])
{ // Begin of Main
  /*
   * INITIALIZE ROS NODE
   */
  ros::init(argc, argv, "perception_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");

  /*
   * Set up parameters (Could be input from launch file/terminal)
   */

  /*
   * Parameters for the cloud topic and the reference frames
   */
  std::string cloud_topic, world_frame, camera_frame;

//  cloud_topic = priv_nh_.param<std::string>("cloud_topic", "kinect2/qhd/points");
  cloud_topic = priv_nh_.param<std::string>("cloud_topic", "kinect2/hd/points");
  world_frame = priv_nh_.param<std::string>("world_frame", "world");
  camera_frame = priv_nh_.param<std::string>("camera_frame", "kinect2_link");

  /*
   * the cube size of each volumn element
   */
  float voxel_leaf_size; // default to 2mm cube
  voxel_leaf_size = priv_nh_.param<float>("voxel_leaf_size", 0.003);

  /*
   * we need to specify how much we want to see, ie how to crop the image
   */
  float x_filter_min, x_filter_max, y_filter_min, y_filter_max, 
        z_filter_min, z_filter_max;

  x_filter_min = priv_nh_.param<float>("x_filter_min", 0.13);
  x_filter_max = priv_nh_.param<float>("x_filter_max",  0.90);
  y_filter_min = priv_nh_.param<float>("y_filter_min",  -0.25);
  y_filter_max = priv_nh_.param<float>("y_filter_max",  0.50);
//  z_filter_min = priv_nh_.param<float>("z_filter_min",  0.00046);
  z_filter_min = priv_nh_.param<float>("z_filter_min",  0.007);
  z_filter_max = priv_nh_.param<float>("z_filter_max",  0.023);
//  z_filter_max = priv_nh_.param<float>("z_filter_max",  0.008);

  /*
   * parameters for the plane segmentation
   */
  int plane_max_iter;
  float plane_dist_thresh;

  plane_max_iter = priv_nh_.param<int>("plane_max_iterations", 550);
  plane_dist_thresh = priv_nh_.param<float>("plane_distance_threshold", 0.004);

  /*
   * parameters for clustering, to identify the welding groove
   */
  float cluster_tol;
  int cluster_min_size;
  int cluster_max_size;

  cluster_tol = priv_nh_.param<float>("cluster__tolerance_threshold", 0.004);
  cluster_min_size = priv_nh_.param<int>("cluster_min_size", 1000);
  cluster_max_size = priv_nh_.param<int>("cluster_max_size", 20000);

  /*
   * Setup publisher to publish point clouds to RViz
   */
  ros::Publisher object_pub;
  object_pub = nh.advertise<sensor_msgs::PointCloud2>("object_cluster", 1);

  // setup point Marker message
  visualization_msgs::Marker mk_points;

  mk_points.header.frame_id = world_frame;
  mk_points.ns = "points_and_lines";
  mk_points.action = visualization_msgs::Marker::ADD;
  mk_points.pose.orientation.w = 1.0;
  mk_points.id = 0;
  mk_points.type = visualization_msgs::Marker::POINTS;
  mk_points.scale.x = 0.008; // so the point is shown as of 8mm cube
  mk_points.scale.y = 0.008; // make it slightly bigger so that the user can 
//  mk_points.scale.z = 0.008; // see them.
  mk_points.color.r = 1.5;   // in red
  mk_points.color.a = 1.5;   // 

  // set up my curve profile Marker publisher
  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("myCurve", 100);

  // set up a way points Marker publisher
  ros::Publisher wypts_pub = nh.advertise<visualization_msgs::Marker>("wayPoints", 100);

  // Visual tools
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().x() = 0.15;
  text_pose.translation().z() = 0.75;
  visual_tools.publishText(text_pose, "RoboWeld PointCloud Calibration", rvt::WHITE, rvt::XLARGE, false);
  visual_tools.trigger();

 while (ros::ok())
 { // Begin of infinite loop

   /*
    * Listen for point cloud
    */
   std::string topic = nh.resolveName(cloud_topic);
   ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic "<< topic);

   /*
    * recent_cloud is the raw point cloud received from the camera
    */
   sensor_msgs::PointCloud2::ConstPtr recent_cloud =
                ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);

   /*
    * Transform PointCloud from Camera Frame to World Frame
    */
   tf::TransformListener listener;
   tf::StampedTransform  stransform;
   try
   {
     listener.waitForTransform(world_frame,
                               recent_cloud->header.frame_id,
                               ros::Time::now(),
                               ros::Duration(2.0));
     listener.lookupTransform (world_frame,
                               recent_cloud->header.frame_id,
                               ros::Time(0),
                               stransform);
   }
   catch (tf::TransformException ex)
   {
     ROS_ERROR("%s",ex.what());
   }
   sensor_msgs::PointCloud2 transformed_cloud;

   pcl_ros::transformPointCloud(world_frame,
                                stransform,
                                *recent_cloud,
                                transformed_cloud);

   /*
    * Convert Point Cloud from ROS format to PCL format
    */
   pcl::PointCloud<pcl::PointXYZ> cloud;
   // Added on 17 November 2020. ROS calls the field "intensities" while PCL calls
   // the same field "intensity". Therefore it needs to be renamed before the 
   // conversion otherwise the intensity will be lost.

   //transformed_cloud.fields[3].name = "intensity";

   pcl::fromROSMsg (transformed_cloud, cloud);

   /*
    * cloud_ptr is the transformed_cloud in PCL format
    */
   pcl::PointCloud<pcl::PointXYZ>::Ptr 
                        cloud_ptr(new pcl::PointCloud<pcl::PointXYZ> (cloud));

   /*******************************************************************************
    *                         Downsample the point cloud                          *
    * Note:                                                                       *
    *   We must do the downsample first or voxel filter first.                    *
    * The main reason is to filter out all the noise, i.e. all                    *
    * all the NAN. Otherwise the noise will remain in the point                   *
    * cloud and cause a lot of trouble later on.                                  *
    *******************************************************************************/

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered 
                     (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

    voxel_filter.setInputCloud (cloud_ptr);
    voxel_filter.setLeafSize (voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_filter.filter (*cloud_voxel_filtered);

    ROS_INFO_STREAM("Original cloud  had " << cloud_ptr->size() << " points");
    ROS_INFO_STREAM("Downsampled cloud  with " << cloud_voxel_filtered->size()
                                               << " points");


  /*********************************************************************************
   *                                                                               *
   *  PASSTHROUGH FILTER(S)                                                        *
   *                                                                               *
   *  Now that we have filtered the noise then we can concentrate on the           *
   *  working area. Crop the image to focus on our intereset, that is the welding  *
   *  table.                                                                       *
   *  The world is at the bottom of the robot                                      *
   *  The x direction is pointing forward from the robot                           *
   *  The y direction is pointing to the left from the robot                       *
   *  The z direction is pointing upward                                           *
   *  The table centre is 0.33 from the world in the y direction                   *
   *  and at 0 in the x direction and -ve half the thickness of the table top      *
   *  in the z direction.                                                          *
   *  Everything outside this box will not be processed.                           *
   *                                                                               *
   *********************************************************************************/

    //filter in x
    pcl::PointCloud<pcl::PointXYZ> xf_cloud, yf_cloud, zf_cloud;
    pcl::PassThrough<pcl::PointXYZ> pass_x;

    pass_x.setInputCloud(cloud_voxel_filtered);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_filter_min, x_filter_max);
    pass_x.filter(xf_cloud);

    //filter in y
    pcl::PointCloud<pcl::PointXYZ>::Ptr xf_cloud_ptr
                               (new pcl::PointCloud<pcl::PointXYZ>(xf_cloud));
    pcl::PassThrough<pcl::PointXYZ> pass_y;

    pass_y.setInputCloud(xf_cloud_ptr);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_filter_min, y_filter_max);
    pass_y.filter(yf_cloud);

    //filter in z 
    pcl::PointCloud<pcl::PointXYZ>::Ptr yf_cloud_ptr
                               (new pcl::PointCloud<pcl::PointXYZ>(yf_cloud));
    pcl::PassThrough<pcl::PointXYZ> pass_z;

    pass_z.setInputCloud(yf_cloud_ptr);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_filter_min, z_filter_max);
    pass_z.filter(zf_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr croped_cloud_ptr 
                              (new pcl::PointCloud<pcl::PointXYZ> (zf_cloud));

    ROS_INFO_STREAM("Croped cloud  had " << croped_cloud_ptr->size() << " points");


   /*********************************************************************************
    *                                                                               *
    * Trying to add in the Bilaterial Filter, hopefully it will increase the clarity*
    * of the point clouds.                                                          *
    *                                                                               *
    * Commented out for it takes a long time and not much improvement!              *
    *                                                                               *
    *********************************************************************************/
/*
    typedef pcl::PointXYZ PointT;

    pcl::PointCloud<PointT>::Ptr outcloud = croped_cloud_ptr;
    // pcl::PointCloud<PointT>::Ptr outcloud_ptr;
    //                          (new pcl::PointCloud<PointT> (outcloud));

    // Set up KDTree
    pcl::search::KdTree<PointT>::Ptr tree1 (new pcl::search::KdTree<PointT>);
    
    pcl::BilateralFilter<PointT> bf;

    ROS_INFO("Going to do Bilateral Filterring.");

    bf.setInputCloud(croped_cloud_ptr);
    bf.setSearchMethod(tree1);
    bf.setHalfSize(1.0);
    bf.setStdDev(0.2);
    bf.filter(*outcloud);

    ROS_INFO("Bilateral Filtered.");
*/
  /*********************************************************************************
   *                                                                               *
   *                               Plane Segmentation                              *
   *                                                                               *
   *********************************************************************************/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud = croped_cloud_ptr;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud = outcloud;

    // Create the segmentation object for the planar model
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    // Set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (plane_max_iter);
    seg.setDistanceThreshold (plane_dist_thresh);

    // Segment the largest planar component from the cropped cloud
    seg.setInputCloud (cropped_cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      ROS_WARN_STREAM("Could not estimate a planar model for the given dataset.");
      //break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud (cropped_cloud);
    extract.setIndices(inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane 
                           (new pcl::PointCloud<pcl::PointXYZ>());

    extract.filter (*cloud_plane);
    ROS_INFO_STREAM("PointCloud representing the planar component: " 
                    << cloud_plane->points.size() << " data points." );

    // Remove the planar inliers, extract the rest
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f 
                           (new pcl::PointCloud<pcl::PointXYZ>());

    extract.setNegative (true);
    // cloud_f now contains the point cloud without the table top.
    extract.filter (*cloud_f);

  /*********************************************************************************
   *                                                                               *
   *                               Euclidean Cluster Extraction                    *
   *                                                                               *
   *********************************************************************************/

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr 
                        tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr 
                        cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

  *cloud_filtered = *cloud_f;
  tree->setInputCloud (cloud_filtered);

  /*********************************************************************************
   *                                                                               *
   * There could be none or more clusters. Each cluster is represented by a        *
   * PointIndices (meaning they are indices into the point cloud). We use a vector *
   * to store this list of clusters. So, before we make use of this list we must   *
   * be sure that it is not empty.                                                 *
   *                                                                               *
   *********************************************************************************/

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (cluster_tol); //  
    ec.setMinClusterSize (cluster_min_size);
    ec.setMaxClusterSize (cluster_max_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    // pc2_clusters is a vector of clusters in ROS format
    std::vector<sensor_msgs::PointCloud2::Ptr> pc2_clusters;
    // clusters is a vector of clusters in PCL format
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;

    // cluster indices is a vector of indices of a point cloud.
    // this point cloud in this case is the "cloud_filtered"
    for (std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin();
                   it != cluster_indices.end(); ++it)
    { // iterate over number of clusters found
      // "cloud_cluster" is the individual cluster and copy from "cloud filtered"
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster
                             (new pcl::PointCloud<pcl::PointXYZ>());

      for (std::vector<int>::const_iterator pit = it->indices.begin();
                   pit != it->indices.end(); pit++)
      { // iterate over each point in the cluster
        // copy each point from "cloud_filtered" to "cloud_cluster"
        cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
      }

      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      std::cout << "Cluster has " << cloud_cluster->points.size() << " points.\n";
      clusters.push_back(cloud_cluster);

      sensor_msgs::PointCloud2::Ptr tempROSMsg(new sensor_msgs::PointCloud2);

      pcl::toROSMsg(*cloud_cluster, *tempROSMsg);

      pc2_clusters.push_back(tempROSMsg);
    }

    /************************************************************************
     * Before we go into dealing with the clusters, let us find out what we *
     * want to do first.                                                    *
     ************************************************************************/

    /****************************************************
     * Convert PointCloud from PCL format to ROS format *
     * then publish the cloud                           *
     ****************************************************/

    sensor_msgs::PointCloud2::Ptr pc2_cloud (new sensor_msgs::PointCloud2);

    cout << "Which point cloud to display?\n";
    cout << "1: Original, 2: Downsampled, 3: Cropped, 4: Plane, 5: No plane, "
         << "6: cluster(s)\n";

    string reply;
    getline(cin, reply);

    if (reply == "1")
    { // use cloud_ptr
      pcl::toROSMsg(*cloud_ptr, *pc2_cloud);
    }
    else if (reply == "2")
    { // use zf_cloud
      pcl::toROSMsg(*cloud_voxel_filtered, *pc2_cloud);
    }
    else if (reply == "3")
    { // use cloud_voxel_filtered
      pcl::toROSMsg(*croped_cloud_ptr, *pc2_cloud);
    }
    else if (reply == "4")
    { // use cloud_plane
      pcl::toROSMsg(*cloud_plane, *pc2_cloud);
      
      float min_z=1.0;
      float max_z=0.0;
      float avg_z=0.0;
      int i=0;

      while (i < cloud_plane->points.size())
        {
          if (cloud_plane->points[i].z<min_z)
          {
            min_z=cloud_plane->points[i].z;
          }
          if (cloud_plane->points[i].z>max_z)
          {
            max_z=cloud_plane->points[i].z;
          }
          avg_z+=cloud_plane->points[i].z;
          i++;
        }
      avg_z=avg_z/cloud_plane->points.size();

      cout << "Min z: " << min_z << "\n Max z: " << max_z << "\n Avg z: " << avg_z << "\n";
    }
    else if (reply == "5")
    { // use cloud_plane
      pcl::toROSMsg(*cloud_f, *pc2_cloud);
    }
    else if (reply == "6")
    {
      if (pc2_clusters.size() == 0)
      {
        cout << "No cluster was found!\n";
      }
      else
      {
        cout << "There are " << pc2_clusters.size() << " clusters.\n";
        for (int i = 0; i < clusters.size(); i++)
        {
          pcl::toROSMsg(*(clusters.at(i)), *pc2_cloud);
          pc2_cloud->header.frame_id=world_frame;
          pc2_cloud->header.stamp=ros::Time::now();
          object_pub.publish(pc2_cloud);

          cout << "Is this the cluster you want?\n";

          string reply1;
          getline(cin, reply1);

          if (reply1 == "y")
          { // begin if yes

            // call a function to setup the way points.
            // use the clusters.at(i) as the input parameter
            // do this later.

            /***********************************************
             * Find the begin, end and intermediate points *
             ***********************************************/

            pcl::PointCloud<pcl::PointXYZ>::Ptr myCurve
                                    (new pcl::PointCloud<pcl::PointXYZ>);
            myCurve = clusters.at(i);

            /****************************************
             * Sort the points in the "y" direction *
             ****************************************/

            /* Meaning the points are now from right to left */

            sort(myCurve->points.begin(), myCurve->points.end(), compareY);

            // Number of points in the cluster
            int no_points = myCurve->points.size();

            float begin_y, end_y;
            float step_y = 0.005; // 5 mm apart in the Y direction
            geometry_msgs::Point begin_pt, end_pt; // Begin and end points

            ROS_INFO_STREAM("No of Points in this cluster: " << no_points);


            begin_pt.x = myCurve->points[0].x;
            begin_pt.y = myCurve->points[0].y;
            begin_pt.z = myCurve->points[0].z;

            end_pt.x = myCurve->points[no_points-1].x;
            end_pt.y = myCurve->points[no_points-1].y;
            end_pt.z = myCurve->points[no_points-1].z;

            begin_y = begin_pt.y;
            end_y = end_pt.y;

            ROS_INFO_STREAM("\nInitial point:\n" << begin_pt <<
                            "End point:\n" << end_pt);
            ROS_INFO_STREAM("step y: " << step_y << "m.\n");

            // There can be more than one X groups
            // That is the problem because the borders from 
            // the top and bottom of the workpiece can have groups of points.

            int pt_indices = 0;
            float yPos = begin_y; // yPos is position pointer in the Y direction.
            geometry_msgs::Point p, this_pt, d_pt;

            // sweep through the points from right to left, ie in the +ve y direction
            // all points within -1 to +1 cluster_tol (cluster tolerance) range of y
            // copy all the points within the range
            while ((myCurve->points[pt_indices].y < end_pt.y) && (pt_indices < no_points))
            { // begin outer while in the cluster

              visualization_msgs::Marker X_buf;
              std::vector<int> nrs_pts;
              std::vector<geometry_msgs::Point> deep_pts;
              std::vector<float> begin_xs;
              std::vector<float> end_xs;

              while ((myCurve->points[pt_indices].y < (yPos + cluster_tol))
                                             && (pt_indices < no_points))
              { // begin while in step section

                // still within the bounds, since we have already sorted
                // the points in the y direction, so y will only increase

                // make a copy of the point
                p.x = myCurve->points[pt_indices].x;
                p.y = myCurve->points[pt_indices].y;
                p.z = myCurve->points[pt_indices].z;

                // put the points in the X collect_buf
                X_buf.points.push_back(p);

                // increment the point indices
                pt_indices++;
              } // end while

              // when it exits the loop meaning the current range of y finished
              /* The range is within the clustering tolerance of the point */

              // Be careful here, there can be no points at all within the range!
              // This occurs when the step size is smaller than the clustering tolerance

   ROS_INFO_STREAM("There are " << X_buf.points.size() << " points in the X collection.\n ");

              // Sort the collection of points in the X direction.
              // from nearest to farest from the robot arm.
              sort(X_buf.points.begin(), X_buf.points.end(), compareX);

   ROS_INFO_STREAM("X collection sorted.\n ");
   ROS_INFO_STREAM("There are " << X_buf.points.size() << " points after sorting.\n ");
   ROS_INFO_STREAM("1st y: " << X_buf.points[0].y << " last y: "
                             << X_buf.points[X_buf.points.size()-1].y);
   ROS_INFO_STREAM("1st x: " << X_buf.points[0].x << " last x: "
                             << X_buf.points[X_buf.points.size()-1].x);

              bool throw_away = false;
              int i = 0;
              // We then loop through this collection of points in the Y direction
              while (i < X_buf.points.size())
              { // begin while in this collection

                // start with this point, it is also the deepest point, number of points
                // in this cluster is 0.
                int no_pts = 0;
                this_pt = X_buf.points[i];
                d_pt = this_pt;
                float last_x = this_pt.x;
                float begin_x = this_pt.x;

                // loop through group(s) in the X direction
                while ((abs(last_x - this_pt.x) < (2.0 * cluster_tol))
                                          && (i < X_buf.points.size()))
                { // begin inner while in this group

                  // this point becomes the last point
                  last_x = this_pt.x;
                  // check if this is the deepest point of this group
                  if (this_pt.z < d_pt.z)
                  {
                    d_pt = this_pt;
                  }
                  // increment the number of points
                  // move onto the next point
                  no_pts++; i++;
                  this_pt = X_buf.points[i];
                } // end inner while - end with the group

                // this group finished.

                // if the distance between the beginning and end of the this cluster
                // is bigger than 20 mm then neglect this cluster.
                if (abs(begin_x - last_x) > 0.040)
                { // this is too long in the X direction to be a groove
                  throw_away = true;
                }

                nrs_pts.push_back(no_pts);
                deep_pts.push_back(d_pt);
                begin_xs.push_back(begin_x);
                end_xs.push_back(last_x);

     ROS_INFO_STREAM("There are " << no_pts << " points in this cluster.\n ");

              } // end while

              // Done, all the points in this collection have been examined.

   ROS_INFO_STREAM("There are " << nrs_pts.size() << " clusters in this collection.\n ");

              if (throw_away)
              {
                // proceed to the next unit of x (cluster_tol away)
                yPos += (2.0 * cluster_tol);
                while (myCurve->points[pt_indices].y < (yPos-cluster_tol))
                {
                  pt_indices++;
                }
                throw_away = false;
              }
              else
              { // begin else Normal group

                // For every group (there could be more than one group) find the one
                // with the most points. i_most is the index of the group with most points.
                int no_pts = 0, i_most;
                for (int i=0; i < nrs_pts.size(); i++)
                {
                  if (nrs_pts[i] > no_pts)
                  {
                    no_pts = nrs_pts[i];
                    i_most = i;
                  }
                }

   ROS_INFO_STREAM("The biggest group is: " << i_most << "th group\n ");
   ROS_INFO_STREAM(" with " << no_pts << " points\n ");
   ROS_INFO_STREAM("Begin X: " <<  begin_xs[i_most] << " End X: " << end_xs[i_most] << "\n ");

                // the point that we are interested in is
                mk_points.points.push_back(deep_pts[i_most]);

                // Let me make this point up. Use the yPos as the y, use
                // middle of Begin X and End X as x and deep_pts[i_most].z
                // for z. Or we can just keep deep_pts[i_most] only change X
                deep_pts[i_most].x = ((begin_xs[i_most] + end_xs[i_most])/2.0);

                // If this point is too far away (2 cm) from the previous mk_points 
                // then throw it away.
                // Only if the mk_points is not empty.
                if (!mk_points.points.empty())
                {

   ROS_INFO_STREAM("The i most y: " << deep_pts[i_most].y << "\n ");
   ROS_INFO_STREAM("The mk last y: " << mk_points.points.back().y << "\n ");

                  if (abs(deep_pts[i_most].y - mk_points.points.back().y) < 0.04)
                    // this point is less than 2 cm from the previous point
                    mk_points.points.push_back(deep_pts[i_most]);
                }
                else // mk_points is empty
                  mk_points.points.push_back(deep_pts[i_most]);

                // proceed to the next range of y
                yPos += step_y;
                while (myCurve->points[pt_indices].y < (yPos-cluster_tol))
                {
                  pt_indices++;
                }
              } // end else - finish normal group

            } // end outer while - finish with the whole cluster.

            // pop the last point before publish it
            // mk_points.points.pop_back();

            int nrmkpts = mk_points.points.size();

ROS_INFO_STREAM("Just before publishing the mk_points: " << nrmkpts);

            // publish the Marker points as way points to be shown on RViz
            wypts_pub.publish(mk_points);

        
        char text2[100];

        int n = sprintf(text2, "P1\n x: %5.2f\n y: %5.2f\n z: %5.2f",
                        begin_pt.x * 1000, begin_pt.y * 1000, begin_pt.z * 1000);

        string out_text2(text2);

        Eigen::Isometry3d text_pose2 = Eigen::Isometry3d::Identity();

        text_pose2.translation().x() = begin_pt.x;
        text_pose2.translation().y() = begin_pt.y;
        text_pose2.translation().z() = begin_pt.z + 0.15;

        visual_tools.publishText(text_pose2, out_text2, rvt::WHITE, rvt::XLARGE, false);

        char text3[100];

        n = sprintf(text3, "P2\n x: %5.2f\n y: %5.2f\n z: %5.2f",
                    end_pt.x * 1000, end_pt.y * 1000, end_pt.z * 1000);

        std::string out_text3(text3);

        Eigen::Isometry3d text_pose3 = Eigen::Isometry3d::Identity();
        text_pose3.translation().x() = end_pt.x;
        text_pose3.translation().y() = end_pt.y;
        text_pose3.translation().z() = end_pt.z + 0.15;

        visual_tools.publishText(text_pose3, out_text3, rvt::WHITE, rvt::XLARGE, false);

        double distance;
        Eigen::Isometry3d text_pose4 = Eigen::Isometry3d::Identity();

        distance = sqrt(
                        pow(abs(end_pt.x - begin_pt.x), 2) +
                        pow(abs(end_pt.y - begin_pt.y), 2) + 
                        pow(abs(end_pt.z - begin_pt.z), 2)
                       );

        char text4[100];

        n = sprintf(text4, "Distance:\n %5.2f mm", distance * 1000);

        std::string out_text4(text4);

        text_pose4.translation().x() = (begin_pt.x + end_pt.x)/2;
        text_pose4.translation().y() = (begin_pt.y + end_pt.y)/2;
        text_pose4.translation().z() = (begin_pt.z + end_pt.z)/2 + 0.40;

        visual_tools.publishText(text_pose4, out_text4, rvt::WHITE, rvt::XLARGE, false);

        visual_tools.trigger();

        ROS_INFO_STREAM("Begin point " << out_text4);

            pub.publish(mk_points);
            // remember to clear the mk_points otherwise it will carry on 
            // and shown on RViz as well as traverse by the robot arm again!
            mk_points.points.clear();
            break;
          } // end if "y"
        }
      }
    }
    else
    { // use cloud_ptr
      pcl::toROSMsg(*cloud_ptr, *pc2_cloud);
    }

    if (reply != "6")
    {
      pc2_cloud->header.frame_id=world_frame;
      pc2_cloud->header.stamp=ros::Time::now();
      object_pub.publish(pc2_cloud);
    }

  } // End of Infinite loop

  return 0;
} // End of Main
