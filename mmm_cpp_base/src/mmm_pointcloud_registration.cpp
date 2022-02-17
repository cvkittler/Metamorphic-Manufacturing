/*
TODO:
Collecting the point cloud from the topic
convert the pointcloud to a useable data type or class 
register the point cloud so it lines up with other ones (https://github.com/aosmundson/pcl-registration/tree/master/src)
combine the registered point clouds
proform planer model segmentation on a copy of the data to eliminate unimportant data
        in order to aid the registration process segmentation should happen on a differnt point cloud
close holes in the model (most likely the bottom)
publish the pointcloud 
*/

/*
Referances Used:
    http://wiki.ros.org/pcl/Tutorials
    http://wiki.ros.org/pcl/Overview
    https://github.com/aosmundson/pcl-registration/tree/master/src
*/

/* Default FAILED (rotated wrong)
Computing source cloud normals
Computing target cloud normals
Found 735 SIFT keypoints in source cloud
Found 996 SIFT keypoints in target cloud
Computed 735 FPFH features for source cloud
Computed 996 FPFH features for target cloud
Calculated transformation

REPEATABLE? YES

Computing source cloud normals
Computing target cloud normals
Found 735 SIFT keypoints in source cloud
Found 996 SIFT keypoints in target cloud
Computed 735 FPFH features for source cloud
Computed 996 FPFH features for target cloud
Calculated transformation
*/

/* Downsampled FAILED (rotated wrong)
PointCloud before filtering: 841455 data points (x y z).
PointCloud after filtering: 77908 data points (x y z).
Computing source cloud normals
Computing target cloud normals
Found 492 SIFT keypoints in source cloud
Found 929 SIFT keypoints in target cloud
Computed 492 FPFH features for source cloud
Computed 929 FPFH features for target cloud
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
// PCL and point cloud specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

#include <sstream>
// --------------------
// -----Global Vars----
// --------------------
ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZ>);

// --------------------
// -----Parameters-----
// --------------------
// SIFT Keypoint parameters
const float min_scale = 0.01f; // the standard deviation of the smallest scale in the scale space
const int n_octaves = 3;  // the number of octaves (i.e. doublings of scale) to compute
const int n_scales_per_octave = 4; // the number of scales to compute within each octave
const float min_contrast = 0.001f; // the minimum contrast required for detection

// Sample Consensus Initial Alignment parameters (explanation below)
const float min_sample_dist = 0.025f;
const float max_correspondence_dist = 0.01f;
const int nr_iters = 500;

// ICP parameters (explanation below)
const float max_correspondence_distance = 0.05f;
const float outlier_rejection_threshold = 0.05f;
const float transformation_epsilon = 0;
const int max_iterations = 100;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  output = *input;

  // Publish the data.
  pub.publish (output);
}

/* Use SampleConsensusInitialAlignment to find a rough alignment from the source cloud to the target cloud by fin    ding
 * correspondences between two sets of local features
 * Inputs:
 *   source_points
 *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
 *   source_descriptors
 *     The local descriptors for each source point
 *   target_points
 *     The "target" points, i.e., the points to which the source point cloud will be aligned                     
 *   target_descriptors
 *     The local descriptors for each target point
 *   min_sample_distance
 *     The minimum distance between any two random samples
 *   max_correspondence_distance
 *     The maximum distance between a point and its nearest neighbor correspondent in order to be considered
 *     in the alignment process
 *   nr_interations
 *     The number of RANSAC iterations to perform
 * Return: A transformation matrix that will roughly align the points in source to the points in target
 */
typedef pcl::PointWithScale PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::FPFHSignature33 LocalDescriptorT;
typedef pcl::PointCloud<LocalDescriptorT>::Ptr LocalDescriptorsPtr;

Eigen::Matrix4f computeInitialAlignment (const PointCloudPtr & source_points, const LocalDescriptorsPtr & source_descriptors,
                         const PointCloudPtr & target_points, const LocalDescriptorsPtr & target_descriptors,
                         float min_sample_distance, float max_correspondence_distance, int nr_iterations)
{
  pcl::SampleConsensusInitialAlignment<PointT, PointT, LocalDescriptorT> sac_ia;
  sac_ia.setMinSampleDistance (min_sample_distance);
  sac_ia.setMaxCorrespondenceDistance (max_correspondence_distance);
  sac_ia.setMaximumIterations (nr_iterations);

  sac_ia.setInputCloud (source_points);
  sac_ia.setSourceFeatures (source_descriptors);

  sac_ia.setInputTarget (target_points);
  sac_ia.setTargetFeatures (target_descriptors);

  PointCloud registration_output;
  sac_ia.align (registration_output);

  return (sac_ia.getFinalTransformation ());
}

void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],   
                            up_vector[0], up_vector[1], up_vector[2]); 
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr registerClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr){

  // Read pcd file
  pcl::PointCloud<pcl::PointXYZ>& source_cloud = *source_cloud_ptr;
  pcl::PointCloud<pcl::PointXYZ>& target_cloud = *target_cloud_ptr;

  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());

  // Estimate cloud normals
  cout << "Computing source cloud normals\n";
  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
  pcl::PointCloud<pcl::PointNormal>::Ptr src_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>& src_normals = *src_normals_ptr;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_xyz (new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setInputCloud(source_cloud_ptr);
  ne.setSearchMethod(tree_xyz);
  ne.setRadiusSearch(0.05);
  ne.compute(*src_normals_ptr);
  for(size_t i = 0;  i < src_normals.points.size(); ++i) {
      src_normals.points[i].x = source_cloud.points[i].x;
      src_normals.points[i].y = source_cloud.points[i].y;
      src_normals.points[i].z = source_cloud.points[i].z;
  }

  cout << "Computing target cloud normals\n";
  pcl::PointCloud<pcl::PointNormal>::Ptr tar_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>& tar_normals = *tar_normals_ptr;
  ne.setInputCloud(target_cloud_ptr);
  ne.compute(*tar_normals_ptr);
  for(size_t i = 0;  i < tar_normals.points.size(); ++i) {
      tar_normals.points[i].x = target_cloud.points[i].x;
      tar_normals.points[i].y = target_cloud.points[i].y;
      tar_normals.points[i].z = target_cloud.points[i].z;
  }

  // Estimate the SIFT keypoints
  pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
  pcl::PointCloud<pcl::PointWithScale>::Ptr src_keypoints_ptr (new pcl::PointCloud<pcl::PointWithScale>);
  pcl::PointCloud<pcl::PointWithScale>& src_keypoints = *src_keypoints_ptr;
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree_normal(new pcl::search::KdTree<pcl::PointNormal> ());
  sift.setSearchMethod(tree_normal);
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  sift.setInputCloud(src_normals_ptr);
  sift.compute(src_keypoints);

  cout << "Found " << src_keypoints.points.size () << " SIFT keypoints in source cloud\n";
 
  pcl::PointCloud<pcl::PointWithScale>::Ptr tar_keypoints_ptr (new pcl::PointCloud<pcl::PointWithScale>);
  pcl::PointCloud<pcl::PointWithScale>& tar_keypoints = *tar_keypoints_ptr;
  sift.setInputCloud(tar_normals_ptr);
  sift.compute(tar_keypoints);

  cout << "Found " << tar_keypoints.points.size () << " SIFT keypoints in target cloud\n";
  
  // Extract FPFH features from SIFT keypoints
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_keypoints_xyz (new pcl::PointCloud<pcl::PointXYZ>);                           
  pcl::copyPointCloud (src_keypoints, *src_keypoints_xyz);
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
  fpfh.setSearchSurface (source_cloud_ptr);
  fpfh.setInputCloud (src_keypoints_xyz);
  fpfh.setInputNormals (src_normals_ptr);
  fpfh.setSearchMethod (tree_xyz);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
  pcl::PointCloud<pcl::FPFHSignature33>& src_features = *src_features_ptr;
  fpfh.setRadiusSearch(0.05);
  fpfh.compute(src_features);
  cout << "Computed " << src_features.size() << " FPFH features for source cloud\n";

  pcl::PointCloud<pcl::PointXYZ>::Ptr tar_keypoints_xyz (new pcl::PointCloud<pcl::PointXYZ>);                           
  pcl::copyPointCloud (tar_keypoints, *tar_keypoints_xyz);
  fpfh.setSearchSurface (target_cloud_ptr);
  fpfh.setInputCloud (tar_keypoints_xyz);
  fpfh.setInputNormals (tar_normals_ptr);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr tar_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
  pcl::PointCloud<pcl::FPFHSignature33>& tar_features = *tar_features_ptr;
  fpfh.compute(tar_features);
  cout << "Computed " << tar_features.size() << " FPFH features for target cloud\n";
  
  // Compute the transformation matrix for alignment
  Eigen::Matrix4f tform = Eigen::Matrix4f::Identity();
  tform = computeInitialAlignment (src_keypoints_ptr, src_features_ptr, tar_keypoints_ptr,
          tar_features_ptr, min_sample_dist, max_correspondence_dist, nr_iters);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>& transformed_cloud = *transformed_cloud_ptr;
  pcl::transformPointCloud(source_cloud, transformed_cloud, tform);
  cout << "Calculated transformation\n";

  // Create 3D viewer and add point clouds
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor (0, 0, 0);
  int v1 (0);
  int v2 (1);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_cloud_color_handler (source_cloud_ptr, 255, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tar_cloud_color_handler (target_cloud_ptr, 0, 255, 255);
  viewer.addPointCloud (source_cloud_ptr, src_cloud_color_handler, "source cloud v1", v1);
  viewer.addPointCloud (target_cloud_ptr, tar_cloud_color_handler, "target cloud v1", v1);
  viewer.addPointCloud (target_cloud_ptr, tar_cloud_color_handler, "target cloud v2", v2);
  viewer.initCameraParameters ();
  setViewerPose (viewer, scene_sensor_pose);
  
  // Show keypoints in 3D viewer
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithScale> src_keypoints_color_handler (src_keypoints_ptr, 255, 0, 0);
  viewer.addPointCloud<pcl::PointWithScale> (src_keypoints_ptr, src_keypoints_color_handler, "source keypoints", v1);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "source keypoints");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithScale> tar_keypoints_color_handler (tar_keypoints_ptr, 0, 0, 255);
  viewer.addPointCloud<pcl::PointWithScale> (tar_keypoints_ptr, tar_keypoints_color_handler, "target keypoints", v1);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "target keypoints");

  // Add transformed point cloud to viewer
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tf_cloud_color_handler (transformed_cloud_ptr, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ> (transformed_cloud_ptr, tf_cloud_color_handler, "initial aligned cloud", v2);
  
  return nullptr;
} 

int main (int argc, char** argv)
{
  // load point clouds
  pcl::io::loadPCDFile<pcl::PointXYZ> ("pointClouds/test_pcd1.pcd", *cloudA);
  pcl::io::loadPCDFile<pcl::PointXYZ> ("pointClouds/test_pcd2.pcd", *cloudB);

  //down sample the points because 800,000 per cloud is too much
  std::cerr << "PointCloud before filtering: " << cloudA->width * cloudA->height 
       << " data points (" << pcl::getFieldsList (*cloudA) << ")." << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB_downsampled (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::VoxelGrid<pcl::PointXYZ> sorA;
  sorA.setInputCloud (cloudA);
  sorA.setLeafSize (0.005f, 0.005f, 0.005f); //5mm sized voxels
  sorA.filter (*cloudA_downsampled);

  pcl::VoxelGrid<pcl::PointXYZ> sorB;
  sorB.setInputCloud (cloudB);
  sorB.setLeafSize (0.005f, 0.005f, 0.005f); //5mm sized voxels
  sorB.filter (*cloudB_downsampled);
  std::cerr << "PointCloud after filtering: " << cloudA_downsampled->width * cloudA_downsampled->height 
    << " data points (" << pcl::getFieldsList (*cloudA_downsampled) << ")." << std::endl;


  // Initialize ROS
  ros::init (argc, argv, "mmm_pointcloud_registration");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("mmm_full_scan", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud_output", 1);
  registerClouds(cloudA_downsampled,cloudB_downsampled);
  // Spin
  ros::spin ();
}