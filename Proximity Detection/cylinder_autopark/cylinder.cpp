#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <boost/foreach.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


typedef pcl::PointXYZ PointT;


ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	
	//All the objects needed
	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ()); //Input PCL
	pcl_conversions::toPCL(*input, *cloud); // Create a container for the data.
	
	
	pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ()); //filtered cloud 
																		//after VoxelGrid Filtering
	pcl::PCLPointCloud2::Ptr cloud_filtered2 (new pcl::PCLPointCloud2()); //filtered cloud 
																		//after passthrough
	pcl::PCLPointCloud2::Ptr cloud_filtered3 (new pcl::PCLPointCloud2);
	
	//pcl::PCLPointCloud2::Ptr cloud_filtered4 (new pcl::PCLPointCloud2);
	
	
	

  // Voxel Grid Downsampling
  
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.02f, 0.02f, 0.02f);
  sor.filter (*cloud_filtered);
  
  //Passthrough Filtering
 pcl::PassThrough< pcl::PCLPointCloud2 > pass;
 pass.setInputCloud(cloud_filtered);
 pass.setFilterFieldName ("z");
 pass.setFilterLimits (0, 1.0);
 //pass.setFilterLimitsNegative (true);
 pass.filter(*cloud_filtered2);
 
 
  // Datasets

  pcl::PointCloud2<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud2<pcl::Normal>); //normals
  pcl::PointCloud2<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>); // for plane and cylinder
  
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

//Objects Needed
 
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
	pcl::ExtractIndices<PointT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
 
 // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered2);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);
 
 // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals);
  
 // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  
  
   // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);
  
  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered3);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);
  
  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.1);
  seg.setInputCloud (cloud_filtered3);
  seg.setInputNormals (cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  
  
   // Extract Cylinder Inliers
  extract.setInputCloud (cloud_filtered3);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  
  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  
  pcl::PointCloud2<PointT>::Ptr cloud_cylinder (new pcl::PointCloud2<PointT> ());
  extract.filter (*cloud_cylinder);
  
  if (cloud_cylinder->points.empty ()) 
    {}
  else
  {
  }
  
  //Output the final segmented cylinder
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(*cloud_cylinder, output);
  
  
  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
