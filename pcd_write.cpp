#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>


typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

pcl::PointCloud<PointType>::Ptr received_cloud_ptr, cloud_filtered_ptr;
CloudPtr cloud_pass_;


void roscloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  received_cloud_ptr.reset(new pcl::PointCloud<PointType>);
  cloud_filtered_ptr.reset(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg (*input.get(), *received_cloud_ptr.get());

  pcl::PassThrough<PointType> pass;
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 10.0);
  pass.setKeepOrganized (false);
  pass.setInputCloud (received_cloud_ptr);
  pass.filter (*cloud_filtered_ptr);

  pcl::io::savePCDFileASCII ("/home/pfs-mt/test_pcd1.pcd", *cloud_filtered_ptr);
  std::cerr << "Saved " << cloud_filtered_ptr->points.size () << " data points to test_pcd.pcd." << std::endl;
}

int
main (int argc, char** argv)
{


  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, roscloud_cb);



  ros::spin();
  return (0);
}

