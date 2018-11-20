#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data

 // std::string filename = "/home/pfs-mt/Documents/scanned_every_30_deg/27.pcd";
  std::string filename = "/home/pfs-mt/Documents/pcd_models/blumen_topf_scan/cloud.pcd";
  std::string destination = "/home/pfs-mt/Documents/pcd_models/scene.pcd";


    if(pcl::io::loadPCDFile(filename, *cloud) == -1){
      std::cout << "pcd file not found" << std::endl;
      exit(-1);
    }


  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.02, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);



  pcl::io::savePCDFile(destination, *cloud_filtered);



  return (0);

}
