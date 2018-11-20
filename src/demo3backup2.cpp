#include "ipa_pcl_demo/demo3.h"

int main (int argc, char** argv) {

  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  ObjectDetection od(nh);
  /*
  if (model_partial_views.empty())
  {

  loadModels("/home/pfs-mt/Documents/scanned_every_30_deg", extension, model, model_partial_views,
             model_partial_views_descriptors, model_partial_views_histograms, model_partial_views_centroids);

   ROS_WARN_STREAM("Loaded " << static_cast<int>(model_partial_views.size())<< " models" ) ;
   ROS_WARN_STREAM("Computed Descriptors for " << static_cast<int>(model_partial_views_descriptors.size())<< " models" ) ;
   ROS_WARN_STREAM("Computed CRH for " << static_cast<int>(model_partial_views_histograms.size())<< " models" ) ;
  }
  */
  //pub_kp_scene=nh.advertise<sensor_msgs::PointCloud2>("kp_scene",1);
  //pub_kp_object=nh.advertise<sensor_msgs::PointCloud2>("kp_object",1);

  //target_cloud_.reset(new Cloud());

 /* pcl::PointCloud<pcl::PointXYZ> target_cloud_XYZ;
    if(pcl::io::loadPCDFile("/home/pfs-mt/cloud.pcd", target_cloud_XYZ) == -1){
      std::cout << "pcd file not found" << std::endl;
      exit(-1);
    }
    pcl::copyPointCloud(target_cloud_XYZ, *target_cloud_);
*/
    ros::Rate loop_rate(40);
     while(ros::ok())
     {
         ros::spinOnce();
         loop_rate.sleep();
      }


      return 0;
}

