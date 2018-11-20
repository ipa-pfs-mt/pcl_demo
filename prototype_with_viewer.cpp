#include <ros/ros.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <boost/format.hpp>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
#include <iostream>
#include <fstream>

#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/sample_consensus_prerejective.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>


#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

class Prototype
{
private:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

public:
  Prototype()
  {
    sub_ = nh.subscribe ("/stereo/points2", 1, &ObjectDetection::roscloud_cb, this);
  };
  ~Prototype();



  void Prototype::roscloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
  {
    ROS_WARN("roscloud_cb");
    received_cloud_ptr.reset(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg (*input, *received_cloud_ptr);
    cloud_cb (received_cloud_ptr);
  }


};

int main(int argc, char** argv)
{


  return 0;
}
