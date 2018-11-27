#ifndef IPA_AKIT_DETECTION_H
#define IPA_AKIT_DETECTION_H

#include "ipa_akit_detection_commons.h"
#include <qt5/QtCore/qmutex.h>
#include <qt5/QtCore/qtimer.h>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <pcl/search/kdtree.h>

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

class ipa_akit_detection
{
public:
  ros::NodeHandle node_handle_;
  ros::NodeHandle np_;
  ipa_akit_detection(ros::NodeHandle nh);
  ~ipa_akit_detection();
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input);
  void spin();

protected:

  string topic_name;
  ros::Subscriber sub_;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;
  CloudPtr cloud;
  //locking
  std::mutex m_;
  condition_variable cv;
  thread visualisation_;


  QMutex mtx_;
  QMutex vis_mtx_;


  bool capture_;
  bool data_modified_;
  size_t previous_data_size_;
  size_t previous_clusters_size_;

  bool display_normals_;
  bool display_curvature_;
  bool display_distance_map_;

  bool use_planar_refinement_;
  bool use_clustering_;

  pcl::PointCloud<PointT> prev_cloud_;
  pcl::PointCloud<pcl::Normal> prev_normals_;
  std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > prev_regions_;

  void run();
  void removePreviousDataFromScreen (size_t prev_models_size, size_t prev_clusters_size, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);


  CloudPtr received_cloud_ptr;
  CloudPtr input_pointcloud;

  ros::Subscriber input_pointcloud_sub_;	///< incoming point cloud topic
  ros::Publisher output_pointcloud_pub_;	///< pointcloud with one segmented object
  ros::Publisher output_plane_pub_;
  sensor_msgs::PointCloud2 output_pointcloud_segments_msg;



    ///< ROS node handle

    // parameters
 double target_publishing_rate_;		///< rate at which the input messages are published (in Hz)
 ros::Time last_publishing_time_;	///< time of the last publishing activity


};

#endif // IPA_AKIT_DETECTION_H
