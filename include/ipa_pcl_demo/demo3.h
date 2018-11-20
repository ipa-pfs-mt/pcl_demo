#ifndef DEMO3_H
#define DEMO3_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <boost/format.hpp>

#include <stdio.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <flann/flann.h>

#include <boost/filesystem.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/geometric_consistency.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/our_cvfh.h>

#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/cloud_viewer.h>

//Segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/common/centroid.h>
#include <pcl/features/crh.h>
#include <pcl/recognition/crh_alignment.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointXYZRGB RefPointType;
typedef pcl::PointXYZRGB XYZ;
typedef pcl::PointCloud<XYZ> CloudXYZ;
typedef CloudXYZ::Ptr CloudXYZPtr;
typedef CloudXYZ::ConstPtr CloudXYZConstPtr;
typedef pcl::PointCloud<pcl::VFHSignature308> VFHSig;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef pcl::Histogram<90> CRH90;
typedef std::pair<std::string, std::vector<float> > vfh_model;

class ObjectDetection
{
  ros::NodeHandle node_handle_;
  ros::Publisher pub;
  sensor_msgs::PointCloud2 kp_scene_msg, kp_object_msg, cloud_pass_msg, object_msg, down_cloud_msg, objects_msg, plane_msg, convex_hull_msg;
  ros::Publisher pub_kp_scene, pub_kp_object, pub_cloud_pass, pub_object, pub_down_cloud, pub_objects, pub_plane, pub_convex_hull;

  public:

  ObjectDetection(ros::NodeHandle nh);
  ~ObjectDetection();

  protected:

  void filterPassThrough (const CloudConstPtr &cloud, CloudPtr &result);
  void gridSampleApprox (const CloudConstPtr &cloud, CloudPtr &result, double leaf_size);
  void publishObjectPose(Eigen::Affine3f &transformation);
  void viz_cb (pcl::visualization::PCLVisualizer& viz);
  double computeCloudResolution(const pcl::PointCloud<XYZ>::ConstPtr &cloud);
  void computeISSKeypoint3D(const CloudConstPtr &cloud, CloudXYZPtr &keypoints_cloud);
  void computeSHOTDescriptors(const CloudXYZConstPtr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals,
                              pcl::PointCloud<pcl::SHOT352>::Ptr &descriptors);
  typedef pcl::SHOT352 DescriptorsT;
  void computeCorrespondences (const pcl::PointCloud<DescriptorsT>::Ptr &scene_descriptors,
                               const pcl::PointCloud<DescriptorsT>::Ptr &object_descriptors,
                               pcl::CorrespondencesPtr &correspondences);
  void computeCorrespondenceGrouping (const pcl::PointCloud<XYZ>::Ptr &keypoints_scene,
                               const pcl::PointCloud<XYZ>::Ptr &keypoints_object,
                               pcl::CorrespondencesPtr &correspondences,
                              std::vector<pcl::Correspondences> &clusteredCorrespondences,
                               std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transformations);
  void publishPointCloud (const CloudConstPtr &cloud, sensor_msgs::PointCloud2 msgs, ros::Publisher pub);
  void displayClusters(const CloudConstPtr &cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer);
  void nearestKSearch(flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model,
                      int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances);
  void computePlaneExtraction(const CloudConstPtr &cloud, CloudPtr &plane, CloudPtr &convex_hull, CloudPtr &objects);
  void computeEuclideanSegmentation(const CloudConstPtr &cloud,
                                    boost::shared_ptr<std::vector<pcl::PointIndices> > &clusters);
  void computeOURCVFHDescriptor(const CloudConstPtr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals,
                           pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptors);
  void computeCVFHDescriptor(const CloudConstPtr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals,
                           pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptors);
  void computeCRH(const CloudConstPtr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals,
             pcl::PointCloud<CRH90>::Ptr &histogram, boost::shared_ptr<Eigen::Vector4f> &centroid);
  void computeCentroidsAlignment(const CloudConstPtr &scene, const CloudConstPtr &object,
                            pcl::PointCloud<CRH90>::Ptr &histogram_scene, pcl::PointCloud<CRH90>::Ptr &histogram_object,
                            Eigen::Vector4f &scene_centroid, Eigen::Vector4f &object_centroid);
  bool loadHist (const std::string filename, const pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptors,
            vfh_model &vfh);
  void loadModels (const boost::filesystem::path &base_dir, const std::string &extension,
              std::vector<vfh_model> &models,
              std::vector<pcl::PointCloud<XYZ>::Ptr > &model_partial_views,
              std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr > &model_partial_views_descriptors,
              std::vector<pcl::PointCloud<CRH90>::Ptr > &model_partial_views_histograms,
              std::vector<boost::shared_ptr<Eigen::Vector4f> > &model_partial_views_centroids);
  bool loadHisto (const boost::filesystem::path &path, vfh_model &vfh);

  void cloud_cb (const CloudConstPtr &cloud);
  void roscloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);
  void loadModelDataBase(flann::Matrix<float> *data);
  void loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension,
                     std::vector<vfh_model> &models);
  std::vector<pcl::PointCloud<XYZ>::Ptr > model_partial_views;
  std::vector<pcl::PointCloud<pcl::Normal>::Ptr > model_partial_views_normals;
  std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr > model_partial_views_descriptors;
  std::vector<pcl::PointCloud<CRH90>::Ptr > model_partial_views_histograms;
  std::vector<boost::shared_ptr<Eigen::Vector4f> > model_partial_views_centroids;
  std::vector<vfh_model> models;
  boost::mutex mtx_;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  bool new_cloud_;
  double downsampling_grid_size_ = 0.001;
  int counter;
  double x,y,z;
  double quaternion_x,quaternion_y,quaternion_z,quaternion_w;

  std::vector<pcl::Correspondences> clusteredCorrespondences;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;

  pcl::PointCloud<PointType>::Ptr received_cloud_ptr;

  CloudPtr cloud_pass_;
  CloudPtr cloud_pass_downsampled_;
  CloudPtr target_cloud_;

  pcl::PointCloud<XYZ>::Ptr keypoints_scene;
  pcl::PointCloud<XYZ>::Ptr keypoints_object;

  pcl::PointCloud<pcl::Normal>::Ptr normals_scene;
  pcl::PointCloud<pcl::SHOT352>::Ptr descriptors_scene;

  pcl::PointCloud<pcl::Normal>::Ptr normals_object;
  pcl::PointCloud<pcl::SHOT352>::Ptr descriptors_object;
  CloudPtr plane;
  CloudPtr convex_hull;
  CloudPtr objects;
  pcl::PointCloud<pcl::Normal>::Ptr normals;
  pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors;
  pcl::PointCloud<CRH90>::Ptr histogram_object;
  pcl::PointCloud<CRH90>::Ptr histogram_scene;
  ros::Subscriber sub_;

};























#endif // DEMO3_H
