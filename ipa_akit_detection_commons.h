#ifndef IPA_AKIT_DETECTION_COMMONS_H
#define IPA_AKIT_DETECTION_COMMONS_H

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

//#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>


#include <boost/filesystem.hpp>

#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/geometric_consistency.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/registration/icp.h>


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
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>


#include <pcl/console/parse.h>
#include <pcl/console/print.h>

#endif // IPA_AKIT_DETECTION_COMMONS_H
