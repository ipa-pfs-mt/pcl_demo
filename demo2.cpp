
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <boost/format.hpp>
//#include <pcl/filters/uniform_sampling.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/keypoints/uniform_sampling.h>



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

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/common/centroid.h>
#include <pcl/features/crh.h>
#include <pcl/recognition/crh_alignment.h>
#include <pcl/segmentation/extract_clusters.h>


ros::Publisher pub;

using namespace pcl::tracking;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointXYZRGB RefPointType;
typedef ParticleXYZRPY ParticleT;

typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;


typedef pcl::Normal NormalType;
typedef pcl::PointXYZRGB XYZ;
typedef pcl::PointCloud<XYZ> CloudXYZ;
typedef CloudXYZ::Ptr CloudXYZPtr;
typedef CloudXYZ::ConstPtr CloudXYZConstPtr;



typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

int counter1 = 0;

sensor_msgs::PointCloud2 kp_scene_msg, kp_object_msg, cloud_pass_msg, object_msg, down_cloud_msg;
ros::Publisher pub_kp_scene, pub_kp_object, pub_cloud_pass, pub_object, pub_down_cloud;

pcl::PointCloud<PointType>::Ptr received_cloud_ptr (new pcl::PointCloud<PointType>);


CloudPtr cloud_pass_ (new Cloud);
CloudPtr cloud_pass_downsampled_  (new Cloud);
CloudPtr target_cloud_ (new Cloud);

pcl::PointCloud<XYZ>::Ptr keypoints_scene (new pcl::PointCloud<XYZ>);
pcl::PointCloud<XYZ>::Ptr keypoints_object (new pcl::PointCloud<XYZ>);

pcl::PointCloud<pcl::Normal>::Ptr normals_scene (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::SHOT352>::Ptr descriptors_scene (new pcl::PointCloud<pcl::SHOT352>());

pcl::PointCloud<pcl::Normal>::Ptr normals_object (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::SHOT352>::Ptr descriptors_object (new pcl::PointCloud<pcl::SHOT352>());

std::vector<pcl::Correspondences> clusteredCorrespondences;
std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;

CloudPtr plane (new Cloud);
CloudPtr convex_hull (new Cloud);
CloudPtr objects (new Cloud);

boost::mutex mtx_;

bool new_cloud_;
double downsampling_grid_size_ = 0.0001;
int counter;
double x,y,z;
double quaternion_x,quaternion_y,quaternion_z,quaternion_w;

//Filter along a specified dimension
void filterPassThrough (const CloudConstPtr &cloud, CloudPtr &result)
{
  pcl::PassThrough<PointType> pass;
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.5);
  pass.setKeepOrganized (true);
  pass.setInputCloud (cloud);
  pass.filter (*result);
}


void gridSampleApprox (const CloudConstPtr &cloud, CloudPtr &result, double leaf_size)
{
  pcl::ApproximateVoxelGrid<PointType> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (*result);
}
void computePlaneExtraction(const CloudConstPtr &cloud, CloudPtr &plane, CloudPtr &convex_hull, CloudPtr &objects){

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::SACSegmentation<XYZ> segmentation;
  segmentation.setInputCloud(cloud);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(0.01);
  segmentation.setOptimizeCoefficients(true);
  pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
  segmentation.segment(*planeIndices, *coefficients);

  if(planeIndices->indices.size() == 0)
    ROS_WARN_STREAM("Could not find a plane in the scene.");
  else
  {
    pcl::ExtractIndices<XYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(planeIndices);
    extract.filter(*plane);

    pcl::ConvexHull<XYZ> hull;
    hull.setInputCloud(plane);
    hull.setDimension(2);
    hull.reconstruct(*convex_hull);

    if(hull.getDimension()==2)
    {
      pcl::ExtractPolygonalPrismData<XYZ> prism;
      prism.setInputCloud(cloud);
      prism.setInputPlanarHull(convex_hull);
      // First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
      // Second parameter: maximum Z value, set to 10cm. Tune it according to the height of the objects you expect.
      prism.setHeightLimits(0.010f, 1.0f);
      pcl::PointIndices::Ptr object_indices (new pcl::PointIndices);

      prism.segment(*object_indices);

      extract.setIndices(object_indices);
      extract.filter(*objects);
/*
      pcl::toROSMsg(*objects.get(), objects_msg);
      objects_msg.header.frame_id = "camera";
      pub_objects.publish(objects_msg);
      */

    }
    else ROS_WARN_STREAM("The chosen hull is not planar.");



  }
}
/*
bool
drawParticles ()
{
  ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
  if (particles && new_cloud_)
    {
      //Set pointCloud with particle's points
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
      for (size_t i = 0; i < particles->points.size (); i++)
  {
    pcl::PointXYZRGB point(255,0,0);

    point.x = particles->points[i].x;
    point.y = particles->points[i].y;
    point.z = particles->points[i].z;
    ROS_WARN_STREAM("point: x: "<<point.x<<" y: "<<point.y<< " z: "<<point.z);
    particle_cloud->points.push_back (point);
  }

      //Draw red particles
      {
        pcl::toROSMsg(*particle_cloud.get(), particle_cloud_msg );
                   // particle_cloud_msg.header.frame_id = "/camera_color_optical_frame";
                    particle_cloud_msg.header.frame_id = "camera";

                    pub_praticles_cloud.publish(particle_cloud_msg );
      }
      return true;
    }
  else
    {
      return false;
    }
}
*/
void publishObjectPose(Eigen::Affine3f &transformation)
{
    Eigen::Matrix3f rotationMatrix=transformation.rotation();

    x=transformation.translation().x();
    y=transformation.translation().y();
    z=transformation.translation().z();

    Eigen::Quaternionf quaternion(rotationMatrix);
    quaternion_x=quaternion.x();
    quaternion_y=quaternion.y();
    quaternion_z=quaternion.z();
    quaternion_w=quaternion.w();


}
/*
template <typename PointT>
void computeMomentOfInertia(boost::shared_ptr<pcl::PointCloud<PointT> > cloud_ptr)
{

    pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
    feature_extractor.setInputCloud (cloud_ptr);
    feature_extractor.compute ();

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    pcl::PointXYZRGB min_point_AABB;
    pcl::PointXYZRGB max_point_AABB;
    pcl::PointXYZRGB min_point_OBB;
    pcl::PointXYZRGB max_point_OBB;
    pcl::PointXYZRGB position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;


    feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);

    x=mass_center(0,0);
    y=mass_center(1,0);
    z=mass_center(2,0);


}
*/
/*
void viz_cb (boost::shared_ptr<pcl::visualization::PCLVisualizer> & viz)
{
  boost::mutex::scoped_lock lock (mtx_);
  if(!cloud_pass_)
  {
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    return;
  }
  if(new_cloud_ && cloud_pass_downsampled_)
  {
    CloudPtr cloud_pass;
    cloud_pass = cloud_pass_downsampled_;

    if (!viz->updatePointCloud (cloud_pass, "cloudpass"))
    {
      viz->addPointCloud (cloud_pass, "cloudpass");
      viz->resetCameraViewpoint("cloudpass");
    }
    /*
   bool ret = drawParticles (viz);
     if (ret)
     drawResult (viz);
        }



  }
  new_cloud_ = false;

}
  */
double
computeCloudResolution(const pcl::PointCloud<XYZ>::ConstPtr &cloud)
{
  double resolution = 0.0;
  int numberOfPoints = 0;
  int nres;
  std::vector<int> indices(2);
  std::vector<float> squaredDistances(2);
  pcl::search::KdTree<XYZ> tree;
  tree.setInputCloud(cloud);

  for (size_t i = 0; i < cloud->size(); ++i)
  {
    if (! pcl_isfinite((*cloud)[i].x))
      continue;

    // Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
    if (nres == 2)
    {
      resolution += sqrt(squaredDistances[1]);
      ++numberOfPoints;
    }
  }
  if (numberOfPoints != 0)
    resolution /= numberOfPoints;

  return resolution;
}

void computeISSKeypoint3D(const CloudConstPtr &cloud, CloudXYZPtr &keypoints_cloud)
{

pcl::ISSKeypoint3D<XYZ, XYZ> detector;
detector.setInputCloud(cloud);
pcl::search::KdTree<XYZ>::Ptr kdtree1 (new pcl::search::KdTree<XYZ>);
detector.setSearchMethod(kdtree1);
double resolution = computeCloudResolution(cloud);
ROS_WARN_STREAM("Resolution " << resolution );

// Set the radius of the spherical neighborhood used to compute the scatter matrix.
  detector.setSalientRadius(6 * resolution);
  // Set the radius for the application of the non maxima supression algorithm.
  detector.setNonMaxRadius(4 * resolution);
  // Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
  detector.setMinNeighbors(5);
  // Set the upper bound on the ratio between the second and the first eigenvalue.
  detector.setThreshold21(0.975);
  // Set the upper bound on the ratio between the third and the second eigenvalue.
  detector.setThreshold32(0.975);
  // Set the number of prpcessing threads to use. 0 sets it to automatic.
  detector.setNumberOfThreads(4);

  detector.compute(*keypoints_cloud);

}
/*
void computeUniformSampling(const CloudConstPtr &cloud, CloudXYZPtr &keypoints_cloud, float model_ss)
{
  uniform_sampling.setInputCloud (cloud);
  uniform_sampling.setRadiusSearch (model_ss);
  uniform_sampling.filter (*model_keypoints);
}
*/
void computeSHOTDescriptors(const CloudXYZConstPtr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals,
                            pcl::PointCloud<pcl::SHOT352>::Ptr &descriptors)
{
    pcl::NormalEstimation<XYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    normalEstimation.setRadiusSearch(0.03);
    pcl::search::KdTree<XYZ>::Ptr kdtree(new pcl::search::KdTree<XYZ>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);

    pcl::SHOTEstimationOMP<XYZ, pcl::Normal, pcl::SHOT352> shot;
    shot.setInputCloud(cloud);
    shot.setInputNormals(normals);
    shot.setRadiusSearch(0.03);
    shot.compute(*descriptors);
 }



typedef pcl::SHOT352 DescriptorsT;
void computeCorrespondences (const pcl::PointCloud<DescriptorsT>::Ptr &scene_descriptors,
                             const pcl::PointCloud<DescriptorsT>::Ptr &object_descriptors,
                             pcl::CorrespondencesPtr &correspondences)
{
  pcl::KdTreeFLANN<DescriptorsT> matching;
  matching.setInputCloud(object_descriptors);

  for (size_t i=0; i<scene_descriptors->size();++i)
  {
    std::vector<int> neighbors(1);
    std::vector<float> squaredDistances(1);

    if(pcl_isfinite(scene_descriptors->at(i).descriptor[0]))
    {
      int neighborCount = matching.nearestKSearch(scene_descriptors->at(i), 1, neighbors, squaredDistances);
      if (neighborCount == 1 && squaredDistances[0] < 0.25f)
      {
        pcl::Correspondence correspondence(neighbors[0], static_cast<int>(i), squaredDistances[0]);
        correspondences->push_back(correspondence);
      }

    }
  }
  ROS_WARN_STREAM("Found " << correspondences->size() << " correspondences." );

}
void computeCorrespondenceGrouping (const pcl::PointCloud<XYZ>::Ptr &keypoints_scene,
                             const pcl::PointCloud<XYZ>::Ptr &keypoints_object,
                             pcl::CorrespondencesPtr &correspondences,
                             std::vector<pcl::Correspondences> &clusteredCorrespondences,
                             std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transformations)
{
  pcl::GeometricConsistencyGrouping<XYZ, XYZ> grouping;
  grouping.setSceneCloud(keypoints_scene);
  grouping.setInputCloud(keypoints_object);
  grouping.setModelSceneCorrespondences(correspondences);
  grouping.setGCThreshold(5);
  grouping.setGCSize(0.01);
  grouping.recognize(transformations, clusteredCorrespondences);

  ROS_WARN_STREAM("Model instances found: " << transformations.size() << std::endl);

}


void cloud_cb (const CloudConstPtr &cloud)
{



  boost::mutex::scoped_lock lock (mtx_);
  cloud_pass_.reset (new Cloud);
  cloud_pass_downsampled_.reset (new Cloud);
 filterPassThrough (cloud, cloud_pass_);

 if(counter1 == 6)
   pcl::io::savePCDFile("/home/pfs-mt/Documents/pcd_models/scene.pcd", *cloud_pass_);

 counter1++;
  //gridSampleApprox (cloud_pass_, cloud_pass_downsampled_, downsampling_grid_size_);
  //computePlaneExtraction(cloud_pass_downsampled_, plane, convex_hull, objects);
 /* if( !viewer->updatePointCloud( cloud, "cloud" ) ){
  viewer->addPointCloud( cloud, "cloud" );}
  viewer->spinOnce(1);
*/

  //pcl::toROSMsg(*cloud_pass_, down_cloud_msg);
   //cloud_pass_downsampled_.header.frame_id = "world";
   //pub_down_cloud.publish(down_cloud_msg );
   /*
   ROS_WARN_STREAM(cloud_pass_->size());

   computeISSKeypoint3D(objects, keypoints_scene);
   ROS_WARN_STREAM("Scene Keypoint");
   computeISSKeypoint3D(target_cloud_, keypoints_object);
   ROS_WARN_STREAM("Object Keypoint");

*/
  pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
    pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
    pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

  pcl::UniformSampling<XYZ> uniform_sampling;
  uniform_sampling.setInputCloud(target_cloud_);
  uniform_sampling.setRadiusSearch(0.01f);

  pcl::PointCloud<int> keypointIndices1;
  uniform_sampling.compute(keypointIndices1);
  pcl::copyPointCloud(*target_cloud_, keypointIndices1.points, *model_keypoints);

  ROS_WARN_STREAM("Model total points: " << target_cloud_->size () << "; Selected Keypoints: " << keypoints_object->size ());

  uniform_sampling.setInputCloud(cloud_pass_);
  uniform_sampling.setRadiusSearch (0.03f);
  //uniform_sampling.filter (*scene_keypoints);

  pcl::PointCloud<int> keypointIndices2;
    uniform_sampling.compute(keypointIndices2);
    pcl::copyPointCloud(*cloud_pass_, keypointIndices2.points, *scene_keypoints);

  ROS_WARN_STREAM("Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size ());

  pcl::toROSMsg(*scene_keypoints.get(), kp_scene_msg);
  kp_scene_msg.header.frame_id = "camera";
  pub_kp_scene.publish(kp_scene_msg);

  pcl::toROSMsg(*model_keypoints.get(), kp_object_msg);
  kp_object_msg.header.frame_id = "camera";
  pub_kp_object.publish(kp_object_msg);



      //##################### Descriptors

  computeSHOTDescriptors(scene_keypoints, normals_scene, descriptors_scene);
  computeSHOTDescriptors(model_keypoints, normals_object, descriptors_object);

        //################## Matching

  pcl::CorrespondencesPtr correspondences (new pcl::Correspondences());
  computeCorrespondences(descriptors_scene, descriptors_object, correspondences);

        //################ Correspondence Grouping

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;

  
  
  computeCorrespondenceGrouping(scene_keypoints, model_keypoints, correspondences, clusteredCorrespondences,
                                     transformations);
        for(size_t i = 0; i<transformations.size(); i++)
        {
          ROS_WARN_STREAM("Instance " << (i + 1) << ":");
          ROS_WARN_STREAM("\tHas " << clusteredCorrespondences[i].size() << " correspondences." << std::endl);
          Eigen::Matrix3f rotation = transformations[i].block<3, 3>(0, 0);
          Eigen::Vector3f translation = transformations[i].block<3, 1>(0, 3);

          ROS_WARN("\n");
          ROS_WARN("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
          ROS_WARN("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
          ROS_WARN("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
          ROS_WARN("\n");
          ROS_WARN("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));


        }



new_cloud_=true;

}
void roscloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  received_cloud_ptr.reset(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg (*input, *received_cloud_ptr);
  cloud_cb (received_cloud_ptr);
}

void parseCommandLine (int argc, char *argv[])
{


}

int main (int argc, char** argv) {

  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;





  ros::Subscriber sub = nh.subscribe ("/stereo/points2", 1, roscloud_cb);
  //ros::Subscriber sub = nh.subscribe ("camera/depth_registered/points", 1, roscloud_cb);
  
  pub_down_cloud=nh.advertise<sensor_msgs::PointCloud2>("cloud_pass_downsampled_",1);
  pub_kp_scene=nh.advertise<sensor_msgs::PointCloud2>("kp_scene",1);
  pub_kp_object=nh.advertise<sensor_msgs::PointCloud2>("kp_object",1);





  //target_cloud_.reset(new Cloud());

  pcl::PointCloud<pcl::PointXYZ> target_cloud_XYZ;
 // std::string filename = "/home/pfs-mt/Documents/scanned_every_30_deg/27.pcd";
  std::string filename = "/home/pfs-mt/Documents/pcd_models/volleyball/polygon_mesh.ply_output/41.pcd";



    if(pcl::io::loadPCDFile(filename, target_cloud_XYZ) == -1){
      std::cout << "pcd file not found" << std::endl;
      exit(-1);
    }
    pcl::copyPointCloud(target_cloud_XYZ, *target_cloud_);


  counter = 0;



      ros::Rate loop_rate(40);
     while(ros::ok())
     { 
         ros::spinOnce();
         loop_rate.sleep();
      }


      return 0;
}

