
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

#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <flann/io/hdf5.h>


ros::Publisher pub;

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
//typedef std::pair<std::string, std::vector<float> > cvfh_model;

typedef std::pair<std::string, std::vector<float> > vfh_model;


sensor_msgs::PointCloud2 kp_scene_msg, kp_object_msg, cloud_pass_msg, object_msg, down_cloud_msg, objects_msg, plane_msg,
convex_hull_msg, choosen_msg, final_msg;
ros::Publisher pub_kp_scene, pub_kp_object, pub_cloud_pass, pub_object, pub_down_cloud, pub_objects, pub_plane,
pub_convex_hull, pub_choosen, pub_final;

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
pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors(new pcl::PointCloud<pcl::VFHSignature308>);


pcl::PointCloud<CRH90>::Ptr histogram_object (new pcl::PointCloud<CRH90>);
pcl::PointCloud<CRH90>::Ptr histogram_scene (new pcl::PointCloud<CRH90>);

//Model partial views data base for model training
std::string extension (".pcd");
std::vector<pcl::PointCloud<XYZ>::Ptr > model_partial_views;
std::vector<pcl::PointCloud<pcl::Normal>::Ptr > model_partial_views_normals;
std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr > model_partial_views_descriptors;
std::vector<pcl::PointCloud<CRH90>::Ptr > model_partial_views_histograms;
std::vector<boost::shared_ptr<Eigen::Vector4f> > model_partial_views_centroids;
std::vector<vfh_model> models;
flann::Matrix<float> data;


boost::mutex mtx_;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;


bool new_cloud_;
double downsampling_grid_size_ = 0.001;
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


void viz_cb (pcl::visualization::PCLVisualizer& viz)
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

    if (!viz.updatePointCloud (cloud_pass, "cloudpass"))
    {viz.addPointCloud (cloud_pass, "cloudpass");
      viz.resetCameraViewpoint("cloudpass");
    }
  }

}


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
    shot.setRadiusSearch(0.02);
    shot.compute(*descriptors);
 }

typedef pcl::SHOT352 DescriptorsT;
void computeCorrespondences (const pcl::PointCloud<DescriptorsT>::Ptr &scene_descriptors,
                             const pcl::PointCloud<DescriptorsT>::Ptr &object_descriptors,
                             pcl::CorrespondencesPtr &correspondences)
{
  pcl::KdTreeFLANN<DescriptorsT> matching;
  matching.setInputCloud(object_descriptors);

  for (size_t i=0; i<scene_descriptors->size();i++)
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
  grouping.setGCThreshold(3);
  grouping.setGCSize(0.01);
  grouping.recognize(transformations, clusteredCorrespondences);

  ROS_WARN_STREAM("Model instances found: " << transformations.size() << std::endl);

}
void publishPointCloud (const CloudConstPtr &cloud, sensor_msgs::PointCloud2 msgs, ros::Publisher pub)
{
  pcl::toROSMsg(*cloud.get(), msgs);
  msgs.header.frame_id = "camera_link";
  //msgs.header.frame_id = "camera_color_frame";
  pub.publish(msgs);
}
void displayClusters(const CloudConstPtr &cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer)
{

}
void nearestKSearch(flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model,
                    int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
  flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size()], 1, model.second.size());
  memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

    indices = flann::Matrix<int>(new int[k], 1, k);
    distances = flann::Matrix<float>(new float[k], 1, k);
    index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
    delete[] p.ptr ();


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
      prism.setHeightLimits(0.018f, 0.3f);
      pcl::PointIndices::Ptr object_indices (new pcl::PointIndices);

      prism.segment(*object_indices);

      extract.setIndices(object_indices);
      extract.filter(*objects);
/*
      pcl::toROSMsg(*objects.get(), objects_msg);
      objects_msg.header.frame_id = "camera";
      pub_objects.publish(objects_msg);
      */
      publishPointCloud(objects, objects_msg, pub_objects);
    }
    else ROS_WARN_STREAM("The chosen hull is not planar.");
  publishPointCloud(convex_hull, convex_hull_msg, pub_convex_hull);
  publishPointCloud(plane, plane_msg, pub_plane);

  }
}
void computeEuclideanSegmentation(const CloudConstPtr &cloud,
                                  boost::shared_ptr<std::vector<pcl::PointIndices> > &clusters)
{
  pcl::search::KdTree<XYZ>::Ptr kdtree(new pcl::search::KdTree<XYZ>);
  kdtree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<XYZ> clustering;
  // Set cluster tolerance to 2cm (small values may cause objects to be divided
  // in several clusters, whereas big values may join objects in a same cluster).
  clustering.setClusterTolerance(0.02);
  // Set the minimum and maximum number of points that a cluster can have.
  clustering.setMinClusterSize(100);
  clustering.setMaxClusterSize(25000);
  clustering.setSearchMethod(kdtree);
  clustering.setInputCloud(cloud);
  clustering.extract(*clusters);

}

void computeOURCVFHDescriptor(const CloudConstPtr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals,
                         pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptors)
{
  pcl::NormalEstimation<XYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(cloud);
  normalEstimation.setRadiusSearch(0.03);
  pcl::search::KdTree<XYZ>::Ptr kdtree(new pcl::search::KdTree<XYZ>);
  normalEstimation.setSearchMethod(kdtree);
  normalEstimation.compute(*normals);

  pcl::OURCVFHEstimation<XYZ, pcl::Normal, pcl::VFHSignature308> ourcvfh;
  ourcvfh.setInputCloud(cloud);
  ourcvfh.setInputNormals(normals);
  ourcvfh.setSearchMethod(kdtree);
  ourcvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
  ourcvfh.setCurvatureThreshold(1.0);
  ourcvfh.setNormalizeBins(false);
  // Set the minimum axis ratio between the SGURF axes. At the disambiguation phase,
  // this will decide if additional Reference Frames need to be created, if ambiguous.
  ourcvfh.setAxisRatio(0.8);

  ourcvfh.compute(*descriptors);
}
void computeCVFHDescriptor(const CloudConstPtr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals,
                         pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptors)
{
  pcl::NormalEstimation<XYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(cloud);
  normalEstimation.setRadiusSearch(0.03);
  pcl::search::KdTree<XYZ>::Ptr kdtree(new pcl::search::KdTree<XYZ>);
  normalEstimation.setSearchMethod(kdtree);
  normalEstimation.compute(*normals);

  pcl::CVFHEstimation<XYZ, pcl::Normal, pcl::VFHSignature308> cvfh;
  cvfh.setInputCloud(cloud);
  cvfh.setInputNormals(normals);
  cvfh.setSearchMethod(kdtree);
  cvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
  cvfh.setCurvatureThreshold(1.0);
  cvfh.setNormalizeBins(false);

  cvfh.compute(*descriptors);
}

void computeCRH(const CloudConstPtr &cloud,
           pcl::PointCloud<CRH90>::Ptr &histogram, Eigen::Vector4f &centroid)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<XYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(cloud);
  normalEstimation.setRadiusSearch(0.03);
  pcl::search::KdTree<XYZ>::Ptr kdtree(new pcl::search::KdTree<XYZ>);
  normalEstimation.setSearchMethod(kdtree);
  normalEstimation.compute(*normals);

  pcl::CRHEstimation<XYZ, pcl::Normal, pcl::Histogram<90>> crh;
  crh.setInputCloud(cloud);
  crh.setInputNormals(normals);
  pcl::compute3DCentroid(*cloud, centroid);
  crh.setCentroid(centroid);
  crh.compute(*histogram);
}
void computeCentroidsAlignment(const CloudConstPtr &scene, const CloudConstPtr &object,
                          pcl::PointCloud<CRH90>::Ptr &histogram_scene, pcl::PointCloud<CRH90>::Ptr &histogram_object,
                          Eigen::Vector4f &scene_centroid, Eigen::Vector4f &object_centroid)
{

}
bool
loadHist (const std::string filename, const pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptors,
          vfh_model &vfh)
{
  vfh.second.resize (308);

  for (size_t i = 0; i < descriptors->points.size(); i++)
  {
      vfh.second[i] = descriptors->points[0].histogram[i];
        //point.points[0].histogram[i];
  }
  vfh.first = filename;
  return (true);
}
bool loadHisto (const boost::filesystem::path &path, vfh_model &vfh)
  {
  int vfh_idx;
    // Load the file as a PCD
    try
    {
      pcl::PCLPointCloud2 cloud;
      int version;
      Eigen::Vector4f origin;
      Eigen::Quaternionf orientation;
      pcl::PCDReader r;
      int type; unsigned int idx;
      r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);

      vfh_idx = pcl::getFieldIndex (cloud, "vfh");
      if (vfh_idx == -1)
        return (false);
      if ((int)cloud.width * cloud.height != 1)
        return (false);
    }
    catch (const pcl::InvalidConversionException&)
    {
      return (false);
    }

    // Treat the VFH signature as a single Point Cloud
    pcl::PointCloud <pcl::VFHSignature308> point;
    pcl::io::loadPCDFile (path.string (), point);
    vfh.second.resize (308);

    std::vector <pcl::PCLPointField> fields;
    pcl::getFieldIndex (point, "vfh", fields);

    for (size_t i = 0; i < fields[vfh_idx].count; ++i)
    {
      vfh.second[i] = point.points[0].histogram[i];
    }
    vfh.first = path.string ();
    return (true);
  }
void
loadModels (const boost::filesystem::path &base_dir, const std::string &extension,
            std::vector<vfh_model> &models,
            std::vector<pcl::PointCloud<XYZ>::Ptr > &model_partial_views,
            std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr > &model_partial_views_descriptors,
            std::vector<pcl::PointCloud<CRH90>::Ptr > &model_partial_views_histograms,
            std::vector<boost::shared_ptr<Eigen::Vector4f> > &model_partial_views_centroids
            )
{
  if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
    return;

  for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
  {
    if (boost::filesystem::is_directory (it->status ()))
    {
      std::stringstream ss;
      ss << it->path ();
      ROS_WARN_STREAM("Loading "<< (unsigned long)models.size());

      //pcl::console::print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ());
      loadModels (it->path (), extension, models, model_partial_views, model_partial_views_descriptors, model_partial_views_histograms, model_partial_views_centroids);
    }
    if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
    {
      pcl::PointCloud<XYZ>::Ptr model (new pcl::PointCloud<XYZ>);
      pcl::PointCloud<pcl::Normal>::Ptr normals_temp (new pcl::PointCloud<pcl::Normal>);
      pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors_temp (new pcl::PointCloud<pcl::VFHSignature308>);
      pcl::PointCloud<CRH90>::Ptr histograms_temp (new pcl::PointCloud<CRH90>);
      Eigen::Vector4f centroids_temp;

       //if (loadHist (base_dir / it->path ().filename (), model))
      //ROS_WARN_STREAM(it->path().filename()  );
      std::string name = base_dir.string() + "/" + it->path().filename().string();

      ROS_WARN_STREAM(name );
      pcl::io::loadPCDFile(name, *model);
      model_partial_views.push_back (model);
      //computeCVFHDescriptor(model, normals_temp, descriptors_temp);
      computeOURCVFHDescriptor(model, normals_temp, descriptors_temp);

      vfh_model m;
      if (loadHist (it->path().filename().string(), descriptors_temp ,m))
      models.push_back (m);

      model_partial_views_descriptors.push_back(descriptors_temp);
     // computeCRH(model, histograms_temp, centroids_temp);
      model_partial_views_histograms.push_back(histograms_temp);
      //model_partial_views_centroids.push_back(centroids_temp);

      //Saving histograms
      //std::string name_save = name+"vfh.pcd";
      //pcl::io::savePCDFileASCII(name_save, *descriptors_temp);


    }
  }
}

void
loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension,
                   std::vector<vfh_model> &models)
{
  if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
    return;

  for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
  {
    if (boost::filesystem::is_directory (it->status ()))
    {
      std::stringstream ss;
      ss << it->path ();

      pcl::console::print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ());
      loadFeatureModels (it->path (), extension, models);
    }
    if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
    {
      ROS_WARN_STREAM("Loaded "<<it->path().filename());
      vfh_model m;
      if (loadHisto (base_dir / it->path ().filename (), m))
        models.push_back (m);
      ROS_WARN_STREAM("Loaded "<< (unsigned long)models.size());
    }
  }
}
bool
loadFileList (std::vector<vfh_model> &models, const std::string &filename)
{
  ifstream fs;
  fs.open (filename.c_str ());
  if (!fs.is_open () || fs.fail ())
    return (false);

  std::string line;
  while (!fs.eof ())
  {
    getline (fs, line);
    if (line.empty ())
      continue;
    vfh_model m;
    m.first = line;
    models.push_back (m);
  }
  fs.close ();
  return (true);
}

void Visualization(const CloudConstPtr &cloud)
{
  pcl::visualization::PCLVisualizer viewer("viewer");


}
void
displayEuclideanClusters (const pcl::PointCloud<PointType>::CloudVectorType &clusters,
                          boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  char name[1024];
  unsigned char red [6] = {255,   0,   0, 255, 255,   0};
  unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
  unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

  for (size_t i = 0; i < clusters.size (); i++)
  {
    sprintf (name, "cluster_%d" , int (i));
    pcl::visualization::PointCloudColorHandlerCustom<PointType> color0(boost::make_shared<pcl::PointCloud<PointType> >(clusters[i]),red[i%6],grn[i%6],blu[i%6]);
    if (!viewer->updatePointCloud (boost::make_shared<pcl::PointCloud<PointType> >(clusters[i]),color0,name))
      viewer->addPointCloud (boost::make_shared<pcl::PointCloud<PointType> >(clusters[i]),color0,name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);
  }
}
void
removePreviousDataFromScreen (size_t prev_models_size, size_t prev_clusters_size, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  char name[1024];
  for (size_t i = 0; i < prev_models_size; i++)
  {
    sprintf (name, "normal_%d", unsigned (i));
    viewer->removeShape (name);

    sprintf (name, "plane_%02d", int (i));
    viewer->removePointCloud (name);
  }

  for (size_t i = 0; i < prev_clusters_size; i++)
  {
    sprintf (name, "cluster_%d", int (i));
    viewer->removePointCloud (name);
  }
}

void cloud_cb (const CloudConstPtr &cloud)
{


  cloud_pass_.reset (new Cloud);
  cloud_pass_downsampled_.reset (new Cloud);
  filterPassThrough (cloud, cloud_pass_);
  gridSampleApprox (cloud_pass_, cloud_pass_downsampled_, downsampling_grid_size_);
  computePlaneExtraction(cloud_pass_downsampled_, plane, convex_hull, objects);

  boost::shared_ptr<std::vector<pcl::PointIndices> > clusters (new std::vector<pcl::PointIndices>);
  computeEuclideanSegmentation(objects, clusters);
  ROS_WARN_STREAM(clusters->size());
  publishPointCloud(plane, plane_msg, pub_plane);
  publishPointCloud(objects, objects_msg, pub_objects);


  int j=1;
  typedef std::pair <int, float> match;
  std::vector<match> best_match;
  pcl::PointCloud<XYZ>::Ptr choosen_cluster (new pcl::PointCloud<XYZ>);
  int score_distance;
  std::string choosen_model;


  for(std::vector<pcl::PointIndices>::const_iterator i = clusters->begin(); i != clusters->end(); ++i)
  {
    //sprintf (cluster, "clusters_%d", j);
    pcl::PointCloud<XYZ>::Ptr cluster (new pcl::PointCloud<XYZ>);
    for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
    {

      cluster->points.push_back(objects->points[*point]);
    }
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;




   }


}
void roscloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  received_cloud_ptr.reset(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg (*input, *received_cloud_ptr);
  cloud_cb (received_cloud_ptr);
}

int main (int argc, char** argv) {

  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

 // ros::Subscriber sub = nh.subscribe ("/stereo/points2", 1, roscloud_cb);
 ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, roscloud_cb);
  pub_objects=nh.advertise<sensor_msgs::PointCloud2>("objects",1);
  pub_convex_hull=nh.advertise<sensor_msgs::PointCloud2>("convex_hull",1);
  pub_plane=nh.advertise<sensor_msgs::PointCloud2>("plane",1);
  pub_choosen=nh.advertise<sensor_msgs::PointCloud2>("choosen",1);
  pub_final=nh.advertise<sensor_msgs::PointCloud2>("final",1);

  counter = 0;

    ros::Rate loop_rate(40);
     while(ros::ok())
     {
         ros::spinOnce();
         loop_rate.sleep();
      }


      return 0;
}

