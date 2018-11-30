#include "ipa_akit_detection.h"

ipa_akit_detection::ipa_akit_detection(ros::NodeHandle nh)
    : node_handle_(nh), cloud(new Cloud)
{
  vis_.reset(new pcl::visualization::PCLVisualizer("~"));
  //vis_->addCoordinateSystem(1);
  vis_->setBackgroundColor(0, 0, 0);

  if (nh.getParam("/topic_name", topic_name))
    ;
  else
    topic_name = "/camera/depth_registered/points";
  sub_ = nh.subscribe(topic_name, 1, &ipa_akit_detection::cloud_cb, this);

  visualisation_ = thread(&ipa_akit_detection::run, this);

  //visualisation_.join();
  capture_ = false;
  previous_data_size_ = 0;
  previous_clusters_size_ = 0;
  data_modified_ = true;

  display_normals_ = false;
  display_curvature_ = false;
  display_distance_map_ = false;

  use_planar_refinement_ = true;
  use_clustering_ = false;
  output_plane_pub_=node_handle_.advertise<sensor_msgs::PointCloud2>("plane", 5);
  output_pointcloud_pub_=node_handle_.advertise<sensor_msgs::PointCloud2>("pc", 5);

  euclidean_comparator_.reset (new pcl::EuclideanPlaneCoefficientComparator<PointT, pcl::Normal> ());
  euclidean_cluster_comparator_.reset(new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());

}

ipa_akit_detection::~ipa_akit_detection() {}

void ipa_akit_detection::removePreviousDataFromScreen(
    size_t prev_models_size, size_t prev_clusters_size,
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  char name[1024];
  for (size_t i = 0; i < prev_models_size; i++)
  {
    sprintf(name, "normal_%d", unsigned(i));
    viewer->removeShape(name);

    sprintf(name, "plane_%02d", int(i));
    viewer->removePointCloud(name);
  }

  for (size_t i = 0; i < prev_clusters_size; i++)
  {
    sprintf(name, "cluster_%d", int(i));
    viewer->removePointCloud(name);
  }
}

void ipa_akit_detection::cloud_cb(
    const sensor_msgs::PointCloud2ConstPtr& input_pointcloud_msg)
{

  lock_guard<mutex> lock(m_);
  ROS_INFO("cloud-cb");
  input_pointcloud.reset(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*input_pointcloud_msg, *input_pointcloud);

  prev_cloud_ = *input_pointcloud;


  //Voxel grid
  pcl::VoxelGrid<PointT> vg;

  cloud_filtered.reset(new pcl::PointCloud<PointT>);
  vg.setInputCloud(input_pointcloud);
  vg.setLeafSize(0.005f, 0.005f, 0.005f);
  vg.filter(*cloud_filtered);
  std::cout << "PointCloud after filtering has: "
            << cloud_filtered->points.size() << " data points." << std::endl;

  if (cloud_filtered->points.size() == 0)
    return;

  std::vector<int> NANindex;
  //pcl::removeNaNFromPointCloud(*cloud_filtered0, *cloud_filtered, NANindex)  ;

  std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
  //Normals
  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud (cloud_filtered);
  ne.compute (*normal_cloud);

  if(cloud_filtered->points.size() != normal_cloud->points.size())
    return;

  eapc.reset(new EdgeAwarePlaneComparator<PointT, pcl::Normal>);
  eapc->setDistanceMap (ne.getDistanceMap ());
  eapc->setDistanceThreshold (0.01f, false);

  //Plane segmentation
  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<pcl::PointIndices> inlier_indices;
  pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;
  mps.setInputNormals (normal_cloud);
  mps.setInputCloud (cloud_filtered);
  //mps.segment (regions);
  mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

  prev_regions_ = regions;
  ROS_INFO("plane-seg");

  for(int i=0; i<model_coefficients.size(); ++i)
  {
    Eigen::Vector3f normal1;
    normal1 << model_coefficients[i].values[0],model_coefficients[i].values[1],model_coefficients[i].values[2];
    for(int j=1; j<model_coefficients.size(); ++j)
    {
      Eigen::Vector3f normal2;
      normal2 << model_coefficients[j].values[0],model_coefficients[j].values[1],model_coefficients[j].values[2];
      float dot_normal = normal1.dot(normal2);
      Eigen::Vector3f cross_product = normal1.cross(normal2);
      float cross_normalized = cross_product.norm();
      float angle = acos(dot_normal/cross_normalized) * 180 / M_PI ;

      if(angle < 89 || angle > 91) continue;
        //cout << angle <<endl;
        for(int k=2; k<model_coefficients.size(); ++k)
        {
          Eigen::Vector3f normal3;
          normal3 << model_coefficients[k].values[0],model_coefficients[k].values[1],model_coefficients[k].values[2];
          float dot_normal3 = cross_product.dot(normal3);
          if(dot_normal3 <0.98 || dot_normal3>1.02) continue;
          cout<<dot_normal3<<endl;
        }

    }

  }


  //Cluster segementation
  pcl::PointCloud<PointT>::CloudVectorType clusters;
//  //vector<bool> plane_labels;
//  vector<bool> plane_labels;
//  if (regions.size () > 0)
//   {

//     for (size_t i = 0; i < label_indices.size (); ++i)
//     {
//       if (label_indices[i].indices.size () > 10000)
//         plane_labels.push_back(true);
//      else
//         plane_labels.push_back(false);
//     }
//  }

//     euclidean_cluster_comparator_->setInputCloud (cloud_filtered);
//     euclidean_cluster_comparator_->setLabels (labels);
//     //setExludeLabels takes in vector<bool>
//     euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
//     euclidean_cluster_comparator_->setDistanceThreshold (0.01f, false);
//  //ROS_INFO("ecc");
//     pcl::PointCloud<pcl::Label> euclidean_labels;
//     std::vector<pcl::PointIndices> euclidean_label_indices;
//     //ROS_INFO("ecc1");
//     pcl::OrganizedConnectedComponentSegmentation<PointT,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
//     ROS_INFO("ecc2");

//     euclidean_segmentation.setInputCloud (cloud_filtered);
//     //ROS_INFO("ecc3");

//     euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);
////ROS_INFO("ecc4");
//     for (size_t i = 0; i < euclidean_label_indices.size (); i++)
//     {
//       if (euclidean_label_indices[i].indices.size () > 1000)
//       {
//         pcl::PointCloud<PointT> cluster;
//         pcl::copyPointCloud (*cloud_filtered, euclidean_label_indices[i].indices,cluster);
//         clusters.push_back (cluster);
//       }



//    }
     prev_clusters_ = clusters;
// cout<<"Got %d euclidean clusters!\n"<<clusters.size ()<<endl;


//  //Cylinder segmentation - slow
//  ROS_INFO("cylinder segmentation");
//  inliers_cylinder_.reset(new pcl::PointIndices);
//  coefficients_cylinder_.reset(new pcl::ModelCoefficients);
//  seg.setOptimizeCoefficients (true);
//    seg.setModelType (pcl::SACMODEL_CYLINDER);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setNormalDistanceWeight (0.1);
//    seg.setMaxIterations (10000);
//    seg.setDistanceThreshold (0.05);
//    seg.setRadiusLimits (0, 0.1);
//    seg.setInputCloud (cloud_filtered);
//  seg.setInputNormals (normal_cloud);
//  seg.segment (*inliers_cylinder_, *coefficients_cylinder_);
//  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder_ << std::endl;




//  pcl::SACSegmentation<PointT> seg;
//  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//  pcl::ModelCoefficients coefficients; // (new pcl::ModelCoefficients);
//  pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
//  pcl::PCDWriter writer;
//  seg.setOptimizeCoefficients(true);
//  seg.setModelType(pcl::SACMODEL_PLANE);
//  seg.setMethodType(pcl::SAC_RANSAC);
//  seg.setMaxIterations(100);
//  seg.setDistanceThreshold(0.015);
//  int planeRemovals = 0;
//  int nr_points = (int)cloud_filtered->points.size();

//  while (cloud_filtered->points.size() > 0.2 * nr_points && planeRemovals < 6)
//  {
//    // Segment the largest planar component from the remaining cloud
//    seg.setInputCloud(cloud_filtered);
//    seg.segment(*inliers, coefficients);
//    if (inliers->indices.size() == 0)
//    {
//      std::cout << "Could not estimate a planar model for the given dataset."
//                << std::endl;
//      break;
//    }

//    std::cout << "PointCloud representing the planar component: "
//              << cloud_filtered->size() - inliers->indices.size()
//              << " data points." << std::endl;

//    //				pcl::PointCloud<PointType> temp;
//    //				for (unsigned int i=0; i<input_pointcloud.size();
//    //i++)
//    //					if
//    //(fabs(input_pointcloud[i].x*coefficients.values[0]+input_pointcloud[i].y*coefficients.values[1]+input_pointcloud[i].z*coefficients.values[2]+coefficients.values[3])
//    //> 0.02)
//    //						temp.push_back(input_pointcloud[i]);
//    //				input_pointcloud = temp;

//    planeRemovals++;

//    // Extract the planar inliers from the input cloud
//    pcl::ExtractIndices<PointT> extract;
//    extract.setInputCloud(cloud_filtered);
//    extract.setIndices(inliers);
//    extract.setNegative(false);

//    // Write the planar inliers to disk
//    extract.filter(*cloud_plane);
//    sensor_msgs::PointCloud2 output_plane_msg;
//    pcl::toROSMsg(*cloud_plane, output_plane_msg);
//    output_plane_pub_.publish(output_plane_msg);
//    std::cout << "PointCloud representing the planar component: "
//              << cloud_plane->points.size() << " data points." << std::endl;

//    //				extract.setNegative (false);
//    //
//    //				// Write the planar inliers to disk
//    //				extract.filter (*cloud_plane);
//    //				std::cout << "PointCloud representing the planar component: " <<
//    //cloud_plane->points.size () << " data points." << std::endl;
//    //

//    // Remove the planar inliers, extract the rest
//    extract.setNegative(true);
//    extract.filter(*cloud_filtered);
//  }

//  //		cloud_filtered->header.stamp =
//  //input_pointcloud_msg->header.stamp;
//  //		cloud_filtered->header.frame_id =
//  //input_pointcloud_msg->header.frame_id;
//  //		sensor_msgs::PointCloud2 output_pointcloud_msg;
//  //		pcl::toROSMsg(*cloud_filtered, output_pointcloud_msg);
//  //		output_pointcloud_pub_.publish(output_pointcloud_msg);

  // Creating the KdTree object for the search method of the extraction
//  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
//  // pcl::search::Search<PointT>::Ptr tree (new pcl_search<PointT>);
//  tree->setInputCloud(cloud_filtered);

//  std::vector<pcl::PointIndices> cluster_indices;
//  pcl::EuclideanClusterExtraction<PointT> ec;
//  ec.setClusterTolerance(0.1); // 2cm
//  ec.setMinClusterSize(50);
//  ec.setMaxClusterSize(25000);
//  ec.setSearchMethod(tree);
//  // pcl::PointCloud<PointType>::ConstPtr
//  // input_pointcloud_ptr(&input_pointcloud);
//  ec.setInputCloud(cloud_filtered);
//  ec.extract(cluster_indices);

//  // cob_perception_msgs::PointCloud2Array output_pointcloud_segments_msg;
//  int j = 0;
//  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();it != cluster_indices.end(); ++it)
//  {
//    pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
//    pcl::PointXYZ avgPoint;
//    avgPoint.x = 0;
//    avgPoint.y = 0;
//    avgPoint.z = 0;
//    for (std::vector<int>::const_iterator pit = it->indices.begin();
//         pit != it->indices.end(); pit++)
//    {
//      cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
//      avgPoint.x += cloud_filtered->points[*pit].x;
//      avgPoint.y += cloud_filtered->points[*pit].y;
//    }

//    std::cout << "PointCloud representing the Cluster: "
//              << cloud_cluster->points.size() << " data points." << std::endl;

//    //				if ((fabs(avgPoint.x) < cloud_cluster->points.size()*/*0.15*/0.5)
//    //&& (fabs(avgPoint.y) < /*0.30*/0.5*cloud_cluster->points.size()) &&
//    //(fabs(avgPoint.z) < 1.0*cloud_cluster->points.size()))
//    if ((fabs(avgPoint.x) < cloud_cluster->points.size() * /*0.15*/ 0.3) &&
//        (fabs(avgPoint.y) < /*0.30*/ 0.4 * cloud_cluster->points.size()) &&
//        (fabs(avgPoint.z) < 1.2 * cloud_cluster->points.size()))
//    {
//      std::cout << "found a cluster in the center" << std::endl;

//      cloud_cluster->header = pcl_conversions::toPCL(input_pointcloud_msg->header);
//      sensor_msgs::PointCloud2 output_pointcloud_msg;
//      pcl::toROSMsg(*cloud_cluster, output_pointcloud_msg);
//      //					std::string filename =
//      //ros::package::getPath("cob_object_categorization") + "/test.pcd";
//      //					pcl::io::savePCDFileASCII(filename.c_str(),
//      //*cloud_cluster);
//      // output_pointcloud_segments_msg.segments.push_back(output_pointcloud_msg);
//    }
//    // std::stringstream ss;
//    // ss << "cloud_cluster_" << j << ".pcd";
//    // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);
//    j++;

//  }


  // output_pointcloud_segments_msg.header = input_pointcloud_msg->header;
  output_pointcloud_pub_.publish(output_pointcloud_segments_msg);
  last_publishing_time_ = ros::Time::now();
}



void ipa_akit_detection::run()
{

  while (!vis_->wasStopped())
  {
    m_.lock();
    ROS_INFO("vis thread");
    {

      removePreviousDataFromScreen(previous_data_size_, previous_clusters_size_,
                                   vis_);
      std::cout<<prev_cloud_.size()<<std::endl;
      if (!vis_->updatePointCloud(boost::make_shared<Cloud>(prev_cloud_),
                                  "cloud"))
      {
        std::cout<<"updating pointcloud"<<std::endl;
        vis_->addPointCloud(boost::make_shared<Cloud>(prev_cloud_), "cloud");
        vis_->resetCameraViewpoint("cloud");


      }
      displayPlanarRegions(prev_regions_, vis_);
     // displayEuclideanClusters (prev_clusters_,vis_);
      previous_data_size_ = prev_regions_.size();
      previous_clusters_size_ = prev_clusters_.size ();

      data_modified_ = false;
    }
    m_.unlock();
    vis_->spinOnce(100);
    this_thread::sleep_for(chrono::milliseconds(1));
  }
}
void ipa_akit_detection::displayPlanarRegions (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > &regions,
                      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  char name[1024];
  unsigned char red [6] = {255,   0,   0, 255, 255,   0};
  unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
  unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

  pcl::PointCloud<PointT>::Ptr contour (new pcl::PointCloud<PointT>);

  for (size_t i = 0; i < regions.size (); i++)
  {
    Eigen::Vector3f centroid = regions[i].getCentroid ();
    Eigen::Vector4f model = regions[i].getCoefficients ();
    pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
    pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
                                       centroid[1] + (0.5f * model[1]),
                                       centroid[2] + (0.5f * model[2]));
    sprintf (name, "normal_%d", unsigned (i));
    viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, name);

    contour->points = regions[i].getContour ();
    sprintf (name, "plane_%02d", int (i));
    pcl::visualization::PointCloudColorHandlerCustom <PointT> color (contour, red[i%6], grn[i%6], blu[i%6]);
    if(!viewer->updatePointCloud(contour, color, name))
      viewer->addPointCloud (contour, color, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
  }
}

void ipa_akit_detection::displayEuclideanClusters(const pcl::PointCloud<PointT>::CloudVectorType &clusters,
                                                  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  char name[1024];
    unsigned char red [6] = {255,   0,   0, 255, 255,   0};
    unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
    unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

    for (size_t i = 0; i < clusters.size (); i++)
    {
      sprintf (name, "cluster_%d" , int (i));
      pcl::visualization::PointCloudColorHandlerCustom<PointT> color0(boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),red[i%6],grn[i%6],blu[i%6]);
      if (!viewer->updatePointCloud (boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),color0,name))
        viewer->addPointCloud (boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),color0,name);
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);
  }
}
void ipa_akit_detection::spin() {}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ipa_akit_detection");

  ros::NodeHandle nh;

  ipa_akit_detection iad(nh);

  ros::spin();
  return 0;
}
