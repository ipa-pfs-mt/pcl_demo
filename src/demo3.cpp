#include "ipa_pcl_demo/demo3.h"

   ObjectDetection::ObjectDetection(ros::NodeHandle nh) : node_handle_(nh), plane (new Cloud), convex_hull (new Cloud), objects (new Cloud),
     normals (new pcl::PointCloud<pcl::Normal>), descriptors (new pcl::PointCloud<pcl::VFHSignature308>)
   {
   ROS_WARN("object detection constructor");
   sub_ = nh.subscribe ("/stereo/points2", 1, &ObjectDetection::roscloud_cb, this);
   pub_objects=nh.advertise<sensor_msgs::PointCloud2>("objects",1);
   pub_convex_hull=nh.advertise<sensor_msgs::PointCloud2>("convex_hull",1);
   pub_plane=nh.advertise<sensor_msgs::PointCloud2>("plane",1);

   /*
   extension = (".pcd");
   received_cloud_ptr = new pcl::PointCloud<PointType>;
   cloud_pass_ = new Cloud;
   cloud_pass_downsampled_ = new Cloud;
   target_cloud_ = new Cloud;

   keypoints_scene = new pcl::PointCloud<XYZ>;
   keypoints_object = new pcl::PointCloud<XYZ>;
   normals_scene = new pcl::PointCloud<pcl::Normal>;
   descriptors_scene = new pcl::PointCloud<pcl::SHOT352>();
   normals_object = new pcl::PointCloud<pcl::Normal>;
   descriptors_object = new pcl::PointCloud<pcl::SHOT352>();
   received_cloud_ptr = new pcl::PointCloud<PointType>;

   plane = new Cloud;
   convex_hull = new Cloud;
   objects = new Cloud;
   normals = new pcl::PointCloud<pcl::Normal>;
   descriptors = new pcl::PointCloud<pcl::VFHSignature308>;
   histogram_object = new pcl::PointCloud<CRH90>;
   histogram_scene = new pcl::PointCloud<CRH90>;
*/
   }

   ObjectDetection::~ObjectDetection(){}

   void ObjectDetection::filterPassThrough(const CloudConstPtr &cloud, CloudPtr &result)
   {
     pcl::PassThrough<PointType> pass;
     pass.setFilterFieldName ("z");
     pass.setFilterLimits (0.0, 1.5);
     pass.setKeepOrganized (true);
     pass.setInputCloud (cloud);
     pass.filter (*result);
   }


   void ObjectDetection::gridSampleApprox (const CloudConstPtr &cloud, CloudPtr &result, double leaf_size)
   {
     pcl::ApproximateVoxelGrid<PointType> grid;
     grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
     grid.setInputCloud (cloud);
     grid.filter (*result);
   }

   void ObjectDetection::publishObjectPose(Eigen::Affine3f &transformation)
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


   void ObjectDetection::viz_cb (pcl::visualization::PCLVisualizer& viz)
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
   ObjectDetection::computeCloudResolution(const pcl::PointCloud<XYZ>::ConstPtr &cloud)
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

   void ObjectDetection::computeISSKeypoint3D(const CloudConstPtr &cloud, CloudXYZPtr &keypoints_cloud)
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

   void ObjectDetection::computeSHOTDescriptors(const CloudXYZConstPtr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals,
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
   void ObjectDetection::computeCorrespondences (const pcl::PointCloud<DescriptorsT>::Ptr &scene_descriptors,
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
   void ObjectDetection::computeCorrespondenceGrouping (const pcl::PointCloud<XYZ>::Ptr &keypoints_scene,
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
   void ObjectDetection::publishPointCloud (const CloudConstPtr &cloud, sensor_msgs::PointCloud2 msgs, ros::Publisher pub)
   {
     pcl::toROSMsg(*cloud.get(), msgs); /*
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
         if (loadHist (base_dir / it->path ().filename (), m))
           models.push_back (m);
       }
     }
   }
   */
     msgs.header.frame_id = "camera";
     pub.publish(msgs);
   }
   void ObjectDetection::displayClusters(const CloudConstPtr &cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer)
   {

   }
  inline void ObjectDetection::nearestKSearch(flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model,
                       int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
   {
     flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size()], 1, model.second.size());
     memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

       indices = flann::Matrix<int>(new int[k], 1, k);
       distances = flann::Matrix<float>(new float[k], 1, k);
       ROS_WARN_STREAM("object col: "<< p.cols);
       index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
       delete[] p.ptr ();


   }
   void ObjectDetection::computePlaneExtraction(const CloudConstPtr &cloud, CloudPtr &plane, CloudPtr &convex_hull, CloudPtr &objects){

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
   void ObjectDetection::computeEuclideanSegmentation(const CloudConstPtr &cloud,
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

   void ObjectDetection::computeOURCVFHDescriptor(const CloudConstPtr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals,
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
   void ObjectDetection::computeCVFHDescriptor(const CloudConstPtr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals,
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

   void ObjectDetection::computeCRH(const CloudConstPtr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals,
              pcl::PointCloud<CRH90>::Ptr &histogram, boost::shared_ptr<Eigen::Vector4f> &centroid)
   {
     pcl::NormalEstimation<XYZ, pcl::Normal> normalEstimation;
     normalEstimation.setInputCloud(cloud);
     normalEstimation.setRadiusSearch(0.03);
     pcl::search::KdTree<XYZ>::Ptr kdtree(new pcl::search::KdTree<XYZ>);
     normalEstimation.setSearchMethod(kdtree);
     normalEstimation.compute(*normals);

     pcl::CRHEstimation<XYZ, pcl::Normal, pcl::Histogram<90>> crh;
     crh.setInputCloud(cloud);
     crh.setInputNormals(normals);
     pcl::compute3DCentroid(*cloud, *centroid);
     crh.setCentroid(*centroid);
     crh.compute(*histogram);
   }
   void ObjectDetection::computeCentroidsAlignment(const CloudConstPtr &scene, const CloudConstPtr &object,
                             pcl::PointCloud<CRH90>::Ptr &histogram_scene, pcl::PointCloud<CRH90>::Ptr &histogram_object,
                             Eigen::Vector4f &scene_centroid, Eigen::Vector4f &object_centroid)
   {

   }
   bool ObjectDetection::loadHist (const std::string filename, const pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptors,
             vfh_model &vfh)
   {
     vfh.second.resize (308);

     for (size_t i = 0; i < descriptors->points.size(); ++i)
     {
         vfh.second[i] = descriptors->points[0].histogram[i];
           //point.points[0].histogram[i];
     }
     vfh.first = filename;
     return (true);
   }
   void ObjectDetection::loadModels (const boost::filesystem::path &base_dir, const std::string &extension,
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
         boost::shared_ptr<Eigen::Vector4f> centroids_temp (new Eigen::Vector4f);

          //if (loadHist (base_dir / it->path ().filename (), model))
         //ROS_WARN_STREAM(it->path().filename()  );
         std::string name = base_dir.string() + "/" + it->path().filename().string();

         ROS_WARN_STREAM(name );
         pcl::io::loadPCDFile(name, *model);
         model_partial_views.push_back (model);
         computeCVFHDescriptor(model, normals_temp, descriptors_temp);

         vfh_model m;
         if (loadHist (it->path().filename().string(), descriptors_temp ,m))
         models.push_back (m);

         model_partial_views_descriptors.push_back(descriptors_temp);
         computeCRH(model, normals_temp, histograms_temp, centroids_temp);
         model_partial_views_histograms.push_back(histograms_temp);
         model_partial_views_centroids.push_back(centroids_temp);

         //Saving histograms
         //std::string name_save = name+"vfh.pcd";
         //pcl::io::savePCDFileASCII(name_save, *descriptors_temp);


       }
     }
   }

 bool ObjectDetection::loadHisto (const boost::filesystem::path &path, vfh_model &vfh)
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

   void ObjectDetection::loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension,
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
         vfh_model m;
         if (loadHisto (base_dir / it->path ().filename (), m))
           models.push_back (m);
       }
     }
   }

   void ObjectDetection::cloud_cb (const CloudConstPtr &cloud)
   {
     ROS_WARN("Start cloud cb");
     std::string extension (".pcd");


    /* if (model_partial_views.empty())
    {

       loadModels("/home/pfs-mt/Documents/scanned_every_30_deg", extension, models, model_partial_views, model_partial_views_descriptors,
   model_partial_views_histograms, model_partial_views_centroids);

       ROS_WARN_STREAM("size: "<< models.size());
  }*/
     if(models.empty())
     {
    loadFeatureModels("/home/pfs-mt/Documents/cvfhs", extension, models);
     ROS_WARN_STREAM("Loading "<< (unsigned long)models.size());
      }

       flann::Matrix<float> data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());
         for (size_t i = 0; i < data.rows; ++i)
           for (size_t j = 0; j < data.cols; ++j)
             data[i][j] = models[i].second[j];

       ROS_WARN("Building the flann structure for %d elements...\n", (int)data.rows );
       flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
       index.buildIndex ();
       ROS_WARN("Building the kdtree index  for %d elements...\n", (int)data.rows );

     cloud_pass_.reset (new Cloud);
     cloud_pass_downsampled_.reset (new Cloud);
     filterPassThrough (cloud, cloud_pass_);
     gridSampleApprox (cloud_pass_, cloud_pass_downsampled_, downsampling_grid_size_);
     computePlaneExtraction(cloud_pass_downsampled_, plane, convex_hull, objects);
     boost::shared_ptr<std::vector<pcl::PointIndices> > clusters (new std::vector<pcl::PointIndices>);
     computeEuclideanSegmentation(objects, clusters);
     ROS_WARN_STREAM(clusters->size());

     char cluster[1024];
     int j=1;
     for(std::vector<pcl::PointIndices>::const_iterator i = clusters->begin(); i != clusters->end(); ++i)
     {
       sprintf (cluster, "clusters_%d", j);
       pcl::PointCloud<XYZ>::Ptr cluster (new pcl::PointCloud<XYZ>);
       for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
         cluster->points.push_back(objects->points[*point]);
       cluster->width = cluster->points.size();
       cluster->height = 1;
       cluster->is_dense = true;
       j++;
       computeCVFHDescriptor(cluster, normals, descriptors);

       vfh_model histogram;

       histogram.second.resize(308);
       for (size_t h=0; h<308;++h)
       {
         histogram.second[h] = descriptors->points[0].histogram[h];
       }

       int k = 1;

       double thresh = DBL_MAX;
       flann::Matrix<int> k_indices;
       flann::Matrix<float> k_distances;
     ROS_WARN_STREAM((long unsigned int)data.cols);

      nearestKSearch(index, histogram, k, k_indices, k_distances);
    /*   ROS_WARN("The closest %d neighbors for cluster %s are:\n", k, i);
       for(int l = 0; l < k; l++)
         ROS_WARN("    %d - %s (%d) with a distance of: %f\n",
                  l, models.at(k_indices[0][l]).first.c_str(), k_indices[0][l], k_distances[0][l]);*/

     }

   }
   void ObjectDetection::roscloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
   {
      ROS_WARN("roscloud_cb");
     received_cloud_ptr.reset(new pcl::PointCloud<PointType>);
     pcl::fromROSMsg (*input, *received_cloud_ptr);
     cloud_cb (received_cloud_ptr);
   }
   /*
   void ObjectDetection::loadModelDataBase(flann::Matrix<float> *data1)
   {

     loadModels("/home/pfs-mt/Documents/scanned_every_30_deg", extension, models, model_partial_views, model_partial_views_descriptors,
                model_partial_views_histograms, model_partial_views_centroids);
     ROS_WARN_STREAM("size: "<< models.size());

     flann::Matrix<float> data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());
       for (size_t i = 0; i < data.rows; ++i)
         for (size_t j = 0; j < data.cols; ++j)
           data[i][j] = models[i].second[j];

     ROS_WARN("Building the flann structure for %d elements...\n", (int)data.rows );

    }

*/




