
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

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
#include <pcl/io/ply_io.h>
#include <iostream>
#include <fstream>

ros::Publisher pub;





using namespace pcl::tracking;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointXYZRGB RefPointType;
typedef ParticleXYZRPY ParticleT;

typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

pcl::PointCloud<PointType>::Ptr received_cloud_ptr;
sensor_msgs::PointCloud2 particle_cloud_msg, result_cloud_msg, down_cloud_msg;
ros::Publisher pub_praticles_cloud,pub_result_cloud,  pub_down_cloud;

boost::shared_ptr<pcl::PointCloud<PointType> > result_cloud;


CloudPtr cloud_pass_;
CloudPtr cloud_pass_downsampled_;
CloudPtr target_cloud;



boost::shared_ptr<ParticleFilter> tracker_;
bool new_cloud_;
double downsampling_grid_size_;
int counter;
double x,y,z;
double quaternion_x,quaternion_y,quaternion_z,quaternion_w;

//Filter along a specified dimension
void filterPassThrough (const CloudConstPtr &cloud, Cloud &result)
{
  pcl::PassThrough<PointType> pass;
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 2.0);
  pass.setKeepOrganized (false);
  pass.setInputCloud (cloud);
  pass.filter (result);
}


void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size)
{
  pcl::ApproximateVoxelGrid<PointType> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
}

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


   // std::cout<<"x: " << x <<std::endl;
    //std::cout<<"y: " << y <<std::endl;
    //std::cout<<"z: " << z <<std::endl;
//    std::cout<<"roll is:" <<atan2( rotationMatrix(2,1),rotationMatrix(2,2) ) <<std::endl;
//    std::cout<<"pitch is:" <<atan2( -rotationMatrix(2,0), std::pow( rotationMatrix(2,1)*rotationMatrix(2,1) +rotationMatrix(2,2)*rotationMatrix(2,2) ,0.5  )  ) <<std::endl;
//    std::cout<<"yaw is:" <<atan2( rotationMatrix(1,0),rotationMatrix(0,0) ) <<std::endl;


/*
    std::cout<<"quaternion_x is:" <<quaternion_x <<std::endl;
    std::cout<<"quaternion_y is:" <<quaternion_y <<std::endl;
    std::cout<<"quaternion_z is:" <<quaternion_z <<std::endl;
    std::cout<<"quaternion_w is:" <<quaternion_w <<std::endl;
    //myfile << x << " " << y << " " << z << " \n";
*/

}
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

/*
bool
drawParticles (pcl::visualization::PCLVisualizer& viz)
{
  ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
  if (particles && new_cloud_)
    {
      //Set pointCloud with particle's points
      pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      for (size_t i = 0; i < particles->points.size (); i++)
  {
    pcl::PointXYZ point;

    point.x = particles->points[i].x;
    point.y = particles->points[i].y;
    point.z = particles->points[i].z;
    particle_cloud->points.push_back (point);
  }

      //Draw red particles
      {
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color (particle_cloud, 250, 99, 71);

  if (!viz.updatePointCloud (particle_cloud, red_color, "particle cloud"))
    viz.addPointCloud (particle_cloud, red_color, "particle cloud");
      }
      return true;
    }
  else
    {
      return false;
    }
}
*/
//Draw model reference point cloud
void
drawResult ()
{
  ParticleXYZRPY result = tracker_->getResult ();
  Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);
  publishObjectPose(transformation);

  transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
//  CloudPtr result_cloud (new Cloud ());
  result_cloud.reset(new pcl::PointCloud<PointType>);
  pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);


  {
    pcl::toROSMsg(*result_cloud,result_cloud_msg );
           // result_cloud_msg.header.frame_id = "/camera_color_optical_frame";
            result_cloud_msg.header.frame_id = "camera";
            pub_result_cloud.publish(result_cloud_msg );
  }
}
/*
void
drawResult (pcl::visualization::PCLVisualizer& viz)
{
  ParticleXYZRPY result = tracker_->getResult ();
  Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);

  //move close to camera a little for better visualization
  transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
  CloudPtr result_cloud (new Cloud ());
  pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);

  //Draw blue model reference point cloud
  {
    pcl::visualization::PointCloudColorHandlerCustom<RefPointType> blue_color (result_cloud, 0, 0, 255);

    if (!viz.updatePointCloud (result_cloud, blue_color, "resultcloud"))
      viz.addPointCloud (result_cloud, blue_color, "resultcloud");
  }
}

//visualization's callback function
*/
/*
void
viz_cb (pcl::visualization::PCLVisualizer& viz)
{

  //Draw downsampled point cloud from sensor
  if (new_cloud_ && cloud_pass_downsampled_)
    {
      CloudPtr cloud_pass;
      cloud_pass = cloud_pass_downsampled_;

      if (!viz.updatePointCloud (cloud_pass, "cloudpass"))
  {
    viz.addPointCloud (cloud_pass, "cloudpass");
    viz.resetCameraViewpoint ("cloudpass");
  }
      bool ret = drawParticles (viz);
      if (ret)
        drawResult (viz);
    }
  new_cloud_ = false;
}
*/
//OpenNI Grabber's cloud Callback function
void
cloud_cb (const CloudConstPtr &cloud)
{

  cloud_pass_.reset (new Cloud);
  cloud_pass_downsampled_.reset (new Cloud);
  filterPassThrough (cloud, *cloud_pass_);
  gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);

  pcl::toROSMsg(*cloud_pass_downsampled_,down_cloud_msg );
   //  cloud_pass_downsampled_.header.frame_id = "/camera_color_frame";
     pub_down_cloud.publish(down_cloud_msg );


  if(counter < 10){
  counter++;
  }else{
    //Track the object
  tracker_->setInputCloud (cloud_pass_downsampled_);
  tracker_->compute ();
  new_cloud_ = true;
  }

}
void
roscloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud

  received_cloud_ptr.reset(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg (*input.get(), *received_cloud_ptr.get());

  cloud_cb (received_cloud_ptr);
  drawParticles();
  drawResult();





}

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;


// ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, roscloud_cb);
 ros::Subscriber sub = nh.subscribe ("/stereo/points2", 1, roscloud_cb);
 pub_praticles_cloud=nh.advertise<sensor_msgs::PointCloud2>("praticles_cloud",1);

 pub_result_cloud=nh.advertise<sensor_msgs::PointCloud2>("result_cloud",1);
pub_down_cloud=nh.advertise<sensor_msgs::PointCloud2>("cloud_pass_downsampled_",1);

  target_cloud.reset(new Cloud());

   //read pcd file
pcl::PointCloud<pcl::PointXYZ> target_cloud_XYZ;
  if(pcl::io::loadPCDFile("/home/pfs-mt/cloud.pcd", target_cloud_XYZ) == -1){
    std::cout << "pcd file not found" << std::endl;
    exit(-1);
  }
  pcl::copyPointCloud(target_cloud_XYZ, *target_cloud);

   counter = 0;

   //Set parameters
   new_cloud_  = false;
   downsampling_grid_size_ =  0.005;

   std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
   default_step_covariance[3] *= 40.0;
   default_step_covariance[4] *= 40.0;
   default_step_covariance[5] *= 40.0;

   std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
   std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

   boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
     (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8));

   ParticleT bin_size;
   bin_size.x = 0.1f;
   bin_size.y = 0.1f;
   bin_size.z = 0.1f;
   bin_size.roll = 0.1f;
   bin_size.pitch = 0.1f;
   bin_size.yaw = 0.1f;


   //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
   tracker->setMaximumParticleNum (500);
   tracker->setDelta (0.99);
   tracker->setEpsilon (0.2);
   tracker->setBinSize (bin_size);

   //Set all parameters for  ParticleFilter
   tracker_ = tracker;
   tracker_->setTrans (Eigen::Affine3f::Identity ());
   tracker_->setStepNoiseCovariance (default_step_covariance);
   tracker_->setInitialNoiseCovariance (initial_noise_covariance);
   tracker_->setInitialNoiseMean (default_initial_mean);
   tracker_->setIterationNum (3);
   tracker_->setParticleNum (400);
   tracker_->setResampleLikelihoodThr(0.00);
   tracker_->setUseNormal (false);


   //Setup coherence object for tracking
ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
     (new ApproxNearestPairPointCloudCoherence<RefPointType> ());

   boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
     = boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
   coherence->addPointCoherence (distance_coherence);



   boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.01));
   coherence->setSearchMethod (search);
   coherence->setMaximumDistance (2);

   tracker_->setCloudCoherence (coherence);

   //prepare the model of tracker's target
   Eigen::Vector4f c;
   Eigen::Affine3f trans = Eigen::Affine3f::Identity ();

   CloudPtr transed_ref (new Cloud);
   CloudPtr transed_ref_downsampled (new Cloud);

   pcl::compute3DCentroid<RefPointType> (*target_cloud, c);
   /*
   c[0]=0;
   c[1]=0;
   c[2]=1;
*/
   trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
   std::cout << "x : " << c[0] << " :: y : " << c[1] << " :: z : " << c[2] << std::endl;
   pcl::transformPointCloud<RefPointType> (*target_cloud, *transed_ref, trans.inverse());

 gridSampleApprox (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);

   //set reference model and trans
   tracker_->setReferenceCloud (transed_ref_downsampled);
   tracker_->setTrans (trans);

  //computeMomentOfInertia(result_cloud);





/*
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.runOnVisualizationThread(viz_cb);
    while (!viewer.wasStopped ())
       {
       }
       */
    tf::TransformBroadcaster br;
    tf::Transform transform2;

   ros::Rate loop_rate(40);
      while(ros::ok())
      {
          ros::spinOnce();

          loop_rate.sleep();

          transform2.setOrigin( tf::Vector3(x, y, z) );

      transform2.setRotation( tf::Quaternion( quaternion_x,quaternion_y,quaternion_z,quaternion_w  ) );

    //br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "/camera_color_optical_frame", "object"));
    br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "world", "object"));

    }
      return 0;
}

