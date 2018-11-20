#include "ipa_akit_detection.h"

using namespace std;


ipa_akit_detection::ipa_akit_detection()
{

}

void ipa_akit_detection::cloud_cb(const CloudConstPtr &cloud)
{

}

void ipa_akit_detection::roscloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
  received_cloud_ptr.reset(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg (*input, *received_cloud_ptr);
  cloud_cb (received_cloud_ptr);
}



int
main (int argc, char ** argv)
{
  string topic_name;

  ros::NodeHandle nh;
  if (nh.getParam("/topic_name", topic_name));
  else topic_name = "/camera/depth_registered/points";
  ros::Subscriber sub = nh.subscribe (topic_name, 1, roscloud_cb);

  ipa_akit_detection iad;
  return 0;
}
