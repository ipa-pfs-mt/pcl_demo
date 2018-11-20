#ifndef IPA_AKIT_DETECTION_H
#define IPA_AKIT_DETECTION_H

#include "ipa_akit_detection_commons.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

class ipa_akit_detection
{
public:
  ipa_akit_detection();
  void cloud_cb(const CloudConstPtr& cloud);
protected:
  void roscloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);

  boost::mutex mtx_;
  boost::mutex vis_mtx_;


};

#endif // IPA_AKIT_DETECTION_H
