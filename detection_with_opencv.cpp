// ros includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//opencv includes
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

//io
#include <iostream>
#include <sstream>
#include <fstream>

#include <librealsense2/rs.hpp>

//tf includes
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <math.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

class Detection
{

  ros::NodeHandle node_handle;
  image_transport::ImageTransport image_transport;
  image_transport::Subscriber image_sub;
  image_transport::Subscriber image_sub1;
  image_transport::Publisher image_pub;
  Mat camera_matrix = Mat::eye(3, 3, CV_64F);
  Mat distance_coefficients;
  string path = "/home/pfs-mt/Documents/images from cutout objects/topf_cut_outs/2.png";
  Mat image;



 public:

  Detection() : image_transport(node_handle)
  {
  image_sub = image_transport.subscribe("/camera/color/image_raw", 1,
                                        &Detection::imageCb, this);
  //namedWindow("Window", WINDOW_AUTOSIZE);
  loadCameraCalibration("Camera Calib File", camera_matrix,
                        distance_coefficients);
  }
  ~Detection(){}

  bool loadCameraCalibration(string name, Mat& camera_matrix,
                             Mat& distance_coefficients)
  {
    //cout << "loadCameraCalibration function called" << endl;
    ifstream inStream(name);
    if (inStream)
    {
      uint16_t rows;
      uint16_t columns;

      inStream >> rows;
      inStream >> columns;

      camera_matrix = Mat(Size(columns, rows), CV_64F);

      for (int r = 0; r < rows; r++)
      {
        for (int c = 0; c < columns; c++)
        {
          double read = 0.0f;
          inStream >> read;
          camera_matrix.at<double>(r, c) = read;
        }
      }
      // Distance Coefficients
      inStream >> rows;
      inStream >> columns;

      distance_coefficients = Mat::zeros(rows, columns, CV_64F);

      for (int r = 0; r < rows; r++)
      {
        for (int c = 0; c < columns; c++)
        {
          double read = 0.0f;
          inStream >> read;
          distance_coefficients.at<double>(r, c) = read;
        }
      }
      inStream.close();
      return true;
      //cout << "saveCameraCalibration function success" << endl;
    }
    return false;
  }


  void imageCb(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


    }



  };



int main(int argc, char** argv)
{

  ros::init(argc, argv, "marker_detection");
  Detection md;


  Mat input = cv::imread(path);
  cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> detector = cv::xfeatures2d::SiftFeatureDetector::create();
  vector<cv::KeyPoint> kp1, kp2;
  Mat d1, d2;
  detector->detect(input, kp1);
  detector->compute(input, kp1, d1);


  detector->detect(cv_ptr->image, kp2);
  detector->compute(cv_ptr->image, kp2, d2);

  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
  std::vector< std::vector<DMatch> > knn_matches;
  matcher->knnMatch( d1, d2, knn_matches, 2 );
  //-- Filter matches using the Lowe's ratio test
  const float ratio_thresh = 0.7f;
  std::vector<DMatch> good_matches;
  for (size_t i = 0; i < knn_matches.size(); i++)
  {
      if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
      {
          good_matches.push_back(knn_matches[i][0]);
      }
  }
  //-- Draw matches
  Mat img_matches;
  drawMatches( input, kp1, cv_ptr->image, kp2, good_matches, img_matches, Scalar::all(-1),
               Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
  //-- Show detected matches
  imshow("Good Matches", img_matches );
  (waitKey(30);

  ros::spin();
  return 0;
}
