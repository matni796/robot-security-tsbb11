#include <ros/ros.h>
// PCL specific includes
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stereo_msgs/DisparityImage.h>
//PointCloud.h>
//#include <stereo_msgs/PointCload2.h>
using namespace std;

ros::Publisher pub;

namespace enc = sensor_msgs::image_encodings;

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, enc::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cout << "i am receiving something" << endl;
  cv::namedWindow("benjamin!", CV_WINDOW_AUTOSIZE);
  cv::imshow("benjamin!", cv_ptr->image);
  cv::waitKey(10);
  //image_pub_.publish(cv_ptr->toImageMsg());
}

void imageCb2(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat outputPic;
  try
  {
    	cv_ptr = cv_bridge::toCvCopy(msg, enc::TYPE_32FC1);
	cv::Scalar meanPicture = cv::max(cv_ptr->img);
	cv::
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cout << "i am receiving something 2" << endl;
  cv::namedWindow("benjamin2!", CV_WINDOW_AUTOSIZE);
  cv::imshow("benjamin2!", cv_ptr->image);
  cv::waitKey(10);
  //image_pub_.publish(cv_ptr->toImageMsg());
}
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/ir/image_raw", 10, imageCb);
  ros::Subscriber sub2= nh.subscribe ("/camera/depth/image", 10, imageCb2);
  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
