#include <ros/ros.h>
// PCL specific includes
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stereo_msgs/DisparityImage.h>
#include <vector>

#include "camera_calibration_parsers/parse.h"
#include "camera_calibration_parsers/parse_ini.h"
#include "camera_calibration_parsers/parse_yml.h"
#include <boost/algorithm/string/predicate.hpp>

using namespace std;


ros::Publisher pub;

//openCV stuff

cv_bridge::CvImagePtr cv_ptr;
cv::BackgroundSubtractorMOG2 * bg;	
std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Mat> channels;

namespace enc = sensor_msgs::image_encodings;

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, "");
  	}
  	catch (cv_bridge::Exception& e)
  	{
    		ROS_ERROR("cv_bridge exception: %s", e.what());
    		return;
  	}
	//cv::Mat frame(cv_ptr->image.size().width, cv_ptr->image.size().height, CV_8UC3,cv_ptr->image);
	int nanCount = 0;
  	cv::Mat back, fore, frame;
	for(int i = 0; i<=cv_ptr->image.rows;++i){
		for(int k = 0; k<=cv_ptr->image.cols;++k){	
			if(cv_ptr->image.at<float>(i,k) != cv_ptr->image.at<float>(i,k)){
				cv_ptr->image.at<float>(i,k) = 0;
				++nanCount;
			}
		}
	}
	cout <<"nanCOunt: " << nanCount << "number of pixels "<< 640*480   << endl;
	channels.clear();
	channels.push_back(cv_ptr->image);
	channels.push_back(cv_ptr->image);
	channels.push_back(cv_ptr->image);
	cout << "i am receiving something" << endl;
	cv::merge(channels,frame);
	bg->operator()(frame, fore, -1);  	
	bg->getBackgroundImage(back);
	//cv::erode(fore,fore,cv::Mat());
        //cv::dilate(fore,fore,cv::Mat());
        cv::findContours(fore,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
        cv::drawContours(frame,contours,-1,cv::Scalar(0,0,255),2);
       
	cv::imshow("Foreground", fore);
	cv::imshow("Frame",frame/6);
        cv::imshow("Background",back*50);
        cv::waitKey(10);
  	//image_pub_.publish(cv_ptr->toImageMsg());*/
}
float getWorldCoord(float f, float c, float zWorld, int screenCoord)
{
	return (screenCoord - c)*zWorld/f;
}

/*void imageCb2(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptcamera_matrix:r;
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
}*/
int main (int argc, char** argv)
{
	//cv::FileStorage fs("/home/niklas/.ros/camera_info/depth_A00367A00428051A.yaml", cv::FileStorage::READ);

	int width;
	//fs["image_width:"] >> width;
	//int width = (int)fs["image_width"];
	//fs["camera_matrix"] >> cameraMatrix;
	//cout << width << endl;
	//cout << cameraMatrix << endl;
	const string path = "/home/niklas/.ros/camera_info/depth_A00367A00428051A.yaml";
	string cameraName;
	sensor_msgs::CameraInfo camInfo;
	camera_calibration_parsers::readCalibration(path, cameraName, camInfo);

	float fx, fy, cx, cy;
	fx = camInfo.P.elems[0];
	cx = camInfo.P.elems[2];
	fy = camInfo.P.elems[5];
	cy = camInfo.P.elems[6];
	//camInfo.P

  // Initialize ROS and openCV windows
 	cv::namedWindow("Frame");
	cv::namedWindow("Background");
	cv::namedWindow("Foreground");
	ros::init (argc, argv, "testKinect");
  	ros::NodeHandle nh;
	bg = new cv::BackgroundSubtractorMOG2(10000, 0.040f, false);
	bg->set("nmixtures", 10);
	cout << "olle är kingen!" << endl;

	//bg->set("nframes", 1000);
	//bg.set("bShadowDetection", false);//bg.nmixtures = 3;
	//bg.bShadowDetection = false;
	//bg.nchannels = 1;
  	// Create a ROS subscriber for the input point cloud

  	ros::Subscriber sub = nh.subscribe("/camera/depth/image", 10, imageCb);
	//ros::Subscriber sub2= nh.subscribe ("/camera/depth/image", 10, imageCb2);
  	// Create a ROS publisher for the output point cloud
  	//pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  	// Spin
  	ros::spin ();
}
