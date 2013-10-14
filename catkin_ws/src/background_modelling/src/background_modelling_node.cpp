#include <ros/ros.h>
// PCL specific includes
#include <pcl-1.6/pcl/point_cloud.h>
#include <pcl-1.6/pcl/ros/conversions.h>
#include <pcl-1.6/pcl/point_types.h>
#include <pcl-1.6/pcl/visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>
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

using namespace std;


ros::Publisher pub;

//openCV stuff

cv_bridge::CvImagePtr cv_ptr;
cv::BackgroundSubtractorMOG2 * bg;	
std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Mat> channels;


// Elements from projection matrix needed for PC construction
float fx, fy, cx, cy;
//pcl::visualization::CloudViewer viewer("Clod viewer");
std::vector<pcl::PointXYZ> cloud;
namespace enc = sensor_msgs::image_encodings;

float getWorldCoord(float f, float c, float zWorld, int screenCoord)
{
	return (screenCoord - c)*zWorld/f;
}


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
	//cout <<"nanCOunt: " << nanCount << "number of pixels "<< 640*480   << endl;
	channels.clear();
	cv::Mat depth = cv_ptr->image;
	channels.push_back(cv_ptr->image);
	channels.push_back(cv_ptr->image);
	channels.push_back(cv_ptr->image);

	cv::merge(channels,frame);
	bg->operator()(frame, fore, -1);
	bg->getBackgroundImage(back);
	cv::erode(fore,fore,cv::Mat());
        cv::dilate(fore,fore,cv::Mat());
        //cv::findContours(fore,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
        //cv::drawContours(frame,contours,-1,cv::Scalar(0,0,255),2);
        cv::imshow("Foreground", fore);
	cv::imshow("Frame",frame/6.0f);
        cv::imshow("Background",back*50);
        cv::waitKey(10);
        cloud.clear();
        float x,y,z;

       for(int u = 0; u < fore.cols; u++)
       {
    	   for(int v = 0; v < fore.rows; v++)
    	   {

    		   if(fore.at<char>(v,u) != 0)
    		   {
    			   z = depth.at<float>(v,u);
    			   x = getWorldCoord(fx,cx,z,u);
    			   y = getWorldCoord(fy,cy,z,v);
    			   cloud.push_back(pcl::PointXYZ(x,y,z));
    		   }
    	   }

       }
	
	

       // Visualization of PC
       
       	pcl::PointCloud<pcl::PointXYZ>::Ptr myCloud(new pcl::PointCloud<pcl::PointXYZ>());
      	myCloud->insert(myCloud->begin(), cloud.begin(), cloud.end());
       	//sensor_msgs::PointCloud2 msg;
	
	//pcl::toROSMsg(myCloud, msg);
	pub.publish(myCloud);
	//viewer.showCloud(myCloud);
       	myCloud->clear();
       
}

int main (int argc, char** argv)
{
	// Import camera intrinsic
	const string path = "/home/matt/dev/robot-security-tsbb11/catkin_ws/camMat.yaml";
	string cameraName;
	sensor_msgs::CameraInfo camInfo;
	camera_calibration_parsers::readCalibration(path, cameraName, camInfo);


	fx = camInfo.P.elems[0];
	cx = camInfo.P.elems[2];
	fy = camInfo.P.elems[5];
	cy = camInfo.P.elems[6];

  // Initialize ROS and openCV windows
 	cv::namedWindow("Frame");
	cv::namedWindow("Background");
	cv::namedWindow("Foreground");
	ros::init (argc, argv, "testKinect");
  	ros::NodeHandle nh;
	bg = new cv::BackgroundSubtractorMOG2(1000, 0.040f, false);
	bg->set("nmixtures", 10);

	//bg->set("nframes", 1000);
	//bg.set("bShadowDetection", false);//bg.nmixtures = 3;
	//bg.bShadowDetection = false;
	//bg.nchannels = 1;
  	// Create a ROS subscriber for the input point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2>("foreground_cloud",1);
  	ros::Subscriber sub = nh.subscribe("/camera/depth/image", 1, imageCb);
  	// Create a ROS publisher for the output point cloud
  	//pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  	// Spin
  	ros::spin ();
}
