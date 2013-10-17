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
const float normFact = 255.0f/6.0f;			// Normalization factor
//pcl::visualization::CloudViewer viewer("Cloud viewer");
std::vector<pcl::PointXYZ> cloud;				// Point cloud
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
	cv::Mat back, fore, frame, depth, normImg;

	cv_ptr->image.copyTo(depth);	// Copy depth image to create point cloud and vizualization

	channels.clear();
	cv_ptr->image = cv_ptr->image*normFact;			// Normalize image
	cv_ptr->image.convertTo(normImg, CV_8UC1);		// Convert to uint8

	channels.push_back(normImg);
	channels.push_back(normImg);
	channels.push_back(normImg);

	cv::merge(channels,frame);
	bg->operator()(frame, fore, -1);
	bg->getBackgroundImage(back);

	cv::erode(fore,fore,cv::Mat());
	cv::dilate(fore,fore,cv::Mat());

	cv::imshow("Foreground", fore);
	cv::imshow("Frame",frame);
	cv::imshow("Background",back);
	cv::waitKey(10);
	float x,y,z;
	cloud.clear();

	for(int u = 0; u < fore.cols; u++)
	{
		for(int v = 0; v < fore.rows; v++)
		{
			if(fore.at<char>(v,u) != 0)
			{
				z = depth.at<float>(v,u);
				if(z == z)
				{
					x = getWorldCoord(fx,cx,z,u);
					y = getWorldCoord(fy,cy,z,v);
					cloud.push_back(pcl::PointXYZ(x,y,z));
				}
			}
		}
	}
	cloud.push_back(pcl::PointXYZ(0.0f,0.0f,0.0f));

	// Publish point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr myCloud(new pcl::PointCloud<pcl::PointXYZ>());
	myCloud->insert(myCloud->begin(), cloud.begin(), cloud.end());
	pub.publish(myCloud);
	//viewer.showCloud(myCloud);
	myCloud->clear();

}

int main (int argc, char** argv)
{
	// Import camera intrinsic
	const string path = "/home/niklas/dev/tsbb11/catkin_ws/camMat.yaml";
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
	bg = new cv::BackgroundSubtractorMOG2(20000, 16.0f, false);
	bg->set("nmixtures", 10);

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe("/camera/depth/image", 1, imageCb);
	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2>("foreground_cloud",1);

	// Spin
	ros::spin ();
}
