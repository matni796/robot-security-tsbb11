//#define HYDRO

#include <ros/ros.h>
#include <time.h>
// PCL specific includes
//#include <pcl-1.6/pcl/point_cloud.h>
#ifdef HYDRO
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#else
#include <pcl-1.6/pcl/point_cloud.h>
#include <pcl-1.6/pcl/ros/conversions.h>
#include <pcl-1.6/pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#endif

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

cv::Size downsampleSize;
cv::Mat back, fore, frame, depth, normImg;
cv::Mat buffer;

// Elements from projection matrix needed for PC construction
float fx, fy, cx, cy;
const float normFact = 255.0f/6.0f;		// Normalization factor

//Parameters for backgroundmodelling
int history = 20000;
float varThreashold = 4.0f;
int nMixtures = 5;

pcl::PointCloud<pcl::PointXYZ>::Ptr publishedCloud;

//pcl::visualization::CloudViewer viewer("Cloud viewer");
std::vector<pcl::PointXYZ> cloud;				// Point cloud
namespace enc = sensor_msgs::image_encodings;

std::string getEnvVar( std::string const & key ) {
  char * val;
  val = getenv( key.c_str() );
  std::string retval = "";
  if (val != NULL) {
    retval = val;
  } else {
	  cerr << "Warning! Environmentvariable " << key << " is not set!";
  }
  return retval;
}


float getWorldCoord(float f, float c, float zWorld, int screenCoord)
{
	return (screenCoord - c)*zWorld/f;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	//time_t start, stop;
	clock_t start, stop;  	


  	start=clock();  /* get current time; same as: timer = time(NULL)  */

	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, "");
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv::resize(cv_ptr->image, depth, downsampleSize, 0, 0, cv::INTER_NEAREST);
	depth.copyTo(buffer);
	buffer = buffer * normFact;         // Normalize image so [0 m, 6 m [  is in [ 0, 256 [
	buffer.convertTo(normImg, CV_8UC1); // Convert to uint8

	channels.clear(); // clear channel buffer
	channels.push_back(normImg);
	channels.push_back(normImg);
	channels.push_back(normImg);
	cv::merge(channels,frame); // merge three identical channels into one
	
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

	for(int u = 0; u < fore.cols; u++) {
		for(int v = 0; v < fore.rows; v++) {
			if(fore.at<char>(v,u) != 0) {
				z = depth.at<float>(v,u);
				if(z == z) { // NaN check
					x = getWorldCoord(fx,cx,z,u);
					y = getWorldCoord(fy,cy,z,v);
					publishedCloud->push_back(pcl::PointXYZ(x,y,z));
				}
			}
		}
	}

	// Publish point cloud
		
	publishedCloud->header.frame_id = "/camera_depth_optical_frame";
	publishedCloud->insert(publishedCloud->begin(), cloud.begin(), cloud.end());
	ROS_INFO("Publishing %dx%d point cloud to foreground_cloud", publishedCloud->width, publishedCloud->height);
	pub.publish(publishedCloud);
	publishedCloud->clear();
	stop = clock();
	float time =((float)stop-(float)start)/1000.0f;
	ROS_INFO("Background modelling on %dx%d", downsampleSize.width, downsampleSize.height);
	ROS_INFO("One iteration timed at %.3f ms (should be < %.3f ms)", time, 1000/30.0f);
}

int main (int argc, char** argv)
{
	// Import camera intrinsic
	const string path = getEnvVar("CAMERA_MATRIX_PATH")+"/camMat.yaml";
	string cameraName;
	sensor_msgs::CameraInfo camInfo;
	camera_calibration_parsers::readCalibration(path, cameraName, camInfo);
    
    publishedCloud=pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());


	fx = camInfo.P.elems[0];
	cx = camInfo.P.elems[2];
	fy = camInfo.P.elems[5];
	cy = camInfo.P.elems[6];

	float sx = 0.2f, sy = 0.2f * 4.0f / 3.0f;

	downsampleSize.width = 640*sx;
	downsampleSize.height = 480*sy;
	
	cx *= sx;
	cy *= sy;
	fx *= sx;
	fy *= sy;
	
	// Initialize ROS and openCV windows
	cv::namedWindow("Frame");
	cv::namedWindow("Background");
	cv::namedWindow("Foreground");
	ros::init (argc, argv, "background_modelling");
	ros::NodeHandle nh;
	bg = new cv::BackgroundSubtractorMOG2(history, varThreashold, false);
	bg->set("nmixtures", nMixtures);

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe("/camera/depth/image", 1, imageCb);
	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2>("foreground_cloud",1);

	// Spin
	ros::spin ();
}
