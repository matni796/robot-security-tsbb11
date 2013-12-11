#include <ros/ros.h>
//#define HYDRO
// PCL specific includes
#include <memory>
#include <iostream>
#include <sstream>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stereo_msgs/DisparityImage.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#ifndef HYDRO
#include <pcl-1.6/pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl-1.6/pcl/point_cloud.h>
#include <pcl-1.6/pcl/point_types.h>
#include <pcl-1.6/pcl/common/norms.h>
#else
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/norms.h>
#endif
#include <math.h>
#include <std_msgs/Float32MultiArray.h>
#include <clustering/point.h>
#include <clustering/clusterArray.h>
#include <clustering/pointArray.h>
#include "DistanceHandler.hpp"
using namespace std;

int main (int argc, char** argv)
{
	ros::init (argc, argv, "distanceCalc");
	ros::NodeHandle nh;
	DistanceHandler distanceHandler(nh);
	/*while(!nh.ok()){
		distanceHandler.loop();
		ros::spinOnce();
	}*/
	//sub = nh.subscribe ("cluster_vectors", 1, distanceCalc);
	//chatter_pub = nh.advertise<std_msgs::Float32MultiArray>("diances", 1);
	ros::spin ();

}
