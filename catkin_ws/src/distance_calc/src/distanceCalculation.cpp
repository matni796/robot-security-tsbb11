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
/*
float minDistance;
pcl::PointXYZ point;
pcl::PointXYZ p;
vector<pcl::PointXYZ> robotJoint(new pcl::PointXYZ(10,10,10),new pcl::PointXYZ(10,10,100));
pcl::PointXYZ minPoint;
pcl::PointXYZ closestJoint;
//float distance;
std_msgs::Float32MultiArray returnArray;
int numberOfClusters;

ros::Subscriber sub;
ros::Publisher chatter_pub;


void compareToRobot(float x, float y, float z){


	for(int i = 0; i < robotJoint.size(); i++){
		float distance = powf(robotJoint[i].x-x, 2)+powf(robotJoint[i].y-y, 2)+powf(robotJoint[i].z-z, 2);
		if (distance <= minDistance)
		{
			minDistance = distance;
			minPoint.x = x;
			minPoint.y = y;st =(cv::Mat_<float>(3,1) <<0.0f,0.5f,1.5f);
			minPoint.z = z;
			closestJoint = robotJoint[i];
		}
	}
}
void distanceCalc(clustering::clusterArray clusters){


	numberOfClusters = clusters.ca.size();
	returnArray.layout.dim.resize(2);
	returnArray.data.resize(numberOfClusters*7);
	returnArray.layout.dim[0].size = 7;
	returnArray.layout.dim[0].stride = 7*numberOfClusters;
	returnArray.layout.dim[1].size = numberOfClusters;
	returnArray.layout.dim[1].stride = 7;
	cout << "Size of clusterArray (outer) " << endl;
	cout << clusters.ca.size() << endl;

	for(int j = 0; j<clusters.ca.size();j++){
		minDistance = 10000.0f;
		for(int i = 0; i<clusters.ca[j].pa.size(); i++){
			compareToRobot(clusters.ca[j].pa[i].x, clusters.ca[j].pa[i].y, clusters.ca[j].pa[i].z);
		}
		returnArray.data[0+j*7] = minPoint.x;
		returnArray.data[1+j*7] = minPoint.x;
		returnArray.data[2+j*7] = minPoint.x;
		returnArray.data[3+j*7] = closestJoint.x;
		returnArray.data[4+j*7] = closestJoint.y;
		returnArray.data[5+j*7] = closestJoint.z;
		returnArray.data[6+j*7] = minDistance;
		//ROS_INFO("%f",minDistance);
		cout << "Printing mindistance:" << endl;
		cout << "MINDISTANCE" << endl;
		cout << returnArray.data[6+j*7] << endl;
		cout << ("%f",minDistance)	<< endl;
	}
	cout << "Publishibng returnArray..." << endl;
	chatter_pub.publish(returnArray);
}*/
int main (int argc, char** argv)
{
	ros::init (argc, argv, "distanceCalc");
	ros::NodeHandle nh;
	DistanceHandler distanceHandler(nh);
	//sub = nh.subscribe ("cluster_vectors", 1, distanceCalc);
	//chatter_pub = nh.advertise<std_msgs::Float32MultiArray>("diances", 1);
	ros::spin ();

}
