#include <ros/ros.h>
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
#include <pcl-1.6/pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl-1.6/pcl/point_cloud.h>
#include <pcl-1.6/pcl/point_types.h>
#include <pcl-1.6/pcl/common/norms.h>
#include <math.h>
#include <std_msgs/Float32MultiArray.h>
#include <distanceCalc/distanceMessage.h>
#include <clustering/point.h>
#include <clustering/clusterArray.h>
#include <clustering/pointArray.h>
using namespace std;

float minDistance;
pcl::PointXYZ point;
pcl::PointXYZ p;
vector<pcl::PointXYZ> robotJoint(new pcl::PointXYZ(10,10,10),new pcl::PointXYZ(10,10,100));
pcl::PointXYZ minPoint;
pcl::PointXYZ closestJoint;
float distance;
std_msgs::Float32MultiArray returnArray;
int numberOfClusters;
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher chatter_pub;


void compareToRobot(float x, float y, float z){


	for(int i = 0; i < robotJoint.size(); i++){
		distance = powf(robotJoint[i].x-x, 2)+powf(robotJoint[i].y-y, 2)+powf(robotJoint[i].z-z, 2);
		if (distance <= minDistance)
		{
			minDistance = distance;
			minPoint.x = x;
			minPoint.y = y;
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

	for(int j = 0; j<clusters.ca.size();j++){
		minDistance = 10000.0f;
		for(int i = 0; i<clusters.ca[j].pa.size(); i++){
			compareToRobot(clusters.ca[j].pa[i].x, clusters.ca[j].pa[i].y, clusters.ca[j].pa[i].z);
		}
		returnArray.data[j][0] = minPoint.x;
		returnArray.data[j][1] = minPoint.x;
		returnArray.data[j][2] = minPoint.x;
		returnArray.data[j][3] = closestJoint.x;
		returnArray.data[j][4] = closestJoint.y;
		returnArray.data[j][5] = closestJoint.z;
		returnArray.data[j][6] = minDistance;
	}
	chatter_pub.publish(returnArray);
}
int main (int argc, char** argv)
{
	ros::init (argc, argv, "distanceCalc");
	sub = nh.subscribe ("cluster_cloud", 1, distanceCalc);
	chatter_pub = nh.advertise<std_msgs::Float32MultiArray>("distances", 1);
	ros::spin ();
}
