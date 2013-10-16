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
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl-1.6/pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl-1.6/pcl/point_cloud.h>
#include <pcl-1.6/pcl/point_types.h>
#include <pcl-1.6/pcl/common/norms.h>
#include <math.h>
using namespace std;

ros::Publisher chatter_pub;
ros::NodeHandle nh;
ros::Subscriber sub;
pcl::PointXYZ p;
pcl::PointXYZ p2;
pcl::PointXYZ minPoint;
float distance;
float minDistance= 10000;
pcl::PointXYZ closestJoint;
vector<pcl::PointXYZ> robotJoint(new pcl::PointXYZ(10,10,10),new pcl::PointXYZ(10,10,100));

vector<pcl::PointXYZ> compareToRobot(pcl::PointXYZ ps){
	float minDistance = 10000.0f;

	for(int i = 0; i < robotJoint.size(); i++){
		distance = powf(robotJoint[i].x-ps.x, 2)+powf(robotJoint[i].y-ps.y, 2)+powf(robotJoint[i].z-ps.z, 2);
		if (distance <= minDistance)
		{
			minDistance = distance;
			minPoint = ps;
			closestJoint = robotJoint[i];
		}

	}
}
void distanceCalc(vector<vector<pcl::PointXYZ> > vec){

	for(int j = 0; j<vec.size();j++){
		for(int i = 0; i<vec[j].size();i++){
			p.x = vec[j][i].x;
			p.y = vec[j][i].y;
			p.z = vec[j][i].z;
			distance = compareToRobot(p);
		}

	}
}
int main (int argc, char** argv)
{
	ros::init (argc, argv, "distanceCalc");
	sub = nh.subscribe ("cluster_cloud", 1,"distanceCalc");
	chatter_pub = nh.advertise<sensor_msgs::PointCloud2>("distances", 1);
	ros::spin ();
}
