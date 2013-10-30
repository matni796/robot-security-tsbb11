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
//#include <PointCloud.h>
//#include <stereo_msgs/PointCload2.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
// PCL specific includes
#include <pcl-1.6/pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl-1.6/pcl/point_cloud.h>
#include <pcl-1.6/pcl/point_types.h>
#include <pcl-1.6/pcl/visualization/cloud_viewer.h>
#include <pcl-1.6/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.6/pcl/segmentation/extract_clusters.h>
#include <pcl-1.6/pcl/kdtree/kdtree.h>
#include <pcl-1.6/pcl/filters/filter.h>
#include <pcl-1.6/pcl/filters/voxel_grid.h>
using namespace std;

namespace enc = sensor_msgs::image_encodings;
ros::Publisher chatter_pub;
//vector<PointCloud2...> temp;
boost::shared_ptr<pcl::visualization::CloudViewer> viewer;

void euclidianClustering(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pc) {
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(pc);
	//TODO: Resize image input (Needed for speedup of calculation..)
	std::vector<pcl::PointXYZRGB> cloud_cluster;
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.05); // 5cm
	ec.setMinClusterSize (800);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(pc);
	ec.extract(cluster_indices);
	int r[] = { 255,   0,   0, 255,   0, 255 };
	int g[] = {   0, 255,   0, 255, 255,   0 };
	int b[] = {   0,   0, 255,   0, 255, 255 };

	int color = 0;
	for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
		//skapa ett punktmoln.
		for(std::vector<int>::const_iterator jt = it->indices.begin(); jt != it->indices.end(); ++jt) {
			pcl::PointXYZRGB p(r[color%6], g[color%6], b[color%6]);
			pcl::PointXYZ p2 = pc->points[*jt];
			p.x = p2.x;
			p.y = p2.y;
			p.z = p2.z;
			//pushback in i punktmoln.
			cloud_cluster.push_back(p);
		}
		//pushback in i vectorn.
		++color;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr myCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	myCloud->insert(myCloud->begin(), cloud_cluster.begin(), cloud_cluster.end());
	viewer->showCloud(myCloud);

	chatter_pub.publish(myCloud);

}
void downsample(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pc,
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered){
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(pc);
	vg.setLeafSize(0.1f, 0.1f, 0.1f);
	vg.filter(*cloud_filtered);
}

void clustering(const sensor_msgs::PointCloud2ConstPtr& input)
{
	printf("Received Point Cloud: %dx%d\n", input->height, input->width);
	//ROS_INFO("Received Point Cloud: %d.", input.height);
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pc(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg<pcl::PointXYZ>(*input,*pc);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	downsample(pc,cloud_filtered);

	if(viewer->wasStopped()) {
		cout << "viewer was stopped\n";
	}
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud_filtered,*cloud_filtered,indices);
	euclidianClustering(cloud_filtered);
}

int main (int argc, char** argv)
{
	ros::init (argc, argv, "clustering");
	viewer.reset(new pcl::visualization::CloudViewer("Simple Cloud Viewer"));

	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe ("foreground_cloud", 1, clustering);
	chatter_pub = nh.advertise<sensor_msgs::PointCloud2>("cluster_cloud", 1);
	ros::spin ();
}
