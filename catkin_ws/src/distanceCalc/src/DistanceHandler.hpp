//Class for handeling distances between objects and robot
#ifndef DISTANCE_HANDLER_HPP
#define DISTANCE_HANDLER_HPP

#include <ros/ros.h>
// PCL specific includes
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <vector>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>



struct ObjectData{
    float minDistance;
    pcl::PointXYZ minPoint;
    int closestJoint;
} //index

struct ObjectDataList{
    std::vector<ObjectData> list;
    int closestObject; //index
}
    

struct ObjectsAndRobot{
    clustering::clusterArray objects;
    clustering::clusterArray robot;
};

//template <>
class DistanceHandler{
    private:
        cv::Mat rotationMatrix;
        cv::Mat tvec;
        ros::Subscriber calibrationSubscriber, robotSubscriber, clusteringSubscriber;
        ros::Publisher cylinderPublisher, pointCloudPublisher;
        clustering::clusterArray rawClusters;
        clustering::clusterArray objectClusters;
        clustering::clusterArray robotClusters;
        float insideRobotParameter;
        visualization_msgs::Marker cylinder;
        ObjectDataList objects;
        //variables from distanceCalculator
        float minDistance;
        pcl::PointXYZ point;
        pcl::PointXYZ p;
        std::vector<cv::Mat_ <float> > robotJoint;
        std::vector<float> sqrLengthBetweenJoints;
        std::vector<float> radiusOfCylinders;
        pcl::PointXYZ minPoint;
        //pcl::PointXYZ closestJoint;
        int closestJoint;
        //float distance;
        //std_msgs::Float32MultiArray returnArray;
        int numberOfClusters;
        pcl::PointCloud<pcl::PointXYZ>::Ptr publishedPointCloud;

    public:
        DistanceHandler(ros::NodeHandle& nh) : tvec(3,1,cv::DataType<float>::type), 
        publishedPointCloud(new pcl::PointCloud<pcl::PointXYZ>())   {
            calibrationSubscriber = nh.subscribe("calibration_data", 1, &DistanceHandler::calibrationCallback, this);
            //TODO add robotSubsriber
            clusteringSubscriber = nh.subscribe("cluster_vectors", 1,  &DistanceHandler::distanceCallback, this);
            distancePublisher = nh.advertise<std_msgs::Float32MultiArray>("distances", 1);
            cylinderPublisher = nh.advertise<visualization_msgs::Marker>("robot_cylinder", 0);
            pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("objects",1);
            //only for testing
            cv::Mat test;
            test = (cv::Mat_<float>(3,1) <<1.00f,0.0f,0.0f);
            robotJoint.push_back(test);
            test =(cv::Mat_<float>(3,1) <<1.00f,0.0f,2.0f);
            robotJoint.push_back(test);
            sqrLengthBetweenJoints.push_back(1.0f);
            radiusOfCylinders.push_back(0.01f);
            insideRobotParameter = 0.4;

            cylinder.header.frame_id = "/camera_depth_frame";
            cylinder.header.stamp=ros::Time();
            cylinder.id = 0;
            cylinder.type = visualization_msgs::Marker::CYLINDER;
            cylinder.action = visualization_msgs::Marker::ADD;

        }
        void calibrationCallback(const std_msgs::Float64MultiArray& msg){
            cv::Mat rvec(3,1,cv::DataType<float>::type);
            std::cout << "Calibrating" << std::endl;
            for (int i = 0; i <3; ++i){
                rvec.at<float>(i,0) = msg.data[i];
                tvec.at<float>(i,0) = msg.data[i+3];
            }
            cv::Rodrigues(rvec,rotationMatrix);
        }

        ///Functions regarding calculation of distances
        void distanceCallback(clustering::clusterArray msg){
            std::cout << "received clusters" << std::endl;
            rawClusters = msg;
            ObjectsAndRobot result = removeRobot(msg); 
            objectClusters = result.objects;
            robotClusters = result.robot;
            distanceCalc(objectClusters);
            publishRobot();
            publishPointCloud(objectClusters);
        }

        void publishPointCloud(clustering::clusterArray& clusters){
            std::vector<pcl::PointXYZ> tempObjectVector;
            publishedPointCloud->clear();
            publishedPointCloud->header.frame_id= "/camera_depth_frame";
            for(int i=0; i<clusters.ca.size();++i){
                for (int k= 0; k<clusters.ca[i].pa.size();++k){
                    tempObjectVector.push_back(pcl::PointXYZ(clusters.ca[i].pa[k].x,clusters.ca[i].pa[k].y,clusters.ca[i].pa[k].z));
                } 
            
            }           
                publishedPointCloud->insert(publishedPointCloud->begin(),tempObjectVector.begin(),tempObjectVector.end());
                pointCloudPublisher.publish(publishedPointCloud);
                std::cout << "size of pointCloud objects: " << tempObjectVector.size() << std::endl; 
        }
            
        void publishRobot(){
            if (!rotationMatrix.empty()){
                cv::Mat joint1 = rotationMatrix*robotJoint[0]+tvec;
                cv::Mat joint2 = rotationMatrix*robotJoint[1]+tvec;
                cylinder.pose.position.x=joint1.at<float>(0,0);
                cylinder.pose.position.y=joint1.at<float>(1,0);
                cylinder.pose.position.z=joint1.at<float>(2,0);
                float diffX =joint2.at<float>(0,0)-joint1.at<float>(0,0);
                float diffY =joint2.at<float>(1,0)-joint1.at<float>(1,0);
                float diffZ =joint2.at<float>(2,0)-joint1.at<float>(2,0);
                cylinder.pose.orientation.x =diffX;
                cylinder.pose.orientation.y =diffY;
                cylinder.pose.orientation.z =diffZ;
                //cylinder.pose.orientation.w = 10.0f;
                cylinder.scale.x=1.0;
                cylinder.scale.y=1.0;
                cylinder.scale.z=1.0;
                cylinder.color.a=0.1;
                cylinder.color.g=1.0;
                cylinder.color.b=0.0;
                cylinder.color.r=0.0;
                cylinderPublisher.publish(cylinder);
        
            }
        }
        
        void compareToRobot(float x, float y, float z){
            for(int i = 0; i < robotJoint.size(); i++){
                cv::Mat jointInKinectCoord = rotationMatrix*robotJoint[i]+tvec;
                float distance = powf(jointInKinectCoord.at<float>(0,0)-x, 2)+powf(jointInKinectCoord.at<float>(1,0)-y, 2)+powf(jointInKinectCoord.at<float>(2,0)-z, 2);
                if (distance <= minDistance)
                {
                    minDistance = distance;
                    minPoint.x = x;
                    minPoint.y = y;
                    minPoint.z = z;
                    closestJoint=i;
               }
            }
        }

        void distanceCalc(clustering::clusterArray clusters){
            numberOfClusters = clusters.ca.size();
            for(int j = 0; j<numberOfClusters;j++){
                minDistance = 10000.0f;
                for(int i = 0; i<clusters.ca[j].pa.size(); i++){
                    compareToRobot(clusters.ca[j].pa[i].x, clusters.ca[j].pa[i].y, clusters.ca[j].pa[i].z);
                }
                ObjectData data {minDistance, minPoint, closestJoint};
            }
        }

        //Functions regarding removal of the robot!
        ObjectsAndRobot removeRobot(clustering::clusterArray& rawClusterCloud){
            clustering::clusterArray objectArray;
            clustering::clusterArray robotArray;

            int numberOfClusters=0;
            int removedClusters = 0;
            for(int i=0; i<rawClusterCloud.ca.size(); i++)
            {
                ++numberOfClusters;
                float inside = 0;
                float outside = 0;
                for(int j=0; j<rawClusterCloud.ca[i].pa.size(); j++)
                {
                    if(pointInsideRobot(rawClusterCloud.ca[i].pa[j]))
                        ++inside;
                    else
                        ++outside;
                }
                std::cout << "***** rMatrixInfoSTART *****" << std::endl;
                std::cout << "Joint0: "<< rotationMatrix*robotJoint[0]+tvec << std::endl;
                std::cout << "Joint1: "<< rotationMatrix*robotJoint[1]+tvec << std::endl;
                std::cout << "***** rMatrixInfoEND *****" << std::endl;

                if((inside+outside != 0)&&!(inside/(inside+outside) > insideRobotParameter) ){
                    objectArray.ca.push_back(rawClusterCloud.ca[i]); // add cluster if not inside robot

                } else{
                    robotArray.ca.push_back(rawClusterCloud.ca[i]);// add cluster if inside robot
                    ++removedClusters;
                }

                std::cout << "inside: "<< inside <<std::endl;
                std::cout << "outside: "<< outside <<std::endl;


            }

            std::cout << "numberofclusters: " << numberOfClusters <<std::endl;
            std::cout << "removedClusters: " << removedClusters <<std::endl;

            return ObjectsAndRobot{objectArray, robotArray};

        }

        bool pointInsideRobot(clustering::point p){
            for(int i=0; i<robotJoint.size()-1; i++){
                if (!rotationMatrix.empty()){
                    cv::Mat joint1KinectCoord = rotationMatrix*robotJoint[i] + tvec;
                    cv::Mat joint2KinectCoord = rotationMatrix*robotJoint[i+1] + tvec;

                    if (pointInsideCylinder(joint1KinectCoord,joint2KinectCoord,sqrLengthBetweenJoints[i],radiusOfCylinders[i],p))
                        return true;
                } // rotationMatrix*robotJoint[i] +tvec,rotationMatrix*robotJoint[i+1] +tvec,sqrLengthBetweenJoints[i],radiusOfCylinders[i],p
                else if (pointInsideCylinder(robotJoint[i],robotJoint[i+1],sqrLengthBetweenJoints[i],radiusOfCylinders[i],p)){
                    std::cout << "Warning!, Calibration is not working" << std::endl;
                    return true;
                }
            }
            return false;
        }

        //Algorithm copied from http://www.flipcode.com/archives/Fast_Point-In-Cylinder_Test.shtml
        bool pointInsideCylinder( cv::Mat& pt1, cv::Mat& pt2, float lengthsq, float radius_sq, const clustering::point& testpt )
        {
            float dx, dy, dz;	// vector d  from line segment point 1 to point 2
            float pdx, pdy, pdz;	// vector pd from point 1 to test point
            float dot, dsq;

            dx = pt2.at<float>(0,0) - pt1.at<float>(0,0);	// translate so pt1 is origin.  Make vector from
            dy = pt2.at<float>(1,0) - pt1.at<float>(1,0);     // pt1 to pt2.  Need for this is easily eliminated
            dz = pt2.at<float>(2,0) - pt1.at<float>(2,0);

            pdx = testpt.x - pt1.at<float>(0,0);		// vector from pt1 to test point.
            pdy = testpt.y - pt1.at<float>(1,0);
            pdz = testpt.z - pt1.at<float>(2,0);


            dot = pdx * dx + pdy * dy + pdz * dz;


            if( dot < 0.0f || dot > lengthsq )
            {
                //std::cout << "beside" << std::endl;
                return(false);
            }
            else
            {

                dsq = (pdx*pdx + pdy*pdy + pdz*pdz) - dot*dot/lengthsq;

                if( dsq > radius_sq )
                {
                    //std::cout << "outside" << std::endl;
                    return(false);
                }
                else
                {
                    //std::cout << "inside" << std::endl;
                    return(true);		// return true if inside cylinder
                }
            }
        }
};


#endif
