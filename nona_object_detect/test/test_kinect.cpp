/* C++ Version: May 28, 2014
   This program will generate a node for testing conversions of point cloud topics. 
   One should start kinect node using 
        roslaunch openni_launch openni.launch
   Then run this program:
        rosrun nona_object_detect nona_test_kinect
that subscribes to topic "/camera/depth/points", using ROS, and then
   inputs the cloud, downsamples it, and then publishes the filtered (downsampled) cloud.
   
   Author: Zhaopeng QIU
*/

// ROS Includes
#include "ros/ros.h"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
//C++ and OPEN GL Includes
#include <iostream>

// PCL Iincludes
#include <pcl/filters/voxel_grid.h>

#include <pcl_ros/point_cloud.h> // not pcl/point_cloud.h --- gives error with xyz publish
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>


#ifndef _PTCLIOPUB_H
	#define _PTCLIOPUB_H
#endif

//typedef pcl::PointCloud<pcl::PointXYZ> pClxyz_;
ros::Publisher pub, pub_mc;

/* Function to Downsample the Point Cloud */
void cl_process(const pcl::PCLPointCloud2ConstPtr& input);
void cl_conv(const sensor_msgs::PointCloud2ConstPtr& input);

int main (int argc, char** argv){
	// Initialize ROS
	ros::init (argc, argv, "nona_pcl_kinect_node"); 
	ros::NodeHandle kinectNH_;

	// Subscribe to the kinect's input point cloud
	ros::Subscriber kinectSub_1 = kinectNH_.subscribe ("/camera/depth/points", 1, cl_conv);
	ros::Subscriber kinectSub_2 = kinectNH_.subscribe ("/camera/depth/points", 1, cl_process);
	// Publish the processed cloud
	ros::Rate r(20); //Callbacks at 20Hz
  pub = kinectNH_.advertise<sensor_msgs::PointCloud2> ("output_voxels", 1);

	while (ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
}


void cl_conv(const sensor_msgs::PointCloud2ConstPtr& input){

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  int ind, ind_chosen ; 
  double minz = 100;
  for (int i =-20; i < 20; i+=1)
    for (int k =-20; k < 20; k+=1)
        {
          ind = (250 + i)*640 + 350 + k;
          if (cloud.points[ind].z < minz)
             { minz = cloud.points[ind].z;
                ind_chosen = ind;
              }
          }

  // std::cout << "position is [" << cloud.points[ind_chosen].x << ", "<< cloud.points[ind_chosen].y << ", "<< cloud.points[ind_chosen].z << "] "<< std::endl; 

}


void cl_process(const pcl::PCLPointCloud2ConstPtr& input){

  pcl::PCLPointCloud2 cloud_filtered;
  pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
  vg.setInputCloud (input);
  vg.setLeafSize (0.02, 0.02, 0.02);
  vg.filter(cloud_filtered);

  // Publish the dataSize 
  pub.publish (cloud_filtered);

}
