#!/usr/bin/env python

import roslib
import rospy

from sensor_msgs.msg import PointCloud2

import sys, select, termios, tty

msg = """
Subscribe pointcloud from kinect and calculate x-y-z position of each pixel
"""

void clProcess(const sensor_msgs::PointCloud2ConstPtr& ptCloud){
	/* This is the point cloud callback function. When a new cloud is received, this 
	   function will be called automatically by the program. This program will then 
	   downsample the cloud, then convert it to XYZ format, for usability. The cloud
	   then be processed for detection tasks.
	*/

	// Print some information, to see if this code works
	ROS_INFO("SANJEEV -> NEW CLOUD RECEIVED");

	// Declare the variable storing the processed cloud
	sensor_msgs::PointCloud2 ptCloudGrid; // Downsampled point cloud

	// Define the grids, to downsample. Probably 5cm would be okay, FOV = 5m
	pcl::VoxelGrid<sensor_msgs::PointCloud2> vxGrid;
	vxGrid.setInputCloud(ptCloud);
	vxGrid.setLeafSize (0.01, 0.01, 0.01); //1x1x1 cm^3
	vxGrid.filter(ptCloudGrid);

	// Declare the variable storing the processed cloud
	pcl::PointCloud<pcl::PointXYZ> ptCloudPrXYZ_; // Processed Point CloudXYZ
	pcl::fromROSMsg(ptCloudGrid, ptCloudPrXYZ_);  // Convert Downsampled CLoud2 to XYZ

	// Publish the filtered cloud to the WMA Kinect Checker
	pub.publish(ptCloudPrXYZ_.makeShared()); // Make Shared to make it a pointer (for XYZ)
}


def pcCallback(msg):
    msg
    mc
    return key
speed = .4
turn = 0.4
def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
   
    pub = rospy.Publisher('/cmd_vel', Twist)
    rospy.init_node('wifibot_teleop')
    x = 0
    th = 0
    status = 0
    try:
        print msg
        print vels(speed,turn)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print vels(speed,turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            else:
                x = 0
                th = 0
                if (key == '\x03'):
                    break
            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub.publish(twist)
    except:
        print e
    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


