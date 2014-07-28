#!/usr/bin/env python

""" face_tracker.py - Version 0.22 2012-01-20

    Track a face using the OpenCV Haar detector to initially locate the face, then OpenCV's
    Good-Features-to-Track and Lucas-Kanade Optical Flow to track the face features over 
    subsequent frames.
    
    Can also be used to track arbitrarily selected patches by setting the parameter
    auto_face_tracking to False and using the mouse to select the desired region.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import roslib
import rospy
import cv
import sys
from sensor_msgs.msg import RegionOfInterest, Image
from math import sqrt, isnan
from ros2opencv import ROS2OpenCV
from nona_object_detect.srv import *
import time 

if __name__ == '__main__':
    print "test succeeds"
    time.sleep(2)
    shutdown()
