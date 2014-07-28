/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Non-A Team, INRIA Lille / EC-LILLE, France.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*  
* Author: Zhaopeng QIU
*
* TODO: for the moment we track only a red object, in the future we expect
*       more capabilities such like contour tracking 
*********************************************************************/
#include <nona_object_detect/nona_localize_object.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <assert.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

static unsigned int index_x, index_y;
static bool is_index_obtained = false;

static boost::mutex g_m;


CvTracks tracks;
tf::Vector3 object_point_camera;
geometry_msgs::Vector3 object_point_world;


void cl_conv(const sensor_msgs::PointCloud2ConstPtr& input);
void blob_track(IplImage* input_image);

inline unsigned int limit_between(int input, const int min_lim, const int max_lim)
{
  unsigned int temp = (unsigned int) ((input < min_lim)? min_lim : input) ;
  return   (unsigned int) (( temp >max_lim) ? max_lim : temp) ;
};

int main (int argc, char** argv){

	ros::init (argc, argv, "nona_localize_object_node"); 
	ros::NodeHandle kinect_nh, nh_;

	ros::Subscriber kinectSub_ = kinect_nh.subscribe ("/camera/depth/points", 1, cl_conv);

  tf::TransformListener listener;

  assert (limit_between(-10, 0, 479) == 0); 
  assert (limit_between(0, 0, 479) == 0); 
  assert (limit_between(100, 0, 479) == 100); 
  assert (limit_between(600, 0, 479) == 479); 


  ImageConverter ic;
  cvNamedWindow("NonA_red_tracking", CV_WINDOW_AUTOSIZE);
  IplImage* current_image = NULL; 
  static IplImage img;

	ros::Rate r(10); 
  ros::Publisher pub = kinect_nh.advertise<geometry_msgs::Vector3> ("/detected_object_position", 1);
  tf::StampedTransform transform;
  int i = 0;
  while (ros::ok())
  {
      try{
            listener.lookupTransform( "/world", "/camera_depth_optical_frame", ros::Time(0), transform);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }

      tf::Vector3 object_in_world = transform * object_point_camera;
      object_point_world.x = object_in_world.getX();
      object_point_world.y = object_in_world.getY();
      object_point_world.z = object_in_world.getZ();
      pub.publish(object_point_world);

      ros::spinOnce();  
      if (ic.initialized && i > 10)
      {    
           img = ic.image;
           current_image  = cvCloneImage(&img);
           if (i == 11)
                ROS_INFO("tracking starts!");
          blob_track(current_image);
      }
    
      r.sleep();
      i++;
  }
  return 0;
}

void cl_conv(const sensor_msgs::PointCloud2ConstPtr& input){

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  unsigned ind, ind_chosen ; 
  int x_f, y_f; 
  double minz = 100;

  boost::unique_lock<boost::mutex> lock(g_m);
  int ind_x = (int) index_x;
  int ind_y = (int) index_y;
  lock.unlock();
  unsigned int lim1 = 640;

  assert (ind_x  < 640); 
  assert (ind_y  < 480); 


  if (is_index_obtained)
  {
    for (int i =-40; i < 40; i+=2)
      for (int k =-40; k < 40; k+=2)
         {
            x_f = ind_x + k;
            y_f = ind_y + i;

            ind = limit_between(y_f , 0, 479)*lim1 +  limit_between(x_f, 0, 639);
            ind_chosen = ind;
            double zdis;
            if (std::isnan(cloud.points[ind].z))
                zdis = 1000.0;
            else 
                zdis = cloud.points[ind].z;

            if (zdis < minz)
               { minz = zdis;
                  ind_chosen = ind;
                  // std::cout << "ind = " << ind  << std::endl; 
                }
          }

    if (!std::isnan(cloud.points[ind].x))
      {object_point_camera.setX(cloud.points[ind_chosen].x);
      object_point_camera.setY(cloud.points[ind_chosen].y);
      object_point_camera.setZ(cloud.points[ind_chosen].z);}
  }
}


void blob_track(IplImage* input_image){

      CvSize imgSize = cvGetSize(input_image);

      IplImage *frame = cvCreateImage(imgSize, input_image->depth, input_image->nChannels);
      IplConvKernel* morphKernel = cvCreateStructuringElementEx(5, 5, 1, 1, CV_SHAPE_RECT, NULL);

      unsigned int blobNumber = 0;
      bool quit = false;

      cvConvertScale(input_image, frame, 1, 0);

      IplImage *segmentated = cvCreateImage(imgSize, 8, 1);

      for (unsigned int j=0; j<imgSize.height; j++)
        for (unsigned int i=0; i<imgSize.width; i++)
        {
          CvScalar c = cvGet2D(frame, j, i);

          double b = ((double)c.val[0])/255.;
          double g = ((double)c.val[1])/255.;
          double r = ((double)c.val[2])/255.;
          unsigned char f = 255*((r>0.2+g)&&(r>0.2+b));

          cvSet2D(segmentated, j, i, CV_RGB(f, f, f));
        }

      cvMorphologyEx(segmentated, segmentated, NULL, morphKernel, CV_MOP_OPEN, 1);
      IplImage *labelImg = cvCreateImage(cvGetSize(frame), IPL_DEPTH_LABEL, 1);

      CvBlobs blobs;
      unsigned int result = cvLabel(segmentated, labelImg, blobs);
      cvFilterByArea(blobs, 50, 1000000);
      cvRenderBlobs(labelImg, blobs, frame, frame, CV_BLOB_RENDER_BOUNDING_BOX);
      cvUpdateTracks(blobs, tracks, 200., 5);
      cvRenderTracks(tracks, frame, frame, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);

      cvShowImage("NonA_red_tracking", frame);

      is_index_obtained = false; 
      if (blobs.size() > 0)
      {
        CvBlobs::iterator it = blobs.begin();
        CvBlob *blob=(*it).second;
        
        boost::unique_lock<boost::mutex> lock(g_m);
        index_x = (unsigned int)(0.5*((*blob).minx + (*blob).maxx));
        index_y = (unsigned int) (0.5*((*blob).miny + (*blob).maxy));
        lock.unlock();

        if (index_x  < 640 && index_y  < 480)
            is_index_obtained = true;
      }
      cvReleaseImage(&labelImg);
      cvReleaseImage(&segmentated);
      cvReleaseBlobs(blobs);

}

