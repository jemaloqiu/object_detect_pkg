
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>

#if (defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__) || defined(__WINDOWS__) || (defined(__APPLE__) & defined(__MACH__)))
#include <cv.h>
#include <highgui.h>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

#include <opencv2/core/core.hpp>
// for img processing 
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>
// for blob detect
#include "cvblob.h"
#include <map>
#include <string>


// author: Zhaopeng QIU 

using namespace cvb;

static const std::string OPENCV_WINDOW = "NonA_red_tracking_ros";

class ImageConverter
{

  ros::NodeHandle nh_;
  std::string sub_image_name;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  

public:
      cv_bridge::CvImagePtr cv_ptr;
      IplImage* current_image_, depth_image;
      cv::Mat image ;
      bool initialized;
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed

    nh_.param<std::string>("/subscribed_image_topic_name", sub_image_name, "/camera/rgb/image_raw");


    image_sub_ = it_.subscribe(sub_image_name, 1, &ImageConverter::imageCb, this);
    // dimage_sub_ = it_.subscribe("/camera/depth/image_rect", 1, &ImageConverter::dimageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    //cv::namedWindow(OPENCV_WINDOW);
    initialized = false;
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      image = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    if(!initialized)
        initialized = true;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");

  CvTracks tracks;

  ImageConverter ic;
  ros::NodeHandle nh_; 
  cvNamedWindow("NonA_red_tracking_ros", CV_WINDOW_AUTOSIZE);
  IplImage* current_image_ = NULL; 
  static IplImage img;
  ros::Rate r(10); // 10 hz
  int i = 0;
  while (ros::ok())
  {
      ros::spinOnce();  
      if (ic.initialized && i > 10)
      {    img = ic.image;
           current_image_  = cvCloneImage(&img);
           if (i == 11)
                ROS_INFO("tracking starts!");
      
      CvSize imgSize = cvGetSize(current_image_);

      IplImage *frame = cvCreateImage(imgSize, current_image_->depth, current_image_->nChannels);
      IplConvKernel* morphKernel = cvCreateStructuringElementEx(5, 5, 1, 1, CV_SHAPE_RECT, NULL);

      //unsigned int frameNumber = 0;
      unsigned int blobNumber = 0;

      bool quit = false;

      cvConvertScale(current_image_, frame, 1, 0);

      IplImage *segmentated = cvCreateImage(imgSize, 8, 1);
      
      // Detecting red pixels:
      // (This is very slow, use direct access better...)
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

      //cvShowImage("segmentated", segmentated);

      IplImage *labelImg = cvCreateImage(cvGetSize(frame), IPL_DEPTH_LABEL, 1);

      CvBlobs blobs;
      unsigned int result = cvLabel(segmentated, labelImg, blobs);
      cvFilterByArea(blobs, 50, 1000000);
      cvRenderBlobs(labelImg, blobs, frame, frame, CV_BLOB_RENDER_BOUNDING_BOX);
      cvUpdateTracks(blobs, tracks, 200., 5);
      cvRenderTracks(tracks, frame, frame, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);

      cvShowImage("NonA_red_tracking_ros", frame);

       CvBlobs::iterator it=blobs.begin();
      while(it!=blobs.end())
      {
          CvBlob *blob=(*it).second;

          int x = 0.5*((*blob).minx + (*blob).maxx);
          int y = 0.5*((*blob).miny + (*blob).maxy);
          std::cout << "Red object center locates at : [" << x<< ", " << y  << "]" << std::endl;
         // std::cout << "minx is: " << (*blob).minx << "; maxx is: " << (*blob).maxx << "; miny is: " << (*blob).miny << "; maxy is: " << (*blob).maxy <<std::endl;
          ++it;
      }

      cvReleaseImage(&labelImg);
      cvReleaseImage(&segmentated);
      cvReleaseBlobs(blobs);
    }
    
      r.sleep();
        i++;
  }

  return 0;
}
