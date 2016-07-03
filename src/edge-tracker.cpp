#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"

#include "visp_bridge/image.h"
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpMeLine.h>

vpImage<unsigned char> I;
vpDisplayX * display;

vpMe me; // moving edges

vpMeLine xLine1;

bool initialized = false;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO("Image Recevied, %d %d %d", msg->header.seq, msg->width, msg->height);
  I  = visp_bridge::toVispImage(*msg);
  
  // hardcoded image frame for now
#if 0
  if(msg->header.seq == 6186)
  {
    vpImagePoint p1(438, 76);
    vpImagePoint p2(322, 145);
    xLine1.initTracking(I, p1, p2);
    ROS_INFO("Tracking started");
    initialized = true;
  }
#endif
  if(msg->header.seq == 6239)
  {
    vpImagePoint p1(614, 613);
    vpImagePoint p2(690, 711);
    xLine1.initTracking(I, p1, p2);
    ROS_INFO("Tracking started");
    initialized = true;
  }
  
  vpDisplay::display(I);
  if(initialized)
  {
    xLine1.track(I);
    xLine1.display(I, vpColor::red);
  }
  
  vpDisplay::flush(I);
  
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "edgetracker");
  ros::NodeHandle n;
  
  I.init(480, 640);
  
  ros::Subscriber sub = n.subscribe("image_raw", 100, imageCallback);
  
  display = new vpDisplayX(I, 0, 0, "Image");
  vpDisplay::display(I);
  vpDisplay::flush(I);
  
  // initialize moving edges
  me.setRange(25);
  me.setThreshold(15000);
  me.setSampleStep(10);
  
  xLine1.setMe(&me);
  xLine1.setDisplay(vpMeSite::RANGE_RESULT);
  
  ros::spin();
}

