#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"

#include "visp_bridge/image.h"
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpMeLine.h>
#include <visp/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp/vpTemplateTrackerWarpHomography.h>
#include <visp/vpTrackingException.h>

vpImage<unsigned char> I;
vpDisplayX * display;

// template tracker variables
vpTemplateTrackerWarpHomography * warp;
vpTemplateTrackerSSDInverseCompositional * tracker;
bool initialized;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO("Image Recevied, %d %d %d", msg->header.seq, msg->width, msg->height);
  I  = visp_bridge::toVispImage(*msg);
  
  if(msg->header.seq == 5844 && !initialized)
  {
    ROS_INFO("Reached hardcoded frame sequence ID");
    vpTemplateTrackerZone zone;
    vpTemplateTrackerTriangle t1(vpImagePoint(387, 346), 
				 vpImagePoint(369, 432),
				 vpImagePoint(484, 454)
			      );
    vpTemplateTrackerTriangle t2(vpImagePoint(484, 454), 
				 vpImagePoint(499, 369),
				 vpImagePoint(387, 346)
			      );
    vpTemplateTrackerZone tz;
    tz.add(t1);
    tz.add(t2);
    tracker->initFromZone(I, tz);
    initialized = true;
  }
  
  vpDisplay::display(I); 
  if(initialized)
  {
    try{
      tracker->track(I);
      
      vpColVector p = tracker->getp();
      vpHomography H = warp->getHomography(p);
      std::cout << "Homography: \n" << H << std::endl;
      
      tracker->display(I, vpColor::red);
    }catch(vpTrackingException e)
    {
      ROS_INFO("An exception occurred.. cancelling tracking.");
      tracker->resetTracker();
      initialized = false;
    }
  }
 
  vpDisplay::flush(I);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "templatetracker");
  ros::NodeHandle n;
  
  I.init(480, 640);
  
  ros::Subscriber sub = n.subscribe("image_raw", 100, imageCallback);
  
  display = new vpDisplayX(I, 0, 0, "Image");
  vpDisplay::display(I);
  vpDisplay::flush(I);

  warp = new vpTemplateTrackerWarpHomography();
  tracker = new vpTemplateTrackerSSDInverseCompositional(warp);
  tracker->setSampling(1, 1);
  tracker->setLambda(0.001);
  tracker->setIterationMax(200);
  tracker->setPyramidal(2, 0);

  ros::spin();
 
  free(tracker);
  free(warp);
  
  return 0;
}