#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"

#include "visp_bridge/image.h"
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpKeyPoint.h>

#include <unistd.h>
#include <algorithm>

// detector info
const std::string detectorName = "ORB";
const std::string extractorName = "ORB";
const std::string matcherName = "FlannBased";
vpImage<unsigned char> rI; // rI image
vpImage<unsigned char> I; // topic image
vpImage<unsigned char> iDisp; // what gets displayed on screen

vpKeyPoint keypoint;

vpDisplayOpenCV * display;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO("Image received, %d", msg->header.seq);
  I = visp_bridge::toVispImage(*msg); 
  

  // match and update iDisp
  iDisp.insert(I, vpImagePoint(0, rI.getWidth()));
  vpDisplay::display(iDisp);
  vpDisplay::displayLine(iDisp, vpImagePoint(0, rI.getWidth()), vpImagePoint(rI.getHeight(), rI.getWidth()), vpColor::blue, 4); // separator line

  unsigned int nbMatch = keypoint.matchPoint(I);
  vpImagePoint iPref, iPcur;
  for(unsigned int i = 0; i < nbMatch; i++)
  {
    keypoint.getMatchedPoints(i, iPref, iPcur);
    std::cout << "Matchpoint " << i << " " << iPref << " " << iPcur << std::endl;
    vpDisplay::displayLine(iDisp, iPref, iPcur + vpImagePoint(0, I.getWidth()), vpColor::green);
  }  
  
  // display all keypoints in reference image
  for(unsigned int i = 0; i < keypoint.getReferencePointNumber(); i++)
  {
    vpImagePoint referencePoint;
    keypoint.getReferencePoint(i, referencePoint);
    vpDisplay::displayPoint(iDisp, referencePoint, vpColor::red, 10);
  }
  
  vpDisplay::flush(iDisp);

}

int main(int argc, char ** argv)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x20101)

  ros::init(argc, argv, "keypointmatcher");
  ros::NodeHandle n;
  
  
//   ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::Subscriber sub = n.subscribe("image_raw", 100, imageCallback);
  ros::spinOnce(); // to initialize the images coming from the topic
  
  // initialize detector
  vpKeyPoint::vpFilterMatchingType filterType = vpKeyPoint::ratioDistanceThreshold;
  keypoint = vpKeyPoint(detectorName, extractorName, matcherName, filterType);
  
  // initialize reference image
  vpImageIo::read(rI, "X.png");
  I.init(480,640);
  
  // build reference
  std::cout << "Reference keypoints = " << keypoint.buildReference(rI) << std::endl;
  
  // Show display image
  iDisp.init(std::max(rI.getHeight(), I.getHeight()), rI.getWidth() + I.getWidth());
  iDisp.insert(rI, vpImagePoint(0, 0));
  display = new vpDisplayOpenCV(iDisp, 0, 0, "Matching keypoints with ORB keypoints");
  vpDisplay::display(iDisp);
  vpDisplay::flush(iDisp);
  
  ros::spin();
    
#else
  std::cout << "You don't have OpenCV >= 2.1.1" << std::endl;
#endif
  
  delete display;
  return 0;
}