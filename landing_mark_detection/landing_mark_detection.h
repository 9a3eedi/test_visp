#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <cstring>
#include <stdio.h>

using namespace cv;
using namespace std;

class DetectLandingMark {
	public:
		struct landing_mark{
			double x, y;				//top-left (x,y) coordinates of the bounding rectangle
			double width, height, center_x, center_y;			//width, height of the rectangle, and the center of the landing mark
		};
		bool detect(Mat);
		landing_mark get_landing_mark() {
			return a;
		}
		DetectLandingMark(void){
			a.x = -1;
			a.y = -1;
			a.width = -1;
			a.height = -1;
			a.center_x = -1;
			a.center_y = -1;
		}
	private:
		landing_mark a;
};
bool DetectLandingMark::detect(Mat frame) {
    Mat edged;
    Mat edged2;
    Mat gray;
    Mat roi_square;
    string status;
    int status_flag = 0;
		
	//convert the frame to grayscale, blur it, and detect edges
	cvtColor(frame, gray, COLOR_BGR2GRAY);
	GaussianBlur(gray, gray, Size(7,7), 0);
	Canny(gray, edged, 50, 150);
			   
	//find contours in the edge map
	vector<vector<Point> > contours;
	vector<Point> approx;
	vector<Vec4i> hierarchy;
	vector<Point> hull;
	edged.copyTo(edged2);
	findContours(edged2, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
	
	//loop over the contours
	for( int i = 0; i< contours.size(); i++ ){
		double peri = arcLength(contours[i], true);
		approxPolyDP(contours[i], approx, 0.01 * peri, true);
		
		//ensure that the approximated contour is "roughly" rectangular
		if(approx.size()>=4 && approx.size()<=6){
			Rect rect = boundingRect(approx);
			float aspectRatio = rect.width / float(rect.height);
			
			//compute the solidity of the original contour
			double area = contourArea(contours[i]);
			convexHull(contours[i], hull);
			double hullArea = contourArea(hull);
			double solidity = area / double(hullArea);
			
			//compute whether or not the width and height, solidity, and aspect ratio of the contour falls within appropriate bounds
			bool keepDims = (rect.width > 100 and rect.height > 100)?true:false;
			bool keepSolidity = (solidity > 0.9)?true:false;
			bool keepAspectRatio = (aspectRatio >= 0.8 and aspectRatio <= 1.2)?true:false;
			
			//ensure that the contour passes all our tests
			if (keepDims and keepSolidity and keepAspectRatio) {
				//get ROI (inside the contour)
				roi_square = frame(rect);
				//detect circles in the image
				vector<Vec3f> circles; 
				HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1.2, 75); //circles = (x,y,radius)
				//if more than one circle is detected, select the one with largest radius
				if(circles.size() >= 1){
					//select circle with largest radius
					int max_r_index = 0;
					for(int m=0; m<circles.size(); m++){
						if(circles[m][2]>circles[max_r_index][2])
							max_r_index = m;
					}
					//draw an outline around the target
					drawContours(frame, contours, i, (0, 0, 255), 4);					
					//draw them
					circle(frame, Point(int(circles[max_r_index][0]), int(circles[max_r_index][1])), circles[max_r_index][2], Scalar(0, 255, 0), 4);
					//take the average of both shapes' centers and display it as the center of the landing mark
					double average_x = ((int(circles[max_r_index][0]))+(rect.x+rect.width/2.0))/2.0;
					double average_y = ((int(circles[max_r_index][1]))+(rect.y+rect.height/2.0))/2.0;
					rectangle(frame, Point(average_x - 5, average_y - 5), Point(average_x + 5, average_y + 5), Scalar(0, 128, 255), -1);
					//if the two centers are very close, then accept it
					if(average_x-(int(circles[max_r_index][0]))<=20){
						status = "Target Detected";
						status_flag = 1;
						putText(frame, status, Point(20, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(100, 120, 0), 2);
						a.x = rect.x;
						a.y = rect.y;
						a.width = rect.width;
						a.height = rect.height;
						a.center_x = average_x;
						a.center_y = average_y;
					}
				}
			}
		}
	}     
	imshow("Landing Mark Detection", frame);
	waitKey(0);
	return((status_flag==1)?true:false);
}
