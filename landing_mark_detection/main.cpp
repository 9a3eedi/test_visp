#include "landing_mark_detection.h"

int main( int argc, char** argv )
{
    if( argc != 3)
    {
		cout <<" Usage: ./landing_mark_detection pathToImage 1/0\n 1 to show results on the frame" << endl;
		return -1;
    }
    Mat img = imread(argv[1]);
    DetectLandingMark d;
    if(d.detect(img, atoi(argv[2]))){
		landing_mark lm = d.get_landing_mark();
		cout<<"X: "<<lm.x<<" Y: "<<lm.y<<"\nWidth: "<<lm.width<<" Height: "<<lm.height<<"\nCircle Center X: "<<lm.center_x<<" Circle Center Y: "<<lm.center_y<<endl;
	}
	else
		cout<<"Couldn't find the landing mark!!"<<endl;

    return 0;
}
