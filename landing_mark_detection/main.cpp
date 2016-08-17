#include "landing_mark_detection.h"

int main( int argc, char** argv )
{
    if( argc != 2)
    {
		cout <<" Usage: ./landing_mark_detection pathToImage" << endl;
		return -1;
    }
    Mat img = imread(argv[1]);
    DetectLandingMark d;
    if(d.detect(img)){
		DetectLandingMark::landing_mark lm = d.get_landing_mark();
		cout<<"X: "<<lm.x<<" Y: "<<lm.y<<"\nWidth: "<<lm.width<<" Height: "<<lm.height<<"\nCircle Center X: "<<lm.center_x<<" Circle Center Y: "<<lm.center_y<<endl;
	}
	else
		cout<<"Couldn't find the landing mark!!"<<endl;

    return 0;
}
