#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>

#include <cmath>

using namespace std;
using namespace cv;


struct InfoEllipse
{
	RotatedRect* ellipse ;
	int index;

	InfoEllipse(int _index, RotatedRect* _ellipse)
	{
		index = _index;
		ellipse = _ellipse;
	}

	float distance_center(InfoEllipse* other)
	{
		return sqrt( pow(ellipse->center.x-other->ellipse->center.x, 2) 
			+ pow (ellipse->center.y-other->ellipse->center.y, 2) );
	}
};


void captureFrames(const string& path_video, const string& path_to_frames, bool show);
Mat getBinaryImage(const Mat& image);
void detect_pattern(Mat& image);
int n_neights(InfoEllipse *item, vector<InfoEllipse*> &ls_info_ellipse, float radio);


int main()
{
	//captureFrames("../res/videos/Kinect2_rgb.avi", "../res/images/frames/", false);
	
	string filename("../res/videos/calibration_kinectv2.avi");
	VideoCapture capture(filename.c_str());
	Mat frame;

	if ( !capture.isOpened() )
	{
		cout << "Error when reading steam_avi" << endl;
		return -1;
	}
	
	namedWindow( "w", 1);
	for ( ; ; )
	{
		capture >> frame;
		if (frame.empty())
		{
			break;
		}
		detect_pattern(frame);
		imshow("w", frame);
		waitKey(10); // waits to display frame
	}

	return 0;
}

Mat getBinaryImage(const Mat& image)
{
	Mat hsv, mask;

	Mat dst(image.rows, image.cols, CV_8UC3, Scalar(255, 255, 255));
	cv::cvtColor(image, hsv, COLOR_RGB2HSV, 4);

	inRange(hsv, Scalar(0, 0, 0), Scalar(180, 255, 30), mask);
	bitwise_and(image, image, dst, mask = mask);

	cvtColor(dst, dst, CV_BGR2GRAY, 1 );

	return dst;
}

void detect_pattern(Mat& image)
{

	resize(image, image, cv::Size(), 0.5, 0.5);
	Mat src_gray = getBinaryImage(image);

	vector<InfoEllipse*> ls_info_ellipse;
	int thresh = 100;
	int max_thresh = 255;
	RNG rng(12345);
	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	/// Detect edges using Threshold
	threshold( src_gray, threshold_output, thresh, max_thresh, THRESH_BINARY );
	/// Find contours
	findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	vector<RotatedRect> minEllipse( contours.size() );
	vector<int> ie;

	for ( int i = 0; i < contours.size(); i++ )
	{
		if ( contours[i].size() > 5 )
		{
			minEllipse[i] = fitEllipse( Mat(contours[i]) );
			ls_info_ellipse.push_back(new InfoEllipse(i, &minEllipse[i]));
		}
	}

	//Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
	
	for ( int i = 0; i < ls_info_ellipse.size(); i++ )
	{
		//Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255) );

		Scalar color = Scalar( 0, 0, 255 );
		if(n_neights(ls_info_ellipse[i], ls_info_ellipse, 1.0f) > 0)
		{
			ellipse( image, *(ls_info_ellipse[i]->ellipse), color, 2, 8 );
		}
		
	}
}

int n_neights(InfoEllipse *item, vector<InfoEllipse*> &ls_info_ellipse, float radio)
{
	int count = 0;
	for(int i=0; i<ls_info_ellipse.size(); i++)
	{
		InfoEllipse* neigh = ls_info_ellipse[i];
		//cout<<item->distance_center(neigh)<<endl;
		if( item->distance_center(neigh)<radio && neigh!=item)
		{
			count++;
		}
	}
	return count;
}

void captureFrames(const string& path_video, const string& path_to_frames, bool show)
{

	//"../res/videos/Kinect2_rgb.avi"
	string filename = path_video;
	VideoCapture capture(filename.c_str());
	Mat frame;

	if ( !capture.isOpened() )
	{
		cout << "Error when reading steam_avi" << endl;
	}

	namedWindow( "w", 1);
	int i = 0;

	cout << "Reading frames" << endl;
	for ( ; ; )
	{
		string path = path_to_frames + string("frame") + to_string(i) + string(".jpg");
		capture >> frame;
		if (frame.empty())
			break;

		if (show)
		{
			imshow("w", frame);
			waitKey(20); // waits to display frame
		}
		imwrite( path, frame );
		i++;
	}

	cout << "all frames was readed" << endl;

	waitKey(0);
}