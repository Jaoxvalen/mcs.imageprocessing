#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace std;
using namespace cv;


void captureFrames(const string& path_video, const string& path_to_frames, bool show);
Mat getBinaryImage(const Mat& image);
void thresh_callback(Mat& src_gray);
int getChangeFrequency(const Mat& image, Point p1, Point p2);

int main()
{


	//captureFrames("../res/videos/Kinect2_rgb.avi", "../res/images/frames/", false);

	Mat image;
	image = imread( "../res/images/frames/frame0.jpg");

	if (!image.data )
	{
		cout << "Image no read" << endl;
		return -1;
	}

	Mat dst;
	dst = getBinaryImage(image);

	//thresh_callback(dst);

	resize(dst, dst, cv::Size(), 0.5, 0.5);

	cout<<dst.rows<<" "<<dst.cols<<endl;

	//row col
	//108 192

	int fx = 8, fy = 6;

	int xp = (float)dst.cols/(float)fx;
	int yp = (float)dst.rows/(float)fy;

	for(int j = 0; j<dst.rows   ; j+=yp)
	{
		for(int i = 0; i<dst.cols  ; i+=xp )
		{
			Point p_ini(i, j);
			Point p_fin(i + xp ,j+yp);
			cout<<getChangeFrequency(dst, p_ini, p_fin )<<"\t";
		}
		cout<<endl;
	}

	/*/cout<<getChangeFrequency(dst, Point(0,0), Point(64,108))<<endl;

	cout<<getChangeFrequency(dst, Point(64,0), Point(128,108))<<endl;

	cout<<getChangeFrequency(dst, Point(128,0), Point(192,108))<<endl;*/



	imshow("dst", dst);

	waitKey();

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


void thresh_callback(Mat& src_gray)
{

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

	/// Find the rotated rectangles and ellipses for each contour
	vector<RotatedRect> minRect( contours.size() );
	vector<RotatedRect> minEllipse( contours.size() );

	for ( int i = 0; i < contours.size(); i++ )
	{	
		minRect[i] = minAreaRect( Mat(contours[i]) );
		if ( contours[i].size() > 5 )
		{
			minEllipse[i] = fitEllipse( Mat(contours[i]) );
		}
	}




	/// Draw contours + rotated rects + ellipses
	Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
	for ( int i = 0; i < contours.size(); i++ )
	{
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255) );
		// contour
		//drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
		// ellipse
		ellipse( drawing, minEllipse[i], color, 2, 8 );
		// rotated rectangle
		//Point2f rect_points[4]; minRect[i].points( rect_points );
		
		/*for ( int j = 0; j < 4; j++ )
			line( drawing, rect_points[j], rect_points[(j + 1) % 4], color, 1, 8 );*/
	}

	/// Show in a window
	namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
	imshow( "Contours", drawing );
}


int getChangeFrequency(const Mat& image, Point p1, Point p2)
{

	int count = 0;
	int val_ant, val_act;

	int count_w = 0, count_b = 0;

	for(int i = p1.x; i<p2.x ;  i++)
	{
		val_ant =((int)image.at<unsigned char>(0,i)!=255)?0:255;

		for(int j = p1.y; j<p2.y; j++)
		{
			val_act = ((int)image.at<unsigned char>(j,i)!=255)?0:255;

			if(val_act == 255)
			{
				count_w++;
			}
			else
			{
				count_b++;
			}

			if(val_ant!=val_act)
			{
				count++;
			}
			val_ant = val_act;
		}
	}


	for(int j = p1.y; j<p2.y; j++)
	{
		val_ant = ((int)image.at<unsigned char>(j,0)!=255)?0:255;
		for(int i = p1.x; i<p2.x ;  i++)
		{
			val_act = ((int)image.at<unsigned char>(j,i)!=255)?0:255;

			if(val_ant!=val_act)
			{
				count++;
			}
			val_ant = val_act;
		}
	}


	return count + abs(count_b - count_w);

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