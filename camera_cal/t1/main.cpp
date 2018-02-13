#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>

#include <cmath>

using namespace std;
using namespace cv;


struct Ellipse
{
	RotatedRect ellipse ;
	int index;
	Ellipse(RotatedRect ellipse, int index)
	{
		this->ellipse = ellipse;
		this->index = index;
	}
};

float distance_center(RotatedRect &a, RotatedRect &b)
{
	return sqrt( pow(a.center.x - b.center.x, 2) 
			+ pow (a.center.y - b.center.y, 2) );
}


Mat simpleTreshold(Mat image)
{
	Mat hsv, mask;

	Mat dst(image.rows, image.cols, CV_8UC3, Scalar(255, 255, 255));

	cv::cvtColor(image, hsv, COLOR_RGB2HSV, 4);

	GaussianBlur( image, image, Size( 25, 25), 0, 0 );

	inRange(hsv, Scalar(0, 0, 0), Scalar(180, 255, 80), mask);

	bitwise_and(image, image, dst, mask = mask);
	cvtColor(dst, dst, CV_BGR2GRAY, 1 );

	return dst;
}

Mat detectShineSquare(Mat image)
{

	int thresh = 250; //deberia ser el maximo valor de  la imagen
	int max_thresh = 255;
	RNG rng(12345);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	Mat src_gray, threshold_output;

	cvtColor(image, src_gray, CV_BGR2GRAY, 1 );

	threshold( src_gray, threshold_output, thresh, max_thresh, THRESH_BINARY );
	findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	vector<RotatedRect> minRect( contours.size() );

	Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );

	int indexMax = 0;
	float maxSum = 0;
	float maxDif = 0;

	for( int i = 0; i < contours.size(); i++ )
    { 
    	minRect[i] = minAreaRect( Mat(contours[i]) );
    	float sum = minRect[i].size.width + minRect[i].size.height;
    	float dif = abs(minRect[i].size.width - minRect[i].size.height);

    	if(sum>maxSum && dif<5)
    	{
    		maxSum = sum;
    		indexMax = i;
    		cout<<"sum: "<<sum<<" "<<"dif: "<<dif<<endl;
    	}

    }


    Point2f rect_points[4]; 
    minRect[indexMax].points( rect_points );

    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    for( int j = 0; j < 4; j++ )
    {
        line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
    }

    


	imshow("drawing", drawing);

	return threshold_output;
}

void detect(Mat& image)
{
	Size size(960,540);
	resize(image, image, size);

	int thresh = 100;
	int max_thresh = 255;
	RNG rng(12345);
	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	Mat src_gray = simpleTreshold(image);

	threshold( src_gray, threshold_output, thresh, max_thresh, THRESH_BINARY );

	findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	vector<RotatedRect> minRect( contours.size() );
  	vector<RotatedRect> minEllipse( contours.size() );
  	vector<Ellipse*> lsEllipses;

  	for( int i = 0; i < contours.size(); i++ )
    { 
    	minRect[i] = minAreaRect( Mat(contours[i]) );
       	if( contours[i].size() > 5 )
        { 
        	minEllipse[i] = fitEllipse( Mat(contours[i]) ); 
        }
    }


    float umbral_center = 1.0f;


    for(int i = 0; i<contours.size(); i++)
    {
    	int count_neigh_center = 0;
    	for(int j = 0; j<contours.size(); j++)
	    {
	    	if(i!=j)
	    	{
	    		if( distance_center(minEllipse[i], minEllipse[j]) < umbral_center)
				{
					count_neigh_center++;
				}
	    	}
	    }
	    cout<<count_neigh_center<<endl;
	    if( count_neigh_center == 1 )
	    {
	    	lsEllipses.push_back(new Ellipse(minEllipse[i], i));
	    }
    }


   
    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );

  	for( int i = 0; i< lsEllipses.size(); i++ )
  	{
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

       ellipse( drawing, lsEllipses[i]->ellipse, color, 2, 8 );
    }

	// Show in a window
	imshow( "Contours", drawing );



}

int main()
{

	
	Mat frame = imread( "../res/images/frames/frame553.jpg");

	if (!frame.data )
	{
		cout << "Image no read" << endl;
		return -1;
	}

	imshow("frame", frame);


	Mat shine = detectShineSquare(frame);
	

	//detect(frame);

	waitKey();


	/*
	string filename("../res/videos/calibration_kinectv2.avi");

	VideoCapture capture(filename.c_str());
	Mat frame;

	if ( !capture.isOpened() )
	{
		cout << "Error when reading steam_avi" << endl;
		return -1;
	}
	
	//int i = 0;
	namedWindow( "w", 1);
	for ( ; ; )
	{
		capture >> frame;
		if (frame.empty())
		{
			break;
		}
		detect(frame);
		imshow("w", frame);
		waitKey(10);
		//i++;

	}*/

	return 0;
}
