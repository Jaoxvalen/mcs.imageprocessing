#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>

#include <cmath>

using namespace std;
using namespace cv;


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
			waitKey(10); // waits to display frame
		}
		imwrite( path, frame );
		i++;
	}

	cout << "all frames was readed" << endl;

	waitKey(0);
}

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

bool compare_center_x(Ellipse *A, Ellipse *B)
{
	return A->ellipse.center.x < B->ellipse.center.x;
}


bool compare_center_y(Ellipse *A, Ellipse *B)
{
	return A->ellipse.center.y < B->ellipse.center.y;
}

bool compare_size(Ellipse *A, Ellipse *B)
{
	float sizeA = A->ellipse.size.width + A->ellipse.size.height;
	float sizeB = B->ellipse.size.width + B->ellipse.size.height;
	return sizeA < sizeB;
}


float distance_center(RotatedRect &a, RotatedRect &b)
{
	return sqrt( pow(a.center.x - b.center.x, 2)
	             + pow (a.center.y - b.center.y, 2) );
}

float distance_center(RotatedRect &a, float x, float y)
{
	return sqrt( pow(a.center.x - x, 2)
	             + pow (a.center.y - y, 2) );
}

Mat simpleTreshold(Mat image)
{
	Mat hsv, mask;

	Mat dst(image.rows, image.cols, CV_8UC3, Scalar(255, 255, 255));

	cv::cvtColor(image, hsv, COLOR_RGB2HSV, 4);

	GaussianBlur( image, image, Size( 15, 15), 0, 0 );

	inRange(hsv, Scalar(0, 0, 0), Scalar(180, 255, 80), mask);

	bitwise_and(image, image, dst, mask = mask);
	cvtColor(dst, dst, CV_BGR2GRAY, 1 );

	return dst;
}


vector<Ellipse*> detect(Mat& image, bool Adaptative, int thresh, int max_thresh)
{


	Mat threshold_output, src_gray;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Ellipse*> lsEllipses;

	if (!Adaptative)
	{
		src_gray = simpleTreshold(image);
		threshold( src_gray, threshold_output, thresh, max_thresh, THRESH_BINARY );
	}
	else
	{
		//GaussianBlur( image, image, Size( 15, 15), 0, 0 );
		cvtColor(image, src_gray, CV_BGR2GRAY, 1 );
		adaptiveThreshold(src_gray , threshold_output, thresh, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 15, 2);
	}

	findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );



	vector<RotatedRect> minRect( contours.size() );
	vector<RotatedRect> minEllipse( contours.size() );

	//encuadrar
	for ( int i = 0; i < contours.size(); i++ )
	{
		minRect[i] = minAreaRect( Mat(contours[i]) );
		if ( contours[i].size() > 5 )
		{
			minEllipse[i] = fitEllipse( Mat(contours[i]) );
		}
	}


	//heuristica
	float umbral_center = 2.0f;

	for (int i = 0; i < contours.size(); i++)
	{
		int count_neigh_center = 0;
		for (int j = 0; j < contours.size(); j++)
		{
			if (i != j)
			{
				if ( distance_center(minEllipse[i], minEllipse[j]) < umbral_center)
				{

					float sizeA = minEllipse[i].size.width + minEllipse[i].size.height;
					float sizeB = minEllipse[j].size.width + minEllipse[j].size.height;

					if (sizeA > 0.5 && sizeB > 0.5)
					{

						float prop = (float) max(sizeA, sizeB) / (float)(max (min(sizeA, sizeB), 1.0f));

						//cout << "prop " << prop << endl;
						//cout<<"SizeA "<<sizeA<<"  "<<"sizeB "<<sizeB<<endl;
						//if (abs(sizeA - sizeB) < 100 && abs(sizeA - sizeB) > 40)
						//if (prop > 2 && prop < 3)
						//{
						count_neigh_center++;
						//}
					}
				}
			}
		}


		//cout<<count_neigh_center<<endl;
		if ( count_neigh_center == 1 )
		{
			lsEllipses.push_back(new Ellipse(minEllipse[i], i));
		}
	}

	//heuristica mediana


	if (lsEllipses.size() >= 2)
	{

		float mx, my;
		sort(lsEllipses.begin(), lsEllipses.end(), compare_center_x);
		mx = lsEllipses[lsEllipses.size() / 2]->ellipse.center.x;
		sort(lsEllipses.begin(), lsEllipses.end(), compare_center_y);
		my = lsEllipses[lsEllipses.size() / 2]->ellipse.center.y;
		//cout << "mx : " << mx << ", my :" << my << endl;


		float size_median = 0;
		sort(lsEllipses.begin(), lsEllipses.end(), compare_size);
		size_median = lsEllipses[lsEllipses.size() / 2]->ellipse.size.width + lsEllipses[lsEllipses.size() / 2]->ellipse.size.height;

		//cout<<"size_median : "<<size_median<<endl;

		vector<Ellipse*> lsEllipses_aux;
		for (int i = 0; i < lsEllipses.size(); i++)
		{
			if (distance_center(lsEllipses[i]->ellipse, mx, my)  < size_median * (3.5))
			{
				lsEllipses_aux.push_back(lsEllipses[i]);
			}
		}

		lsEllipses = lsEllipses_aux;

	}



	return lsEllipses;


	//return vector<Ellipse*>(0);
}

void process(Mat& image)
{

	high_resolution_clock::time_point t1 = high_resolution_clock::now();


	Size size(640, 480);
	resize(image, image, size);

	Mat copy = image.clone();
	vector<Ellipse*> lsEllipses;

	lsEllipses = detect(image, true, 100, 255);


	/*
	//no tiene ellipses
	if (lsEllipses.size() < 1)
	{
		lsEllipses = detect(copy, true, 200, 255);
	}*/


	high_resolution_clock::time_point t2 = high_resolution_clock::now();
	duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

	float fps = (1.0f / time_span.count());
	cout<<fps<<endl;


	//descomentar
	Mat drawing = Mat::zeros( image.size(), CV_8UC3 );

	for ( int i = 0; i < lsEllipses.size(); i++ )
	{
		Scalar color = Scalar( 255, 0, 0 );
		ellipse( image, lsEllipses[i]->ellipse, color, 2, 8 );
	}

	// Show in a window
	//imshow( "drawing", drawing );

	//image = drawing;


}



int main()
{

	//captureFrames("../res/videos/calibration_kinectv2.avi", "../res/images/frames/", true);

	/*

	for(int i = 0 ; i<5; i++)
	{
		Mat frame = imread( "../res/images/frames/frame"+to_string(i)+".jpg");

		if (!frame.data )
		{
			cout << "Image no read" << endl;
			return -1;
		}

		//imshow("frame", frame);

		process(frame);
		imshow( "frame"+to_string(i), frame );
	}



	waitKey();*/




	string filename("../res/videos/calibration_kinectv2.avi");

	VideoCapture capture(filename.c_str());

	//VideoCapture capture(0);
	Mat frame;

	if ( !capture.isOpened() )
	{
		cout << "Error when reading steam_avi" << endl;
		return -1;
	}

	int i = 0;
	namedWindow( "w", 1);
	for ( ; ; )
	{
		capture >> frame;
		if (frame.empty())
		{
			break;
		}
		process(frame);
		imshow("w", frame);
		waitKey(10);
		i++;

		//cout << i << endl;

	}

	return 0;
}
