#pragma once

#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <random>

using namespace std;
using namespace cv;

namespace vision
{

enum TYPE_CALIB
{
	CHESSBOARD,
	CIRCLES_GRID,
	ASYMMETRIC_CIRCLES_GRID,
	CONCENTRIC_CIRCLES
};

enum TYPE_FRONTO_PARALLEL
{
	PERSPECTIVE_TRANSFORM,
	INSTRINSIC_EXTRINSIC
};

enum TYPE_REFINED_POINTS
{
	RP_SIMPLE,
	RP_COLINEARITY,
	RP_AVG_SIMPLE_COLINEARITY,
	RP_BARICENTER
};

class Utils
{
public:


	static int randint(int min, int max)
	{
		std::random_device rd;     // only used once to initialise (seed) engine
		std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
		std::uniform_int_distribution<int> uni(min, max); // guaranteed unbiased

		int random_integer = uni(rng);

		return random_integer;
	}


	static bool readImageFromFile(const string& path_image, Mat& image)
	{
		image = imread(path_image);

		if (!image.data )
		{
			cout << "Image no read" << endl;
			return false;
		}
		return true;
	}
	static void captureFrames(const string& path_video, const string& path_to_frames, bool show)
	{

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

};
}

