#pragma once

#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


using namespace std;
using namespace cv;

namespace vision
{

enum TYPECALIB
{
	CHESSBOARD,
	CIRCLES_GRID,
	ASYMMETRIC_CIRCLES_GRID,
	CONCENTRIC_CIRCLES
};

class Utils
{
public:
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

