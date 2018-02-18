#include "Utils.h"
#include "ProcManager.h"

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <cmath>
#include <chrono>

using namespace std;
using namespace cv;
using namespace chrono;
using namespace vision;

enum initType 
{ 
	CAPTURE_FRAMES, 
	OPEN_FROM_FILE_IMAGE, 
	OPEN_FROM_FILE_VIDEO 
};

int init(int type)
{

	ProcManager::create();

	//capture frames
	if ( type == CAPTURE_FRAMES )
	{
		Utils::captureFrames("../res/videos/calibration_ps3eyecam.avi", "../res/images/frames/", true);
	}
	//open from image file
	else if ( type == OPEN_FROM_FILE_IMAGE)
	{
		Mat frame;

		if ( !(Utils::readImageFromFile("../res/images/frames/frame0.jpg", frame)) )
		{
			return -1;
		}

		
		ProcManager::INSTANCE->preProcessing(frame);
		imshow("frame", frame);
		waitKey(10);
	}
	//read video from file
	else if ( type == OPEN_FROM_FILE_VIDEO )
	{
		string filename("../res/videos/calibration_ps3eyecam.avi");
		VideoCapture capture(filename.c_str());
		//VideoCapture capture(0);

		Mat frame;
		if ( !capture.isOpened() )
		{
			cout << "Error when reading steam_avi" << endl;
			return -1;
		}
		int i = 0;
		int w = -1;
		capture >> frame;
		i++;
		do 
		{
			ProcManager::INSTANCE->preProcessing(frame);
			imshow("frame" , frame);
			w = waitKey();
			if(w == 83)
			{
				capture >> frame;
				i++;
			}
			cout<<"frame"<<i<<endl;
		}while(w!=27);//escape

		/*
		for ( ; ; )
		{
			capture >> frame;
			if (frame.empty())
			{
				break;
			}

			ProcManager::INSTANCE->preProcessing(frame);
			imshow("frame" , frame);
			waitKey(10);
			i++;
			//cout<<i<<endl;
		}*/
	}

	//waitKey();

	return 0;
}

int main()
{
	return init(OPEN_FROM_FILE_VIDEO);
}

