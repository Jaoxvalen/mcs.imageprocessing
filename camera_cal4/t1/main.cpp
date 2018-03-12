#include "Utils.h"
#include "ProcManager.h"
//#include "CalibManager.h"
#include "CalibHandler.h"

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
	OPEN_FROM_FILE_VIDEO,
	CAPTURE_FROM_VIDEO,
	CAPTURE_FRAMES_CALIB_CHECKBOARD,
	CALIB_CHECKBOARD,
	CALIB_CIRCLES_GRID,
	CALIB_ASYMMETRIC_CIRCLES_GRID,
	CALIB_CONCENTRIC_CIRCLES,
	FRONTO_PARALLEL
};

int init(int type)
{

	ProcManager procHandler;

	//ProcManager::create();

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

		vector<Point2f> pointBuf;
		//ProcManager::INSTANCE->preProcessing(frame, pointBuf);

		//ProcManager procHandler;
		procHandler.preProcessing(frame , pointBuf);
		imshow("frame", frame);
		waitKey(10);
	}
	//read video from file
	else if ( type == OPEN_FROM_FILE_VIDEO )
	{

		string filename("../res/videos/Calibration/LifeCam/Rings.wmv");
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
			vector<Point2f> pointBuf;
			//ProcManager::INSTANCE->preProcessing(frame, pointBuf);

			//ProcManager procHandler;
			procHandler.preProcessing(frame , pointBuf);
			imshow("frame" , frame);
			w = waitKey();
			if (w == 83)
			{
				capture >> frame;
				i++;
			}
			cout << "frame" << i << endl;

		} while (w != 27); //escape

	}

	//read video from file
	else if ( type == CAPTURE_FROM_VIDEO )
	{

		VideoCapture capture(0);

		Mat frame;
		if ( !capture.isOpened() )
		{
			cout << "Error when reading steam_avi" << endl;
			return -1;
		}
		int i = 0;

		for ( ; ; )
		{
			capture >> frame;
			if (frame.empty())
			{
				break;
			}

			vector<Point2f> pointBuf;
			//ProcManager::INSTANCE->preProcessing(frame, pointBuf);

			procHandler.preProcessing(frame , pointBuf);
			imshow("frame" , frame);
			waitKey(10);
			i++;
			//cout<<i<<endl;
		}
	}

	else if ( type == CALIB_CHECKBOARD )
	{
		CalibHandler manager(CHESSBOARD , Size(9, 6), 20.0f, "../t1/calib_chess_ps3.yml",
		                     //"../res/videos/Calibration/Camera_Play_3/ChessBoard.webm");
		                     "../res/videos/calibration/PS3_chess.avi");
		//"../res/videos/calibration/LifeCam_chess.avi");
		manager.calibration();
	}
	else if ( type == CALIB_ASYMMETRIC_CIRCLES_GRID )
	{
		CalibHandler manager(ASYMMETRIC_CIRCLES_GRID , Size(4, 11), 20.0f, "../t1/calib_asycircles_ps3.yml",
		                     "../res/videos/calibration/PS3_asymmetric_circles.avi");
		//"../res/videos/calibration/LifeCam_asymmetric_circles.avi");
		manager.calibration();
	}
	else if ( type == CALIB_CONCENTRIC_CIRCLES )
	{
		CalibHandler manager(CONCENTRIC_CIRCLES , Size(5, 4), 42.0f , "../t1/calib_concentrics_ps3.yml",
		                     "../res/videos/calibration/PS3_rings.avi");
		//"../res/videos/calibration/LifeCam_rings.avi");
		manager.calibration();
	}

	else if ( type == FRONTO_PARALLEL )
	{
		CalibHandler manager;
		manager.transFrontoParallel("calib_chess_ps3.yml", "../res/images/calibration/frames/");
	}

	//waitKey();

	return 0;
}

int main()
{

	return init(FRONTO_PARALLEL);
}
