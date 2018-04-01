#include "Utils.h"
#include "ProcManager.h"
//#include "CalibManager.h"
#include "CalibHandler.h"
#include "IterativeCalibration.h"

#include "RingsDetector.h"

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
	FRONTO_PARALLEL,
	ITERATIVE_CALIB,
	DETECTOR_PROTOTYPE,
	CALIBRATION_PROTOTYPE
};

int init(int type)
{

	ProcManager procHandler;

	//capture frames
	if ( type == CAPTURE_FRAMES )
	{
		Utils::captureFrames("../res/videos/calibration_ps3eyecam.avi", "../res/images/frames/", true);
	}
	else if ( type == CALIBRATION_PROTOTYPE )
	{

		CalibHandler manager(CONCENTRIC_CIRCLES , Size(5, 4), 44.3f ,"../res/results/test/", "../res/videos/mkv_ps3.mkv");
		manager.auto_calibration();
	}
	else if ( type == DETECTOR_PROTOTYPE )
	{
		string filename("../res/videos/PS3_rings.webm");
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

			RingsDetector rd;
			rd.findPattern(frame, pointBuf);


			imshow("frame" , frame);
			w = waitKey();
			if (w == 83)
			{
				capture >> frame;
				i++;
			}
			//cout << "frame" << i << endl;

		} while (w != 27); //escape

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
		}
	}

	else if ( type == CALIB_CHECKBOARD )
	{
		CalibHandler manager(CHESSBOARD , Size(7, 5), 27.0f, "../res/results/chessboard_20_life/", "../res/videos/life_chess.webm");
		manager.calibration();
	}
	else if ( type == CALIB_ASYMMETRIC_CIRCLES_GRID )
	{
		CalibHandler manager(ASYMMETRIC_CIRCLES_GRID , Size(4, 11), 35.0f, "../res/results/fakes/", "../res/videos/life_asymmetric_circles.webm");
		manager.calibration();
	}
	else if ( type == CALIB_CONCENTRIC_CIRCLES )
	{
		CalibHandler manager(CONCENTRIC_CIRCLES , Size(5, 4), 44.3f , "../res/results/rings_mkv_out/", "../res/videos/mkv_ps3.mkv");
		manager.calibration();
	}
	else if (ITERATIVE_CALIB)
	{
		int type_choose = CONCENTRIC_CIRCLES;

		if ( type_choose == CHESSBOARD )
		{
			IterativeCalibration ic(CHESSBOARD, 27.0f);
			ic.init_calibrate("../res/results/chessboard_50_life/");
		}
		else if ( type_choose == ASYMMETRIC_CIRCLES_GRID )
		{
			IterativeCalibration ic(ASYMMETRIC_CIRCLES_GRID, 35.0f);
			ic.init_calibrate( "../res/results/asymmetric_50_life/");
		}
		else if ( type_choose == CONCENTRIC_CIRCLES )
		{
			IterativeCalibration ic(CONCENTRIC_CIRCLES, 44.3f, INSTRINSIC_EXTRINSIC, RP_COLINEARITY, true);
			ic.init_calibrate( "../res/results/rings_mkv_out/");
		}

	}

	return 0;
}

int main()
{

	return init(CALIBRATION_PROTOTYPE);

}