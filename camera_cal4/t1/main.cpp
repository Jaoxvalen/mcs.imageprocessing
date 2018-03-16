#include "Utils.h"
#include "ProcManager.h"
//#include "CalibManager.h"
#include "CalibHandler.h"
#include "IterativeCalibration.h"

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
	ITERATIVE_CALIB
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
		int type_choose = CHESSBOARD;
		string frames;
		Size mPatternSize;
		string parameters;
		float sizePattern = 1.0f;
		switch(type_choose) {
		    case CHESSBOARD : 
		    		//chessboard
		    		mPatternSize = Size(9, 6);
		    		frames = "frames_chess/";
		    		parameters = "calib_chess_ps3.yml";
		    		sizePattern = 20.0f;
	    			break;

		    case CONCENTRIC_CIRCLES : 
		    		//rings
		    		mPatternSize = Size(5, 4);
		    		frames = "frames_rings/";
		    		parameters = "calib_concentrics_ps3.yml";
		    		sizePattern = 42.0f;
	    			break;
		}

		cout<<parameters<<endl;
		manager.refineControlPoints(parameters, "../res/images/calibration/"+ frames, mPatternSize, type_choose, sizePattern, "../res/results/");
	}
	else if(ITERATIVE_CALIB)
	{
		int type_choose = ASYMMETRIC_CIRCLES_GRID;

		if( type_choose == CHESSBOARD )
		{
			IterativeCalibration ic(CHESSBOARD, 20.0f);
			ic.init_calibrate( "calib_chess_ps3.yml" ,"../res/images/calibration/frames_chess/", "../res/results/");
		}
		else if( type_choose == CONCENTRIC_CIRCLES )
		{
			IterativeCalibration ic(CONCENTRIC_CIRCLES, 42.0f);
			ic.init_calibrate( "calib_concentrics_ps3.yml" ,"../res/images/calibration/frames_rings/", "../res/results/");
		}
		else if( type_choose == ASYMMETRIC_CIRCLES_GRID )
		{
			IterativeCalibration ic(ASYMMETRIC_CIRCLES_GRID, 20.0f);
			ic.init_calibrate( "calib_asycircles_ps3.yml" ,"../res/images/calibration/frames_asymetrics/", "../res/results/");
		}


		
		
	}

	//waitKey();

	return 0;
}

int main()
{
	/*
	cout << "cv's build information" << endl;
	cout << cv::getBuildInformation() << endl;*/

	return init(ITERATIVE_CALIB);

}