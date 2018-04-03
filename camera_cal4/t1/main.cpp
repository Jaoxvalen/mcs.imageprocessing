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
#include <sstream>

using namespace std;
using namespace cv;
using namespace chrono;
using namespace vision;



/*parametros de entrada*/
string p_distribution;
string p_dir_in;
string p_dir_out;
int p_n_frames;
string p_fronto_par_type;
string p_control_points_type;
int p_fronto_par_type_int;
int p_control_points_type_int;
double p_min_angle;
double p_max_angle;
/*---------------------*/


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
	AUTO_CALIBRATION
};

int init(int type)
{

	ProcManager procHandler;

	//capture frames
	if ( type == CAPTURE_FRAMES )
	{
		Utils::captureFrames("../res/videos/calibration_ps3eyecam.avi", "../res/images/frames/", true);
	}
	else if ( type == AUTO_CALIBRATION )
	{

		/*
		CalibHandler manager(CONCENTRIC_CIRCLES , Size(5, 4), 44.3f , "../res/results/test_dist/", "../res/videos/mkv_ps3.mkv");
		manager.auto_calibration(50);

		IterativeCalibration ic(CONCENTRIC_CIRCLES, 44.3f, PERSPECTIVE_TRANSFORM, RP_SIMPLE, false);
		ic.init_calibrate( "../res/results/test_dist/");
		*/

		CalibHandler manager(CONCENTRIC_CIRCLES , Size(5, 4), 44.3f , p_dir_out, p_dir_in);

		if(p_distribution == "DIST_ANGLE")
		{
			manager.auto_calibration2(p_n_frames, p_min_angle, p_max_angle);
		}
		else if( p_distribution =="DIST_RAMDOM" )
		{
			manager.auto_calibration(p_n_frames);
		}
		

		IterativeCalibration ic(CONCENTRIC_CIRCLES, 44.3f, p_fronto_par_type_int, p_control_points_type_int, false);
		ic.init_calibrate( p_dir_out);

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




int main(int argc, char** argv)
{



	//CASOS DE PRUEBA

	//./main.out DIST_RAMDOM ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_PERSPECTIVE RP_SIMPLE
	//./main.out DIST_RAMDOM ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_PERSPECTIVE RP_AVG_SIMPLE_COLINEARITY
	//./main.out DIST_RAMDOM ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_PERSPECTIVE RP_BARICENTER
	//./main.out DIST_RAMDOM ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_PERSPECTIVE RP_COLINEARITY

	//./main.out DIST_RAMDOM ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_SIMPLE
	//./main.out DIST_RAMDOM ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_AVG_SIMPLE_COLINEARITY
	//./main.out DIST_RAMDOM ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_BARICENTER
	//./main.out DIST_RAMDOM ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_COLINEARITY

	//./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_PERSPECTIVE RP_SIMPLE 0 15
	//./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_PERSPECTIVE RP_AVG_SIMPLE_COLINEARITY 0 15
	//./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_PERSPECTIVE RP_BARICENTER 0 15
	//./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_PERSPECTIVE RP_COLINEARITY 0 15

	//./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_SIMPLE 0 15
	//./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_AVG_SIMPLE_COLINEARITY 0 15
	//./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_BARICENTER 0 15
	//./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_COLINEARITY 0 15

	//./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_SIMPLE 0 15
	//./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_AVG_SIMPLE_COLINEARITY 0 15
	//./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_BARICENTER 5 65
	//./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_COLINEARITY 5 65


	/*

		Ejecutar:
		./main.out p_distribution p_dir_in p_dir_out p_n_frames p_fronto_par_type p_control_points_type p_min_angle p_max_angle

		p_distribution:
		DIST_RAMDOM|DIST_ANGLE

		p_dir_in:
		ruta del video de entrada

		p_dir_out:
		ruta de carpeta de salida

		p_n_frames:
		numero de frames para calibrar

		p_fronto_par_type
		FP_INS_EXT|FP_PERSPECTIVE

		p_control_points_type
		RP_SIMPLE|RP_COLINEARITY|RP_AVG_SIMPLE_COLINEARITY|RP_BARICENTER

		p_min_angle
		minimo angulo en radianes

		p_max_angle
		max angulo en radianes
	*/

	//.mai


	if (argc < 7) {
		cout << "Faltan parametros" << endl;
		return 0;
	}
	else
	{
		p_distribution = string(argv[1]);
		p_dir_in = string(argv[2]);
		p_dir_out = string(argv[3]);
		istringstream ss(argv[4]);
		if (!(ss >> p_n_frames))
		{
			cerr << "p_n_frames debe ser un numero" << argv[4] << endl;
			return 0;
		}

		if ( p_n_frames < 1 )
		{
			cerr << "p_n_frames debe ser un numero positivo" << argv[4] << endl;
			return 0;
		}

		p_fronto_par_type = string(argv[5]);
		p_control_points_type = string(argv[6]);

		if ( !(p_distribution == "DIST_RAMDOM" || p_distribution == "DIST_ANGLE") )
		{
			cerr << "p_distribution debe ser DIST_RAMDOM o DIST_ANGLE" << endl;
			return 0;
		}

		if (p_distribution == "DIST_ANGLE")
		{
			if (argc < 9)
			{
				cout << "Faltan parámetros de minimo angulo y maximo angulo" << endl;
				return 0;
			}

			istringstream ss1(argv[4]);
			if (!(ss1 >> p_min_angle))
			{
				cerr << "p_min_angle debe ser un numero " << argv[7] << endl;
				return 0;
			}

			if ( p_min_angle < 1 )
			{
				cerr << "p_min_angle debe ser un numero positivo " << argv[7] << endl;
				return 0;
			}

			istringstream ss2(argv[8]);
			if (!(ss2 >> p_max_angle))
			{
				cerr << "p_max_angle debe ser un numero " << argv[8] << endl;
				return 0;
			}

			if ( p_max_angle < 1 )
			{
				cerr << "p_max_angle debe ser un numero positivo " << argv[8] << endl;
				return 0;
			}
		}

		if ( !(p_fronto_par_type == "FP_INS_EXT" || p_fronto_par_type == "FP_PERSPECTIVE") )
		{
			cerr << "p_fronto_par_type debe ser FP_INS_EXT o FP_PERSPECTIVE" << endl;
			return 0;
		}

		if ( p_fronto_par_type == "FP_INS_EXT" ) p_fronto_par_type_int = INSTRINSIC_EXTRINSIC;
		else if ( p_fronto_par_type == "FP_PERSPECTIVE" ) p_fronto_par_type_int = PERSPECTIVE_TRANSFORM;

		if ( !(p_control_points_type == "RP_SIMPLE"
		        || p_control_points_type == "RP_COLINEARITY"
		        || p_control_points_type == "RP_AVG_SIMPLE_COLINEARITY"
		        || p_control_points_type == "RP_BARICENTER"
		      ) )
		{
			cerr << "p_fronto_par_type debe ser: RP_SIMPLE, RP_COLINEARITY, RP_AVG_SIMPLE_COLINEARITY o RP_BARICENTER" << endl;
			return 0;
		}

		if ( p_fronto_par_type == "RP_SIMPLE" ) p_control_points_type_int = RP_SIMPLE;
		else if ( p_fronto_par_type == "RP_COLINEARITY" ) p_control_points_type_int = RP_COLINEARITY;
		else if ( p_fronto_par_type == "RP_AVG_SIMPLE_COLINEARITY" ) p_control_points_type_int = RP_AVG_SIMPLE_COLINEARITY;
		else if ( p_fronto_par_type == "RP_BARICENTER" ) p_control_points_type_int = RP_BARICENTER;


	}

	return init(AUTO_CALIBRATION);

}