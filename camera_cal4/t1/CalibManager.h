#pragma once
#include <iostream>


#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <sstream>

using namespace cv;
using namespace std;

namespace vision
{

class CalibManager
{
public:
	CalibManager()
	{

	}

	void captureFrameCheck(const string& dir, const string& dirVideo, int sizeRealQuad, const string& outDir)
	{
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
		compression_params.push_back(100);

		Size patternsize = Size(9, 6); //esquinas interiores del tablero de ajedrez
		vector<Point3f> corners3D;
		vector<Point2f> corners2D;//findChessboardCorners guarda los puntos del tablero aqui
		vector<vector<Point2f>> coord2D;//Ubicacion de las esquinas detectadas en la imagen
		vector<vector<Point3f>> coord3D;//Ubicacion real de los puntos 3D

		calcChessboardCorners(patternsize, sizeRealQuad, corners3D);

		Mat img, imgGray;
		bool found;
		VideoCapture cam;

		cam = VideoCapture(dirVideo.c_str());

		if (!cam.isOpened()) {
			cout << "!No se pudo abrir la camara o el archivo" << endl;
			getchar();
			return;
		}

		bool SaveImage;
		int nimgs = 0;

		string direccion = dir;
		stringstream Saveimgs;

		int w = -1;
		cam >> img;
		do
		{
			w = waitKey(30);

			if (w == 83)
			{
				cam >> img;
				cvtColor(img, imgGray, COLOR_BGR2GRAY);

				found = findChessboardCorners(imgGray, patternsize, corners2D, CALIB_CB_ADAPTIVE_THRESH +
				                              CALIB_CB_NORMALIZE_IMAGE +
				                              CALIB_CB_FAST_CHECK);

				if (found) {
					cornerSubPix(imgGray, corners2D, Size(11, 11), Size(-1, -1), TermCriteria( TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1 ));
					drawChessboardCorners(img, patternsize, Mat(corners2D), found);
				}

			}

			if (w == 13)
			{
				coord2D.push_back(corners2D);
				coord3D.push_back(corners3D);
				nimgs++;
				cout<<"Add to calibration "<<nimgs<<endl;
				cout<<"Press esc to start calibrate"<<endl;
			}

			imshow("image", img);

		} while (w != 27); //escape

		cvDestroyWindow("image");

		Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
		Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
		vector<Mat> rvecs;
		vector<Mat> tvecs;

		double rms = calibrateCamera(coord3D, coord2D, img.size(), cameraMatrix,
		                             distCoeffs, rvecs, tvecs,
		                             CALIB_FIX_PRINCIPAL_POINT +
		                             CALIB_FIX_ASPECT_RATIO +
		                             CALIB_ZERO_TANGENT_DIST);

		cout << "RMS: " << rms << endl;
		cout << "Camera matrix: " << cameraMatrix << endl;
		cout << "Distortion _coefficients: " << distCoeffs << endl;

		saveparams(outDir + "DataCam.yml", cameraMatrix, distCoeffs, rvecs, tvecs, rms);



	}

	void calcChessboardCorners(Size boardSize, float squareSize, std::vector<cv::Point3f>& corners)
	{
		corners.clear();
		for ( int i = 0; i < boardSize.height; i++ )
			for ( int j = 0; j < boardSize.width; j++ )
				corners.push_back(Point3f(float(j * squareSize),
				                          float(i * squareSize), 0));
	}

	void saveparams(const std::string& filename, const Mat& cameraMatrix, const Mat& distCoeffs,
	                const std::vector<Mat>& rvecs, const std::vector<Mat>& tvecs, const double& RMS)
	{
		FileStorage fs( filename, FileStorage::WRITE );
		fs << "Calibrate_Accuracy" << RMS;
		fs << "Camera_Matrix" << cameraMatrix;
		fs << "Distortion_Coefficients" << distCoeffs;
		fs << "Rotation_Vector" << rvecs;
		fs << "Translation_vector" << tvecs;

		if ( !rvecs.empty() && !tvecs.empty() ) {

			CV_Assert(rvecs[0].type() == tvecs[0].type());
			Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
			for ( int i = 0; i < (int)rvecs.size(); i++ ) {
				Mat r = bigmat(Range(i, i + 1), Range(0, 3));
				Mat t = bigmat(Range(i, i + 1), Range(3, 6));

				CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
				CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);

				r = rvecs[i].t();
				t = tvecs[i].t();
			}
			cvWriteComment( *fs, "Rotation vector + Translation vector", 0 );
			fs << "extrinsic_parameters" << bigmat;
		}
		fs.release();
	}

	void calibrateCheckBoard(const string& dir, int nImages, const string& outDir)
	{
		Size patternsize = Size(9, 6); //esquinas interiores del tablero de ajedrez
		vector<Point3f> corners3D;
		vector<Point2f> corners2D;//findChessboardCorners guarda los puntos del tablero aqui
		vector<vector<Point2f>> coord2D;//Ubicacion de las esquinas detectadas en la imagen
		vector<vector<Point3f>> coord3D;//Ubicacion real de los puntos 3D

		string direccion = dir;
		stringstream imgs;

		calcChessboardCorners(patternsize, 2000, corners3D);

		Mat img, imgGray;
		bool found;



		for (int  i = 0; i < nImages; i++) {

			imgs << direccion << i << ".jpg";
			img = imread(imgs.str().c_str());
			cvtColor(img, imgGray, COLOR_BGR2GRAY);

			imgs = stringstream();

			found = findChessboardCorners(imgGray, patternsize, corners2D, CALIB_CB_ADAPTIVE_THRESH +
			                              CALIB_CB_NORMALIZE_IMAGE +
			                              CALIB_CB_FAST_CHECK);
			if ( found) {

				cornerSubPix(imgGray, corners2D, Size(11, 11), Size(-1, -1), TermCriteria(
				                 TermCriteria::EPS +
				                 TermCriteria::COUNT, 30, 0.1 ));
				drawChessboardCorners(img, patternsize, Mat(corners2D), found);
				coord2D.push_back(corners2D);
				coord3D.push_back(corners3D);
			}
			namedWindow("image", WINDOW_AUTOSIZE);
			imshow("image", img);
			waitKey(1500);
		}

		//






		cvDestroyWindow("image");

		Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
		Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
		vector<Mat> rvecs;
		vector<Mat> tvecs;

		double rms = calibrateCamera(coord3D, coord2D, img.size(), cameraMatrix,
		                             distCoeffs, rvecs, tvecs,
		                             CALIB_FIX_PRINCIPAL_POINT +
		                             CALIB_FIX_ASPECT_RATIO +
		                             CALIB_ZERO_TANGENT_DIST
		                             , TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 2.22e-16));

		cout << "RMS: " << rms << endl;
		cout << "Camera matrix: " << cameraMatrix << endl;
		cout << "Distortion _coefficients: " << distCoeffs << endl;

		saveparams(outDir + "DataCam.yml", cameraMatrix, distCoeffs,
		           rvecs, tvecs, rms);

		Mat imageUndistorted, image;
		VideoCapture capture = VideoCapture(0);
		namedWindow("imgOriginal", WINDOW_AUTOSIZE);
		namedWindow("imgCalibrada", WINDOW_AUTOSIZE);



		while (1) {

			capture >> image;
			undistort(image, imageUndistorted, cameraMatrix, distCoeffs);//corrigo distorsion radial

			imshow("imgOriginal", image);
			imshow("imgCalibrada", imageUndistorted);
			waitKey(200);
		}
		waitKey(0);
		getchar();
		return;
	}


};
}