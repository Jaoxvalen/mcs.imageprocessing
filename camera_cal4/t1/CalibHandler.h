#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "ProcManager.h"

using namespace cv;
using namespace std;

enum STATECALIB
{
	STATE_FINISH,
	STATE_CAPTURE_IMAGES,
	STATE_CALIBRATION,
	STATE_CALIBRATED,
	STATE_SHOW_UNDISTORT
};

enum TYPECALIB
{
	CHESSBOARD,
	CIRCLES_GRID,
	ASYMMETRIC_CIRCLES_GRID,
	CONCENTRIC_CIRCLES
};


namespace vision
{
class CalibHandler
{
public:

	int STATE;
	int key;

	int mTypeCalib;
	Size mPatternSize;
	float mSquareSize;
	string mOutPutdir;
	string mInputVideodir;
	ProcManager concentricHand;

	CalibHandler()
	{

	}

	CalibHandler(int pTypeCalib, Size pPatternSize, float pSquareSize, const string& pOutPutdir, const string& pInputVideodir)
	{
		mTypeCalib = pTypeCalib;
		mPatternSize = pPatternSize;
		mSquareSize = pSquareSize;
		key = -1;
		mOutPutdir = pOutPutdir;
		mInputVideodir = pInputVideodir;
	}


	void readParameters(const string& path, Mat& cameraMatrix, Mat& distCoeffs, vector<Mat>& rvecs, vector<Mat>& tvecs)
	{

		//read parameters
		FileStorage fs;
		fs.open(path.c_str(), FileStorage::READ);
		fs["Camera_Matrix"] >> cameraMatrix;
		fs["Distortion_Coefficients"] >> distCoeffs;
		fs["Rotation_Vector"] >> rvecs;
		fs["Translation_vector"] >> tvecs;
		fs.release();
	}

	void transFrontoParallel(const string& pathParameters, const string& pathFrames)
	{
		int key;
		Mat cameraMatrix;
		Mat distCoeffs;
		vector<Mat> rvecs;
		vector<Mat> tvecs;
		readParameters(pathParameters, cameraMatrix, distCoeffs, rvecs, tvecs);
		vector<Mat> frames, frameUndist;

		//FOR REMAP
		Mat map1, map2;


		//leer y undistort para la imagen 0
		Mat image = imread(pathFrames + "frame_0.jpg");
		if (!image.data )
		{
			cout << "Image no read" << endl;
			return;
		}
		initUndistortRectifyMap( cameraMatrix, distCoeffs, Mat(), Mat(), image.size(), CV_16SC2, map1, map2);
		remap(image, image, map1, map2, INTER_LINEAR);
		frames.push_back(image);

		//para el resto de imagenes
		for (int i = 1; i < rvecs.size(); i++)
		{

			image = imread(pathFrames + "frame_" + to_string(i) + ".jpg");

			if (!image.data )
			{
				cout << "Image no read" << endl;
				return;
			}
			remap(image, image, map1, map2, INTER_LINEAR);
			frames.push_back(image);
		}


		double fx = cameraMatrix.at<double>(0, 0);
		double cx = cameraMatrix.at<double>(0, 2);
		double fy = cameraMatrix.at<double>(1, 1);
		double cy = cameraMatrix.at<double>(1, 2);

		Mat rotMatrix, invRotMatrix;

		int nFrame = 23;

		Rodrigues(rvecs[nFrame], rotMatrix);
		invRotMatrix = rotMatrix.inv();

		

		image = frames[nFrame];
		//Mat result = image.clone();
		Mat result(image.rows, image.cols, CV_8UC3, Scalar(0, 0, 0));

		imshow("original", image);

		for ( int u = 0; u < image.rows; u++ )
		{
			for (int v = 0; v < image.cols; v++)
			{

				Point3d pA, pB, pDir, pR;
				//calculamos x' e y' para z = 1
				pA.x = (u - cx) / fx;
				pA.y = (v - cy) / fy;
				pA.z = 1.0f;
				//calculamos x' e y' para z = 2
				pB.x = 2 * (u - cx) / fx;
				pB.y = 2 * (v - cy) / fy;
				pB.z = 2.0f;

				Mat mPA = (Mat_<double>(3, 1) << pA.x, pA.y, pA.z);
				Mat mPB = (Mat_<double>(3, 1) << pB.x, pB.y, pB.z);

				//debemos aplicar la inversa de la rotacion a ambos puntos
				mPA = invRotMatrix * mPA;
				mPB = invRotMatrix * mPB;

				//ahora calculamos para Z = 0 en el nuevo sistema de coordenadas

				pA.x = mPA.at<double>(0, 0);
				pA.y = mPA.at<double>(1, 0);
				pA.z = mPA.at<double>(2, 0);

				pB.x = mPB.at<double>(0, 0);
				pB.y = mPB.at<double>(1, 0);
				pB.z = mPB.at<double>(2, 0);

				pDir = pB - pA;

				//calculamos el punto para un t = -1
				pR = pA - pDir*(-1.0f);

				//calculamos el nuevo U,V
				float uC = (fx * pR.x + cx)+200;
				float vC = (fy * pR.y + cy)-500;


				if (uC >= 0 && uC < image.rows && vC >= 0 && vC < image.cols)
				{
					Vec3b color = image.at<Vec3b>(uC, vC);
					result.at<Vec3b>(u, v) = color;
				}
			}
		}

		imshow("result", result);


		key = waitKey();

	}


	float AVGCheckEndColinearity(const vector< cv::Point2f >& points)
	{

		float avg = 0.0f;
		for ( int i = 0; i < mPatternSize.height; i++ )
		{
			vector<Point2f> pointsSel;
			for ( int j = 0; j < mPatternSize.width; j++ )
			{
				//j,i
				pointsSel.push_back(points[ mPatternSize.width * i + j ] );
			}
			avg += checkEnd2EndColinearity(pointsSel);
		}

		avg = avg / mPatternSize.height;

	}
	float checkEnd2EndColinearity( const vector< cv::Point2f >& points )
	{
		// Get line parameters - compute equation ax + by + c = 0
		/* Given pStart and pEnd, we have
		*   a = ( yEnd - yStart )
		*   b = ( xStart - xEnd )
		*   c = ( xEnd * yStart - xStart * yEnd )
		*   dist( p, Line ) = | a * px + b * py + c | / sqrt( a^2 + b^2 )
		*/
		float _xStart = points[0].x;
		float _yStart = points[0].y;

		float _xEnd = points[ points.size() - 1 ].x;
		float _yEnd = points[ points.size() - 1 ].y;

		float _a = _yEnd - _yStart;
		float _b = _xStart - _xEnd;
		float _c = _xEnd * _yStart - _xStart * _yEnd;

		float _divider = sqrt( _a * _a + _b * _b );

		float _distCum = 0.0f;

		for ( int q = 1; q < points.size() - 1; q++ )
		{
			_distCum += abs( _a * points[q].x + _b * points[q].y + _c ) / _divider;
		}

		return _distCum / ( points.size() - 2 );
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

	bool runCalibrationAndSave(Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
	                           vector<vector<Point2f> > imagePoints)
	{
		vector<Mat> rvecs, tvecs;
		vector<float> reprojErrs;
		double totalAvgErr = 0;


		bool ok = runCalibration(imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs, totalAvgErr);


		cout << (ok ? "Calibration succeeded" : "Calibration failed") << ". avg re projection error = " << totalAvgErr << endl;

		if (ok)
		{
			saveparams(mOutPutdir, cameraMatrix, distCoeffs, rvecs, tvecs,  totalAvgErr);
		}
		return ok;
	}

	static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
	        const vector<vector<Point2f> >& imagePoints,
	        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	        const Mat& cameraMatrix , const Mat& distCoeffs,
	        vector<float>& perViewErrors)
	{
		vector<Point2f> imagePoints2;
		size_t totalPoints = 0;
		double totalErr = 0, err;
		perViewErrors.resize(objectPoints.size());

		for (size_t i = 0; i < objectPoints.size(); ++i )
		{

			projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);

			err = norm(imagePoints[i], imagePoints2, NORM_L2);

			size_t n = objectPoints[i].size();
			perViewErrors[i] = (float) std::sqrt(err * err / n);
			totalErr        += err * err;
			totalPoints     += n;
		}

		return std::sqrt(totalErr / totalPoints);
	}

	void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners)
	{
		corners.clear();

		if ( mTypeCalib == CHESSBOARD )
		{
			for ( int i = 0; i < boardSize.height; i++ )
				for ( int j = 0; j < boardSize.width; j++ )
					corners.push_back(Point3f(float(j * squareSize), float(i * squareSize), 0));
		}
		else if ( mTypeCalib == CIRCLES_GRID)
		{
			for ( int i = 0; i < boardSize.height; ++i )
				for ( int j = 0; j < boardSize.width; ++j )
					corners.push_back(Point3f(j * squareSize, i * squareSize, 0));
		}
		else if ( mTypeCalib == ASYMMETRIC_CIRCLES_GRID)
		{
			for ( int i = 0; i < boardSize.height; i++ )
				for ( int j = 0; j < boardSize.width; j++ )
					corners.push_back(Point3f((2 * j + i % 2)*squareSize, i * squareSize, 0));
		}

		else if ( mTypeCalib == CONCENTRIC_CIRCLES)
		{
			for ( int i = 0; i < boardSize.height; i++ )
				for ( int j = 0; j < boardSize.width; j++ )
					corners.push_back(Point3f(float(j * squareSize), float(i * squareSize), 0));
		}
	}

	bool runCalibration( Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
	                     vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
	                     vector<float>& reprojErrs,  double& totalAvgErr)
	{



		cameraMatrix = Mat::eye(3, 3, CV_64F);

		distCoeffs = Mat::zeros(8, 1, CV_64F);

		vector< vector<Point3f> > objectPoints(1);

		calcBoardCornerPositions(mPatternSize , mSquareSize, objectPoints[0]);

		objectPoints.resize(imagePoints.size(), objectPoints[0]);

		double rms;

		rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);
		//CALIB_FIX_PRINCIPAL_POINT +
		//CALIB_FIX_ASPECT_RATIO +
		//CALIB_ZERO_TANGENT_DIST

		cout << "Camera matrix: " << cameraMatrix << endl;
		cout << "Distortion _coefficients: " << distCoeffs << endl;
		cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

		bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

		totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
		                                        distCoeffs, reprojErrs);

		return ok;
	}




	void calibration()
	{
		//init state capture
		STATE = STATE_CAPTURE_IMAGES;

		Mat view, auxView, temp;
		vector<vector<Point2f> > imagePoints;
		Mat cameraMatrix, distCoeffs;

		//FOR REMAP
		Mat rview, map1, map2;


		const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
		int ESC_KEY = 27;
		int NEXT_KEY = 83;
		int ENTER_KEY = 13;
		int SPACE_KEY = 32;

		int nImg = 0;
		int nImgAdded = 0;

		VideoCapture cam;

		cam = VideoCapture(mInputVideodir.c_str());
		cam >> view;

		//
		//area = Mat(view.size());
		Mat area(view.rows, view.cols, CV_8UC3, Scalar(0, 0, 0));

		if (!cam.isOpened()) {
			cout << "Error: not open file " << mInputVideodir << endl;
			getchar();
			return;
		}

		//bucle
		while (STATE != STATE_FINISH)
		{


			if (STATE == STATE_CAPTURE_IMAGES)
			{

				if (key == NEXT_KEY)
				{
					cam >> view;

					nImg++;
				}
				auxView = view.clone();
				temp = view.clone();


				vector<Point2f> pointBuf;
				bool found;
				int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

				if (mTypeCalib == CHESSBOARD)
				{
					found = findChessboardCorners( view, mPatternSize, pointBuf, chessBoardFlags);
				}
				else if ( mTypeCalib == CIRCLES_GRID )
				{
					found = findCirclesGrid( view, mPatternSize, pointBuf );
				}
				else if ( mTypeCalib == ASYMMETRIC_CIRCLES_GRID )
				{
					found = findCirclesGrid( view, mPatternSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID );

					/*
					if (found)
					{
						RotatedRect rt;
						rt.size.width = 10;
						rt.size.height = 10;

						rt.center.x = pointBuf[0].x;
						rt.center.y = pointBuf[0].y;
						ellipse( temp, rt , Scalar(0, 255, 0) , 1, 8 );
						rt.center.x = pointBuf[1].x;
						rt.center.y = pointBuf[1].y;
						ellipse( temp, rt , Scalar(0, 255, 0) , 1, 8 );
						rt.center.x = pointBuf[2].x;
						rt.center.y = pointBuf[2].y;
						ellipse( temp, rt , Scalar(0, 255, 0) , 1, 8 );
						rt.center.x = pointBuf[3].x;
						rt.center.y = pointBuf[3].y;
						ellipse( temp, rt , Scalar(0, 255, 0) , 1, 8 );
						rt.center.x = pointBuf[4].x;
						rt.center.y = pointBuf[4].y;
						ellipse( temp, rt , Scalar(0, 255, 0) , 1, 8 );
						rt.center.x = pointBuf[5].x;
						rt.center.y = pointBuf[5].y;
						ellipse( temp, rt , Scalar(0, 255, 0) , 1, 8 );

						imshow("temp", temp);

					}*/

				}
				else if ( mTypeCalib == CONCENTRIC_CIRCLES )
				{
					found = concentricHand.findConcentrics(auxView, pointBuf);
				}
				else
				{
					found = false;
				}
				if ( found )
				{
					if ( mTypeCalib == CHESSBOARD)
					{
						Mat viewGray;
						cvtColor(view, viewGray, COLOR_BGR2GRAY);
						cornerSubPix( viewGray, pointBuf, Size(11, 11),
						              Size(-1, -1), TermCriteria( TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1 ));
					}

					if ( mTypeCalib != CONCENTRIC_CIRCLES )
					{
						//imshow("Image View alt", auxView);
						drawChessboardCorners( auxView, mPatternSize, Mat(pointBuf), found );
					}


					if (key == ENTER_KEY)
					{

						//cout<<pointBuf.size()<<endl;

						imwrite( "../res/images/calibration/frames/frame_" + to_string(nImgAdded) + ".jpg", view );
						imagePoints.push_back(pointBuf);
						nImgAdded++;
						cout << "image frame added " << nImg << endl;
						cout << "image to calibration added " << nImgAdded << endl;


						RotatedRect rt;
						rt.size.width = 10;
						rt.size.height = 10;

						for (int i = 0; i < pointBuf.size(); i++)
						{
							rt.center.x = pointBuf[i].x;
							rt.center.y = pointBuf[i].y;
							ellipse( area, rt , Scalar(0, 255, 0) , 1, 8 );
						}

						imshow("area", area);

					}
				}
				if (auxView.data)
				{
					imshow("Image View", auxView);
				}

				if ( key == SPACE_KEY )
				{
					STATE = STATE_CALIBRATION;
				}



			}


			if ( STATE == STATE_CALIBRATION )
			{
				cvDestroyWindow("Image View");
				cout << "calibrating..." << endl;

				runCalibrationAndSave(view.size(),  cameraMatrix, distCoeffs, imagePoints);
				STATE = STATE_CALIBRATED;
			}

			if ( STATE == STATE_CALIBRATED )
			{
				//calculate the rectify map
				cout << "calculate the rectify map" << endl;
				initUndistortRectifyMap(
				    cameraMatrix, distCoeffs, Mat(),
				    Mat(), view.size(),
				    CV_16SC2, map1, map2);

				STATE = STATE_SHOW_UNDISTORT;
			}

			if ( STATE == STATE_SHOW_UNDISTORT )
			{
				cout << "showing the undistorted" << endl;
				cam = VideoCapture(mInputVideodir.c_str());

				cam >> view;

				if (view.empty())
				{
					break;
				}

				remap(view, rview, map1, map2, INTER_LINEAR);
				imshow("Image undistorted View", rview);
				imshow("Image View", view);

				for ( ; ; )
				{
					if (key == NEXT_KEY)
					{
						cam >> view;
						remap(view, rview, map1, map2, INTER_LINEAR);
						//Mat viewAu = view.clone();
						//Mat rviewAu = rview.clone();
						//imshow("Image undistorted View", rview);
						//imshow("Image View", view);


						vector<Point2f> pointBufA, pointBufB;
						int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
						bool foundA = false;
						bool foundB = false;

						if (mTypeCalib == CHESSBOARD)
						{
							foundA = findChessboardCorners( view, mPatternSize, pointBufA, chessBoardFlags);
							foundB = findChessboardCorners( rview, mPatternSize, pointBufB, chessBoardFlags);
						}
						else if ( mTypeCalib == CIRCLES_GRID )
						{
							foundA = findCirclesGrid( view, mPatternSize, pointBufA );
							foundB = findCirclesGrid( rview, mPatternSize, pointBufB );
						}
						else if ( mTypeCalib == ASYMMETRIC_CIRCLES_GRID )
						{
							foundA = findCirclesGrid( view, mPatternSize, pointBufA, CALIB_CB_ASYMMETRIC_GRID );
							foundB = findCirclesGrid( rview, mPatternSize, pointBufB, CALIB_CB_ASYMMETRIC_GRID );
						}
						else if ( mTypeCalib == CONCENTRIC_CIRCLES )
						{
							ProcManager concentricHandA;
							ProcManager concentricHandB;

							foundA = concentricHandA.findConcentrics(view, pointBufA);
							foundB = concentricHandB.findConcentrics(rview, pointBufB);
						}
						else
						{
							foundA = false;
							foundB = false;
						}

						if (foundA && foundB)
						{
							if ( mTypeCalib != CONCENTRIC_CIRCLES )
							{
								drawChessboardCorners( view, mPatternSize, Mat(pointBufA), foundA );
								drawChessboardCorners( rview, mPatternSize, Mat(pointBufB), foundB );

							}

							float colinearityA = AVGCheckEndColinearity(pointBufA);
							float colinearityB = AVGCheckEndColinearity(pointBufB);
							cout << " colinearity original: " << colinearityA << endl;
							cout << " colinearity undistorted: " << colinearityB << endl;

						}
						else
						{
							cout << "pattern not found!!!" << endl;
						}


						imshow("Image undistorted View", rview);
						imshow("Image View", view);



					}
					if (key == ESC_KEY)
					{
						break;
					}


					if (view.empty())
					{
						break;
					}



					key = waitKey();
				}

				STATE = STATE_FINISH;


			}


			if (key == ESC_KEY)
			{
				STATE = STATE_FINISH;
				cout << "esc key by user" << endl;
				break;
			}

			key = waitKey();


		}
	}
};
}