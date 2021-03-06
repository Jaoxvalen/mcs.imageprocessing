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

	CalibHandler(int pTypeCalib, Size pPatternSize, float pSquareSize, const string& pOutPutdir, const string& pInputVideodir)
	{
		mTypeCalib = pTypeCalib;
		mPatternSize = pPatternSize;
		mSquareSize = pSquareSize;
		key = -1;
		mOutPutdir = pOutPutdir;
		mInputVideodir = pInputVideodir;
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

		rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
		                      CALIB_FIX_PRINCIPAL_POINT +
		                      CALIB_FIX_ASPECT_RATIO +
		                      CALIB_ZERO_TANGENT_DIST);

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

		Mat view, auxView;
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
						imagePoints.push_back(pointBuf);
						nImgAdded++;
						cout << "image frame added " << nImg << endl;
						cout << "image to calibration added " << nImgAdded << endl;
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