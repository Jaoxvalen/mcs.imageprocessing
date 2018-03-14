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

	void readParameters(const string& path, Mat& cameraMatrix, Mat& distCoeffs, vector<Mat>& rvecs,  vector<Mat>& tvecs)
	{
		FileStorage fs;
		fs.open(path.c_str(), FileStorage::READ);
		fs["Camera_Matrix"] >> cameraMatrix;
		fs["Distortion_Coefficients"] >> distCoeffs;
		fs["Rotation_Vector"] >> rvecs;
		fs["Translation_vector"] >> tvecs;
		fs.release();
	}

	bool calculatePoints(Mat& input, Size mPatternSize, int type , Size& sizeOut, vector<Point2f> &inputQuad, vector<Point2f> &outputQuad, float& offset)
	{
		/*hallando esquinas*/
		vector<Point2f> pointBuf;
		Mat view = input;
		int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
		float size;// = 100;
		//float offset;// = size/(2.5);//50;

		bool found = false;

		switch (type) {
		case CHESSBOARD :
			//chessboard
			//cout<<"mPatternSize : "<<mPatternSize<<endl;
			found = findChessboardCorners( view, mPatternSize, pointBuf, chessBoardFlags);
			size = input.cols / 10.0f;
			offset = -size / (3.8);
			break;

		case CONCENTRIC_CIRCLES :
			//rings
			ProcManager concentricHand2;
			Mat auxView = input;
			found = concentricHand2.findConcentrics(input, pointBuf, auxView);
			//cout<<"pointBuf[0]"<<pointBuf[0]<<endl;
			size = input.cols / 8.0f;
			offset = -size / 3.8f;


			break;
		}

		//se nao pode-se encontrar os pontos de control entao retornar false
		if (!found) return false;


		int width = mPatternSize.width;
		int height = mPatternSize.height;

		//cout<<"pointBuf[0]"<<pointBuf[0]<<endl;
		//cout<<"pointBuf[1]"<<pointBuf[width-1]<<endl;
		//cout<<"pointBuf[2]"<<pointBuf[width*height -1]<<endl;
		//cout<<"pointBuf[3]"<<pointBuf[width*(height-1)]<<endl;
		//cout<<"size of pointBuf : "<< pointBuf.size()<<endl;

		inputQuad[0] = pointBuf[0];
		inputQuad[1] = pointBuf[width - 1];
		inputQuad[2] = pointBuf[width * height - 1];
		inputQuad[3] = pointBuf[width * (height - 1)];

		/*
		for( int i = 0; i<4; i++ )
		{
			circle(input, inputQuad[i], 5, Scalar(255,0,0), 1,8);
		}*/


		// The 4 points where the mapping is to be done , from top-left in clockwise order

		outputQuad[0] = Point2f( size + offset, size + offset);
		outputQuad[1] = Point2f( width * size + offset, size + offset);
		outputQuad[2] = Point2f( width * size + offset, height * size + offset);
		outputQuad[3] = Point2f( size + offset, height * size + offset);

		sizeOut.width = outputQuad[1].x - outputQuad[0].x + 2 * (-offset) + size;
		sizeOut.height = outputQuad[3].y - outputQuad[1].y + 2 * (-offset) + size;


		return true;

		/*
		for ( int i = 0; i < pointBuf.size(); i++ )
		{
			circle(input, pointBuf[i], 1, Scalar(0, 0, 255), 2, 8);
		}
		*/
	}


	Point2d distortPoint(Point2d undistorted_point, Mat camera_matrix, vector<double> distort_coefficients) {

		// Check that camera matrix is double
		if (!(camera_matrix.type() == CV_64F || camera_matrix.type() == CV_64FC1)) {
			std::ostringstream oss;
			oss << "distortPoint(): Camera matrix type is wrong. It has to be a double matrix (CV_64)";
			throw std::runtime_error(oss.str());
		}

		// Create distorted point
		cv::Point2d distortedPoint;
		distortedPoint.x = (undistorted_point.x - camera_matrix.at<double>(0, 2)) / camera_matrix.at<double>(0, 0);
		distortedPoint.y = (undistorted_point.y - camera_matrix.at<double>(1, 2)) / camera_matrix.at<double>(1, 1);

		// Get model
		if (distort_coefficients.size() < 4 || distort_coefficients.size() > 8 ) {
			throw std::runtime_error("distortPoint(): Invalid numbrer of distortion coefficitnes.");
		}
		double k1(distort_coefficients[0]);
		double k2(distort_coefficients[1]);
		double p1(distort_coefficients[2]);// tangent distortion first coeficinet
		double p2(distort_coefficients[3]);// tangent distortion second coeficinet
		double k3(0);
		double k4(0);
		double k5(0);
		double k6(0);
		if (distort_coefficients.size() > 4)
			k3 = distort_coefficients[4];
		if (distort_coefficients.size() > 5)
			k4 = distort_coefficients[5];
		if (distort_coefficients.size() > 6)
			k5 = distort_coefficients[6];
		if (distort_coefficients.size() > 7)
			k6 = distort_coefficients[7];

		// Distort
		double xcx = distortedPoint.x;
		double ycy = distortedPoint.y;
		double r2 = pow(xcx, 2) + pow(ycy, 2);
		double r4 = pow(r2, 2);
		double r6 = pow(r2, 3);
		double k = (1 + k1 * r2 + k2 * r4 + k3 * r6) / (1 + k4 * r2 + k5 * r4 + k5 * r6);
		distortedPoint.x = xcx * k + 2 * p1 * xcx * ycy + p2 * (r2 + 2 * pow(xcx, 2));
		distortedPoint.y = ycy * k + p1 * (r2 + 2 * pow(ycy, 2)) + 2 * p2 * xcx * ycy;
		distortedPoint.x = distortedPoint.x * camera_matrix.at<double>(0, 0) + camera_matrix.at<double>(0, 2);
		distortedPoint.y = distortedPoint.y * camera_matrix.at<double>(1, 1) + camera_matrix.at<double>(1, 2);

		// Exit
		return distortedPoint;
	}


	void distControlPoints( vector<Point2f>& projectCenters, Mat& K, Mat distCoeffs )
	{
		double fx = K.at<double>(0, 0);
		double cx = K.at<double>(0, 2);
		double fy = K.at<double>(1, 1);
		double cy = K.at<double>(1, 2);

		double k1 = distCoeffs.at<double>(0, 0);
		double k2 = distCoeffs.at<double>(1, 0);
		double p1 = distCoeffs.at<double>(2, 0);
		double p2 = distCoeffs.at<double>(3, 0);
		double k3 = distCoeffs.at<double>(4, 0);

		vector<Point2f> distortCenters;

		for ( int i = 0 ; i < projectCenters.size(); i++ )
		{

			Point2d pDistort =  distortPoint(projectCenters[i], K, distCoeffs);
			distortCenters.push_back(Point2f(pDistort.x, pDistort.y));
		}
		projectCenters = distortCenters;

	}

	void getImagesUndistorted(const string& pathFrames, vector<Mat>& frames, Mat& cameraMatrix, Mat& distCoeffs, unsigned int nImages)
	{

		for (int i = 0; i < nImages; i++)
		{
			Mat image = imread(pathFrames + "frame_" + to_string(i) + ".jpg");
			Mat im2 = image.clone();
			if (!image.data )
			{

				cout << "Image no read" << endl;
				return;
			}

			undistort(image, im2, cameraMatrix, distCoeffs);

			//remap(image, image, map1, map2, INTER_LINEAR);
			frames.push_back(im2);
		}
	}

	bool getRefineFrameControlPoints(int nFrame, vector<Mat> frames ,
	                                 const string& pathFrames, Size mPatternSize , int type_choose, Mat& cameraMatrix,
	                                 Mat& distCoeffs , vector<Point2f>& controlPoints)
	{
		Mat input = frames[nFrame];
		Mat original_image = imread(pathFrames + "frame_" + to_string(nFrame) + ".jpg");
		Mat alt = original_image.clone();
		Mat output, lambda;
		vector<Point2f> inputQuad(4);
		vector<Point2f> outputQuad(4);

		//imshow("image", original_image);
		//waitKey();

		Size outSize;
		float offset;

		bool found =  calculatePoints(input, mPatternSize, type_choose, outSize, inputQuad, outputQuad, offset);

		if (!found) return false;

		lambda = getPerspectiveTransform( inputQuad, outputQuad );

		warpPerspective(input, output, lambda, outSize );

		found = false;
		vector<Point2f> pointBuf, pointBufCorrec;

		if (type_choose == CHESSBOARD)
		{
			int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
			found = findChessboardCorners( input, mPatternSize, pointBuf, chessBoardFlags);
		}
		else if ( type_choose == CONCENTRIC_CIRCLES )
		{
			ProcManager manager;
			found = manager.findConcentrics(input, pointBuf, alt);
		}

		if (!found) return false;

		/*
		for ( int i = 0; i < pointBuf.size(); i++ )
		{
			circle(input, pointBuf[i], 1, Scalar(255, 0, 255), 2, 8);
		}
		*/

		if (type_choose == CHESSBOARD)
		{
			int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
			found = findChessboardCorners( output, mPatternSize, pointBuf, chessBoardFlags);
			drawChessboardCorners( output, mPatternSize, Mat(pointBuf), found );
		}
		else if (type_choose == CONCENTRIC_CIRCLES)
		{
			ProcManager manager;
			found = manager.findConcentrics(output, pointBuf, alt);
		}

		if (!found) return false;

		//regresar los puntos a la corregida
		pointBufCorrec.resize( pointBuf.size() );
		perspectiveTransform( pointBuf, pointBufCorrec, lambda.inv() );


		/*
		for ( int i = 0; i < pointBufCorrec.size(); i++ )
		{
			circle(input, pointBufCorrec[i], 1, Scalar(0, 255, 255), 1, 8);
		}*/

		//fin regresar los puntos

		for ( int i = 0; i < pointBufCorrec.size(); i++ )
		{
			circle(original_image, pointBufCorrec[i], 1, Scalar(0, 0, 255), 2, 8);
		}


		distControlPoints(pointBufCorrec, cameraMatrix, distCoeffs);


		for ( int i = 0; i < pointBufCorrec.size(); i++ )
		{
			circle(original_image, pointBufCorrec[i], 1, Scalar(255, 0, 0), 2, 8);
		}


		//cout << "FOUND " << found << endl;

		controlPoints = pointBufCorrec;


		//imshow("original_image", original_image);
		//waitKey();
		//imshow("undistorted_image", input);
		//imshow("fronto_parallel", output);


		return true;
	}

	bool runIterativeCalibration( Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
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

		cout << "Camera matrix: " << cameraMatrix << endl;
		cout << "Distortion _coefficients: " << distCoeffs << endl;
		cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

		bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

		totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

		return ok;
	}

	bool runIterativeCalibrationAndSave(Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
	                                    vector<vector<Point2f> > imagePoints)
	{
		vector<Mat> rvecs, tvecs;
		vector<float> reprojErrs;
		double totalAvgErr = 0;


		bool ok = runIterativeCalibration(imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs, totalAvgErr);

		cout << (ok ? "Calibration succeeded" : "Calibration failed") << ". avg re projection error = " << totalAvgErr << endl;
		return ok;
	}

	void refineControlPoints(const string& pathParameters, const string& pathFrames, Size mPatternSize , int type_choose, float squareSize)
	{

		//int nFrame = 34;
		int key;
		Mat cameraMatrix;
		Mat distCoeffs;
		vector<Mat> rvecs;
		vector<Mat> tvecs;
		vector<Mat> frames, frameUndist, frames_iterative;


		//leer los parametros iniciales
		readParameters(pathParameters, cameraMatrix, distCoeffs, rvecs, tvecs);


		getImagesUndistorted(pathFrames, frames, cameraMatrix, distCoeffs, rvecs.size());


		//seteamos los valores generales para la calibracions
		this->mTypeCalib = type_choose;
		this->mPatternSize = mPatternSize;
		this->mSquareSize = squareSize;

		frames_iterative = frames;
		vector<unsigned int> indexs;

		for (int i = 0; i < rvecs.size(); i++)
		{
			indexs.push_back(i);
		}


		for ( int iter = 0; iter < 1000; iter++ ) //iterations
		{

			vector<unsigned int> indexs_temp;
			vector< vector<Point2f> >controlPointsRefineAll;
			for ( int i = 0 ; i < frames.size(); i++ ) //frames
			{
				vector<Point2f> controlPointsRefine;
				bool found = getRefineFrameControlPoints(indexs[i], frames_iterative, pathFrames, mPatternSize, type_choose, cameraMatrix, distCoeffs, controlPointsRefine);

				if (found)
				{
					indexs_temp.push_back(indexs[i]);
					controlPointsRefineAll.push_back(controlPointsRefine);
				}

			}


			Mat imgclone = imread(pathFrames + "frame_" + to_string(indexs[0]) + ".jpg");
			for ( int l = 0; l < controlPointsRefineAll[0].size(); l++ )
			{
				circle(imgclone, controlPointsRefineAll[0][l], 1, Scalar(0, 255, 255), 1, 8);
			}

			imshow("frame: ", imgclone);
			waitKey();

			indexs = indexs_temp;

			cout << indexs.size() << endl;

			runIterativeCalibrationAndSave(frames[0].size(), cameraMatrix, distCoeffs, controlPointsRefineAll);

			vector<Mat> framesTemp;
			for (int f = 0; f < indexs.size(); f++)
			{
				framesTemp.push_back( frames[indexs[f] ]);
				//cout<<"agregando: "<<indexs[f]<<endl;
			}
			frames = framesTemp;
		}
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
					found = concentricHand.findConcentrics(view, pointBuf, auxView);
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

							foundA = concentricHandA.findConcentrics(view, pointBufA, view);
							foundB = concentricHandB.findConcentrics(rview, pointBufB, rview);
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