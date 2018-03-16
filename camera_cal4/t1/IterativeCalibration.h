#pragma once
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
#include <thread>

#include "Utils.h"
#include "ProcManager.h"

using namespace cv;
using namespace std;


namespace vision
{
class IterativeCalibration
{
public:
	int type_calibration;
	Size pattern_size;
	float square_size_mm;


	IterativeCalibration(int type_calibration, float square_size_mm ) {

		this->type_calibration = type_calibration;
		this->square_size_mm = square_size_mm;

		if ( type_calibration ==  CONCENTRIC_CIRCLES)
		{
			pattern_size = Size(5 , 4);
		}
		else if ( type_calibration == CHESSBOARD )
		{
			pattern_size = Size(9 , 6);
		}
		else if ( type_calibration == ASYMMETRIC_CIRCLES_GRID )
		{
			pattern_size = Size( 4, 11 );
		}


	}

	//calibrar con opencv
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

		if ( type_calibration == CHESSBOARD )
		{
			for ( int i = 0; i < boardSize.height; i++ )
				for ( int j = 0; j < boardSize.width; j++ )
					corners.push_back(Point3f(float(j * squareSize), float(i * squareSize), 0));
		}
		else if ( type_calibration == CIRCLES_GRID)
		{
			for ( int i = 0; i < boardSize.height; ++i )
				for ( int j = 0; j < boardSize.width; ++j )
					corners.push_back(Point3f(j * squareSize, i * squareSize, 0));
		}
		else if ( type_calibration == ASYMMETRIC_CIRCLES_GRID)
		{
			for ( int i = 0; i < boardSize.height; i++ )
				for ( int j = 0; j < boardSize.width; j++ )
					corners.push_back(Point3f((2 * j + i % 2)*squareSize, i * squareSize, 0));
		}

		else if ( type_calibration == CONCENTRIC_CIRCLES)
		{
			for ( int i = 0; i < boardSize.height; i++ )
				for ( int j = 0; j < boardSize.width; j++ )
					corners.push_back(Point3f(float(j * squareSize), float(i * squareSize), 0));
		}
	}

	bool opencv_calibration( Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
	                         vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs,
	                         vector<Mat>& tvecs, vector<float>& reprojErrs, double& rms)
	{

		cameraMatrix = Mat::eye(3, 3, CV_64F);
		distCoeffs = Mat::zeros(8, 1, CV_64F);

		vector< vector<Point3f> > objectPoints(1);

		calcBoardCornerPositions(pattern_size , square_size_mm, objectPoints[0]);



		objectPoints.resize(imagePoints.size(), objectPoints[0]);


		calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);


		bool is_calibrate = checkRange(cameraMatrix) && checkRange(distCoeffs);

		rms = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

		return is_calibrate;
	}

	//------------------------------------------------------------------------------------------------------------------



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

			//Point2d pDistort =  distortPoint(projectCenters[i], K, distCoeffs);

			float x = (projectCenters[i].x - cx ) / fx;
			float y = (projectCenters[i].y - cy ) / fy;

			float r2 = x * x + y * y;

			Point2d pDistort;
			pDistort.x = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
			pDistort.y = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

			pDistort.x += 2 * p1 * x * y + p2 * (r2 * 2 * x * x);
			pDistort.y += p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;

			pDistort.x = pDistort.x * fx + cx;
			pDistort.y = pDistort.y * fy + cy;

			distortCenters.push_back(Point2f(pDistort.x, pDistort.y));
		}
		projectCenters = distortCenters;

	}

	void undistort_frames(vector<Mat>& frames_input, vector<Mat>& frames_output, Mat& cameraMatrix , Mat& distCoeffs)
	{
		frames_output.resize(frames_input.size());

		for (int i = 0; i < frames_input.size(); i++)
		{
			undistort(frames_input[i], frames_output[i], cameraMatrix, distCoeffs);
		}

	}

	bool find_control_points(Mat& input, vector<Point2f>& control_points )
	{

		bool found = false;
		if ( type_calibration ==  CONCENTRIC_CIRCLES)
		{
			ProcManager pm;
			found = pm.findConcentrics(input, control_points);
		}
		else if ( type_calibration == CHESSBOARD )
		{
			int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
			found = findChessboardCorners( input, pattern_size, control_points, chessBoardFlags);
			Mat viewGray;
			cvtColor(input, viewGray, COLOR_BGR2GRAY);
			cornerSubPix( viewGray, control_points, Size(11, 11), Size(-1, -1), TermCriteria( TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1 ));
		}
		else if ( type_calibration == ASYMMETRIC_CIRCLES_GRID )
		{
			found = findCirclesGrid( input, pattern_size, control_points, ASYMMETRIC_CIRCLES_GRID );
			//waitKey();

		}

		return found;
	}

	bool find_control_points_front_parallel( Mat& input, vector<Point2f>& control_points )
	{
		//TODO: antes de  retornar los puntos, se debe agregar la rectificacion por colinearidad
		return ( find_control_points(input, control_points) );
	}

	bool get_front_parallel(Mat& frame_input, Mat& frame_output, Mat& lambda)
	{
		vector<Point2f> control_points_input;

		float size_unit;
		float offset;

		bool found = find_control_points(frame_input, control_points_input);



		//si no encuentra los puntos no hay fronto paralelo
		if (!found) return false;



		int width = pattern_size.width;
		int height = pattern_size.height;
		Size size_out;

		vector<Point2f> src_quad(4), dest_quad(4);

		if ( type_calibration == CHESSBOARD  )
		{

			size_unit = frame_input.cols / 10.0f;
		 	offset = -size_unit / 3.8f;
			src_quad[0] = control_points_input[0];
			src_quad[1] = control_points_input[width - 1];
			src_quad[2] = control_points_input[width * height - 1];
			src_quad[3] = control_points_input[width * (height - 1)];

			dest_quad[0] = Point2f( size_unit + offset, size_unit + offset);
			dest_quad[1] = Point2f( width * size_unit + offset, size_unit + offset);
			dest_quad[2] = Point2f( width * size_unit + offset, height * size_unit + offset);
			dest_quad[3] = Point2f( size_unit + offset, height * size_unit + offset);

			size_out.width = dest_quad[1].x - dest_quad[0].x + 2 * (-offset) + size_unit;
			size_out.height = dest_quad[3].y - dest_quad[1].y + 2 * (-offset) + size_unit;
		}
		else if(type_calibration == CONCENTRIC_CIRCLES)
		{
			size_unit = frame_input.cols / 8.0f;
		 	offset = -size_unit / 3.8f;

		 	src_quad[0] = control_points_input[0];
			src_quad[1] = control_points_input[width - 1];
			src_quad[2] = control_points_input[width * height - 1];
			src_quad[3] = control_points_input[width * (height - 1)];

			dest_quad[0] = Point2f( size_unit + offset, size_unit + offset);
			dest_quad[1] = Point2f( width * size_unit + offset, size_unit + offset);
			dest_quad[2] = Point2f( width * size_unit + offset, height * size_unit + offset);
			dest_quad[3] = Point2f( size_unit + offset, height * size_unit + offset);

			size_out.width = dest_quad[1].x - dest_quad[0].x + 2 * (-offset) + size_unit;
			size_out.height = dest_quad[3].y - dest_quad[1].y + 2 * (-offset) + size_unit;
		}
		else if( type_calibration == ASYMMETRIC_CIRCLES_GRID )
		{

			size_unit = frame_input.cols / 16.0f;
			offset = size_unit / 3.8f;
			
			src_quad[0] = control_points_input[3];
			src_quad[1] = control_points_input[43];
			src_quad[2] = control_points_input[40];
			src_quad[3] = control_points_input[0];

			width = 11;
			height = 7;

			dest_quad[0] = Point2f( size_unit + offset, size_unit + 3*offset);
			dest_quad[1] = Point2f( width * size_unit + offset, size_unit + 3*offset);

			dest_quad[2] = Point2f( width * size_unit + offset, height * size_unit + 3*offset);
			dest_quad[3] = Point2f( size_unit + offset, height * size_unit + 3*offset);


			float sizeH = dest_quad[3].y - dest_quad[1].y;
			float sizeW = dest_quad[1].x - dest_quad[0].x;

			size_out.width = (10.0f/8.0f)*sizeW;
			size_out.height = (10.0f/7.0f)*sizeH ;
		}


		lambda = getPerspectiveTransform( src_quad, dest_quad );

		

		

		warpPerspective(frame_input, frame_output, lambda, size_out );

		//ProcManager pc;
		//pc.drawControlPointsCross(frame_input, src_quad );

		//imshow("input", frame_input);
		//imshow("fp", frame_output);
		//waitKey();



		return true;
	}

	//recibe una imagen rectificada, devuelve los puntos corregidos en fronto paralelo en el espacio de la imagen rectificada
	bool get_control_points_refine(Mat& frame_input, vector<Point2f>& control_points_refine, Mat& cameraMatrix, Mat& distCoeffs )
	{



		vector<Point2f> control_points_front_parallel;

		//obtener la imagen fronto paralela y la homografia lambda
		Mat front_parallel, lambda;
		bool found = get_front_parallel( frame_input, front_parallel, lambda );



		if (!found) return false;

		//buscar los puntos de control en la imagen fronto paralela
		found = find_control_points_front_parallel(front_parallel, control_points_front_parallel);
		if (!found) return false;


		//llevar los puntos de control del fronto paralelo hacia la imagen rectificada
		perspectiveTransform( control_points_front_parallel, control_points_refine, lambda.inv() );

		//distorsionar los puntos
		distControlPoints(control_points_refine, cameraMatrix, distCoeffs);


		//cout<<"jojo"<<endl;

		return true;

	}

	void readParameters(const string& path, Mat& cameraMatrix, Mat& distCoeffs, vector<Mat>& rvecs,  vector<Mat>& tvecs, double& rms)
	{
		FileStorage fs;
		fs.open(path.c_str(), FileStorage::READ);
		fs["Camera_Matrix"] >> cameraMatrix;
		fs["Distortion_Coefficients"] >> distCoeffs;
		fs["Rotation_Vector"] >> rvecs;
		fs["Translation_vector"] >> tvecs;
		fs["Calibrate_Accuracy"] >> rms;
		fs.release();
	}

	//recibe una calibracion inicial desde archivo y empieza con la calibracion iterativa
	void init_calibrate(const string& pathParameters, const string& pathFrames, const string& pathSave)
	{
		Mat cameraMatrix, distCoeffs;
		vector<Mat> rvecs, tvecs;
		double rms;
		vector<Mat> frames;
		readParameters(pathParameters, cameraMatrix, distCoeffs, rvecs, tvecs, rms);

		//leemos las  imagenes de la calibracion inicial
		for (int i = 0; i < tvecs.size(); i++)
		{
			Mat image = imread(pathFrames + "frame_" + to_string(i) + ".jpg");
			if (!image.data )
			{

				cout << "Image no read" << endl;
				return;
			}
			frames.push_back(image);
		}


		cout << "rms initial " << rms << endl;
		calibrate( cameraMatrix, distCoeffs, rvecs, tvecs, frames );

	}

	void calibrate(	Mat& cameraMatrix , Mat& distCoeffs, vector<Mat>& rvecs, vector<Mat>& tvecs, vector<Mat>& frames)
	{

		for ( int iteration = 0; iteration < 10; iteration++ )
		{

			vector<Mat> frames_undistorted;
			//corregimos las imagenes con los parametros de calibracion inical
			undistort_frames(frames, frames_undistorted, cameraMatrix, distCoeffs);


			vector< vector<Point2f> > image_points;
			vector<Mat> rechoiced_frames;

			//buscar los puntos de control refinados
			for (int i = 0; i < frames.size(); i++ )
			{

				vector< Point2f > control_points_refine;
				bool found = get_control_points_refine( frames_undistorted[i], control_points_refine, cameraMatrix, distCoeffs );


				if ( found )
				{
					rechoiced_frames.push_back(frames[i]);
					image_points.push_back( control_points_refine );
				}
			}

			//actualizar la lista de frames, solo por sea el caso algun frame se descarte por perdida de deteccion
			frames = rechoiced_frames;

			//calibramos con opencv
			vector<float> reprojErrs;
			double rms;


			bool is_calibrate = opencv_calibration(frames[0].size(), cameraMatrix, distCoeffs, image_points, rvecs, tvecs, reprojErrs, rms );



			cout << "iteration: " << iteration << " rms: " << rms << endl;

		}

	}
};
}