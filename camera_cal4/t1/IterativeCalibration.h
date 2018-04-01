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
			pattern_size = Size(7 , 5);
		}
		else if ( type_calibration == ASYMMETRIC_CIRCLES_GRID )
		{
			pattern_size = Size( 4, 11 );
		}


	}

	//metricas

	float distance(const Point2f& A, const Point2f& B)
	{
		return sqrt( pow(A.x - B.x, 2) + pow (A.y - B.y, 2) );
	}

	float checkMin2Colinearity(vector<Point2f>& control_points)
	{

		vector<Point2f> control_points_refined;
		refine_control_points_colinearity(control_points, control_points_refined);
		float dist = 0.0f;

		for (int i = 0; i < control_points_refined.size(); i++)
		{
			dist += distance(control_points_refined[i], control_points[i]);
		}
		dist /= ( (float) (pattern_size.height *  pattern_size.width) );

		return dist;

	}

	//recibe frames no distorsionados y devuelve la colinearidad media entre los frames
	float AVGcolinearityFrames(vector<Mat>& frames_undistorted)
	{
		float avg = 0.0f;
		int count = 0;
		for (int i = 0; i < frames_undistorted.size(); i++)
		{
			vector<Point2f> control_points;
			if ( find_control_points(frames_undistorted[i], control_points) )
			{
				avg += AVGCheckEndColinearity( control_points, frames_undistorted[i] );
				count++;
			}
		}
		avg = avg / ((float) count);
		return avg;
	}

	//recibe los puntos de control encontrado en el patron
	float AVGCheckEndColinearity(vector< cv::Point2f >& points, Mat& image)
	{

		float avg = 0.0f;
		//horizontales
		for ( int i = 0; i < pattern_size.height; i++ )
		{
			vector<Point2f> pointsSel;
			for ( int j = 0; j < pattern_size.width; j++ )
			{
				pointsSel.push_back(points[ pattern_size.width * i + j ] );
			}

			//ProcManager pm;
			//pm.drawControlPointsCross(image, pointsSel);
			//imshow("image", image);
			//waitKey();

			avg += checkEnd2EndColinearity(pointsSel);
		}


		if (type_calibration == CHESSBOARD || type_calibration == CONCENTRIC_CIRCLES)
		{
			//verticales
			for ( int i = 0; i < pattern_size.width; i++ )
			{
				vector<Point2f> pointsSel;
				for ( int j = 0; j < pattern_size.height; j++ )
				{
					pointsSel.push_back(points[ j * pattern_size.width + i ] );
				}


				//ProcManager pm;
				//pm.drawControlPointsCross(image, pointsSel);
				//imshow("image", image);
				//waitKey();

				avg += checkEnd2EndColinearity(pointsSel);
			}

			avg = avg / ( (float) (pattern_size.height +  pattern_size.width) );
		}
		else
		{
			avg = avg / ( (float) (pattern_size.height ) );
		}





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


			//pDistort.x = x; //* (1 + k1 * r2 );
			//pDistort.y = y; //* (1 + k1 * r2 );

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

		frames_output.clear();
		frames_output.resize(frames_input.size());

		for (int i = 0; i < frames_input.size(); i++)
		{
			undistort(frames_input[i], frames_output[i], cameraMatrix, distCoeffs);

			//imshow("distort", frames_input[i]);
			//imshow("un_distort", frames_output[i]);
			//waitKey();
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

			if (!found) return false;

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
		if (type_calibration == CONCENTRIC_CIRCLES || type_calibration == CHESSBOARD)
		{
			bool found = find_control_points(input, control_points);
			if (!found) return false;

			vector<Point2f> refined_colinearity;
			refine_control_points_colinearity( control_points, refined_colinearity );
			control_points = refined_colinearity;
			return found;

		}
		else
		{
			bool found = find_control_points(input, control_points);
			return found;
			//return ( find_control_points(input, control_points) );
		}

	}

	bool get_front_parallel(Mat& frame_input, Mat& frame_output, Mat& lambda)
	{
		vector<Point2f> control_points_input;

		float size_unit;
		float offset;

		bool found = find_control_points(frame_input, control_points_input);

		//joao

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
		else if (type_calibration == CONCENTRIC_CIRCLES)
		{
			size_unit = frame_input.cols / 15.0f;
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
		else if ( type_calibration == ASYMMETRIC_CIRCLES_GRID )
		{

			size_unit = frame_input.cols / 16.0f;
			offset = size_unit / 3.8f;

			src_quad[0] = control_points_input[3];
			src_quad[1] = control_points_input[43];
			src_quad[2] = control_points_input[40];
			src_quad[3] = control_points_input[0];

			width = 11;
			height = 7;

			dest_quad[0] = Point2f( size_unit + offset, size_unit + 3 * offset);
			dest_quad[1] = Point2f( width * size_unit + offset, size_unit + 3 * offset);

			dest_quad[2] = Point2f( width * size_unit + offset, height * size_unit + 3 * offset);
			dest_quad[3] = Point2f( size_unit + offset, height * size_unit + 3 * offset);


			float sizeH = dest_quad[3].y - dest_quad[1].y;
			float sizeW = dest_quad[1].x - dest_quad[0].x;

			size_out.width = (10.0f / 8.0f) * sizeW;
			size_out.height = (10.0f / 7.0f) * sizeH ;
		}


		lambda = getPerspectiveTransform( src_quad, dest_quad );






		warpPerspective(frame_input, frame_output, lambda, size_out );

		//ProcManager pc;
		//pc.drawControlPointsCross(frame_input, src_quad );

		//imshow("input", frame_input);

		//imshow("fp", frame_output);
		//waitKey();



		vector<Point2f> cp_fp;
		find_control_points(frame_output, cp_fp);


		//ProcManager pcx;
		//pcx.drawControlPointsCross(frame_input,src_quad);
		//pcx.drawControlPointsCross(frame_output,dest_quad);
		//pcx.drawControlPointsCross(frame_output,cp_fp);
		//imshow("front parallel", frame_output);
		//imshow("imagen", frame_input);
		//waitKey();




		return true;
	}


	bool intercept_line(Point2f A, Point2f B, Point2f C, Point2f D, Point2f& interception)
	{
		// Linea AB --> a1x + b1y = c1
		double a1 = B.y - A.y;
		double b1 = A.x - B.x;
		double c1 = a1 * (A.x) + b1 * (A.y);

		// Linea CD --> a2x + b2y = c2
		double a2 = D.y - C.y;
		double b2 = C.x - D.x;
		double c2 = a2 * (C.x) + b2 * (C.y);

		double determinant = a1 * b2 - a2 * b1;

		if (determinant == 0)
		{
			//es paralela
			return false;
		}
		else
		{
			double x = (b2 * c1 - b1 * c2) / determinant;
			double y = (a1 * c2 - a2 * c1) / determinant;
			interception = Point2f(x, y);
			return true;
		}
	}

	void aproximate_line(vector<Point2f>& points, vector<Point2f>& line_out)
	{
		line_out.clear();
		Vec4f line;
		fitLine(points, line, CV_DIST_L2, 0, 0.01, 0.01);

		Point2f p1 = Point2f(line[2], line[3]) - 1000 * (Point2f(line[0], line[1]));
		Point2f p2 = Point2f(line[2], line[3]) + 1000 * (Point2f(line[0], line[1]));

		line_out.push_back( p1 );
		line_out.push_back( p2 );
	}

	void refine_control_points_colinearity(vector<Point2f>& control_points_front_parallel, vector<Point2f>& control_points_refined_colinearity)
	{
		vector< vector<Point2f> > points_h(pattern_size.height), points_v(pattern_size.width);
		control_points_refined_colinearity.clear();

		for (int i = 0; i < pattern_size.height; i++)
		{

			for (int j = 0; j < pattern_size.width; j++)
			{
				points_h[i].push_back(control_points_front_parallel[ j + pattern_size.width * i ]);
			}
			/*
			points_h[i].push_back(control_points_front_parallel[ 0 + pattern_size.width * i ]);
			points_h[i].push_back(control_points_front_parallel[ 1 + pattern_size.width * i ]);
			points_h[i].push_back(control_points_front_parallel[ 2 + pattern_size.width * i ]);
			points_h[i].push_back(control_points_front_parallel[ 3 + pattern_size.width * i ]);
			points_h[i].push_back(control_points_front_parallel[ 4 + pattern_size.width * i ]);
			*/
		}

		for (int i = 0; i < pattern_size.width; i++)
		{
			for (int j = 0; j < pattern_size.height; j++)
			{
				points_v[i].push_back(control_points_front_parallel[ j * pattern_size.width + i ]);
			}
			/*
			points_v[i].push_back(control_points_front_parallel[ 0 + i ]);
			points_v[i].push_back(control_points_front_parallel[ 5 + i ]);
			points_v[i].push_back(control_points_front_parallel[ 10 + i ]);
			points_v[i].push_back(control_points_front_parallel[ 15 + i ]);
			*/
		}


		for (int i = 0; i < pattern_size.height; i++)
		{
			for (int j = 0; j < pattern_size.width; j++)
			{
				vector<Point2f> lineH, lineV;
				aproximate_line(points_h[i], lineH);
				aproximate_line(points_v[j], lineV);
				//cv::line( front_parallel, lineH[0], lineH[1] , Scalar(0, 255, 0), 1, 8 );
				//cv::line( front_parallel, lineV[0], lineV[1] , Scalar(0, 255, 0), 1, 8 );
				Point2f interception;
				intercept_line(lineH[0], lineH[1], lineV[0], lineV[1], interception);
				control_points_refined_colinearity.push_back(interception);
			}
		}

		/*
		for(int i=0; i<20; i++)
		{
			cout<<"point detect     : "<<control_points_front_parallel[i]<<endl;
			cout<<"point colinearity: "<<points_refine_colinearity[i]<<endl;
		}

		ProcManager pc;
		pc.drawControlPointsCross(front_parallel, control_points_front_parallel, Scalar(255,255,0) );
		pc.drawControlPointsCross(front_parallel, points_refine_colinearity );
		*/
	}

	//recibe una imagen rectificada, devuelve los puntos corregidos en fronto paralelo en el espacio de la imagen sin rectificar
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


		//imshow("front_parallel", front_parallel);
		//waitKey();

		//llevar los puntos de control del fronto paralelo hacia la imagen rectificada
		perspectiveTransform( control_points_front_parallel, control_points_refine, lambda.inv() );

		//distorsionar los puntos

		//joao quitar para ps3
		distControlPoints(control_points_refine, cameraMatrix, distCoeffs);




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
	void init_calibrate(const string& pathParameters)
	{
		Mat cameraMatrix, distCoeffs;
		vector<Mat> rvecs, tvecs;
		double rms;
		vector<Mat> frames;
		readParameters(pathParameters + "initial_calibration.yml", cameraMatrix, distCoeffs, rvecs, tvecs, rms);

		//leemos las  imagenes de la calibracion inicial
		for (int i = 0; i < tvecs.size(); i++)
		{
			Mat image = imread(pathParameters + "frame_" + to_string(i) + ".png");
			//cout<<pathParameters + "frame_" + to_string(i) + ".jpg"<<endl;
			if (!image.data )
			{

				cout << "Image no read" << endl;
				return;
			}
			frames.push_back(image);
		}


		cout << "rms pre refine :" << rms << endl;

		//float colinearity = computeColinearError(frames, pattern_size);

		//cout << "colinearity pre refine : " << colinearity << endl;

		calibrate( cameraMatrix, distCoeffs, rvecs, tvecs, frames );

	}


	double colineal_error_line(vector<Point2f> _points , vector<float> & _line) {

		vector<float> line(4);
		fitLine(_points, line, CV_DIST_L2, 0, 0.01, 0.01);

		//cout<<"line params "<<line[0]<<" "<<line[1]<<" "<<line[2]<<" "<<line[3]<<endl;
		Point2f a(line[0], line[1]);
		Point2f pa(line[2], line[3]);

		Point2f b;
		double error = 0.0;
		for (Point2f _p : _points) {
			b = _p - pa;
			error += abs(a.cross(b)) / (cv::norm(a));
		}

		error /= _points.size();

		_line = line;
		return error;
	}

	//Compute colinearError using an estimate line NICOLAS
	double computeColinearError(const vector<vector<Point2f> >& imagePoints, Size _boardSize) {
		double col_err = 0.0;
		double col_frame_err = 0.0;
		for (vector<Point2f> centers : imagePoints ) {
			for (int i = 0; i < _boardSize.height; i++) {
				// 0 5
				vector<Point2f> line ( centers.begin() + i * _boardSize.width,  centers.begin() + (i + 1)*_boardSize.width);
				vector<float> _line(4);
				col_frame_err += colineal_error_line(line, _line);

			}
			col_frame_err /= _boardSize.height;
			col_err += col_frame_err;
			col_frame_err = 0.0;
		}


		col_err /= imagePoints.size();
		return col_err;
	}


	void calibrate(	Mat& cameraMatrix , Mat& distCoeffs, vector<Mat>& rvecs, vector<Mat>& tvecs, vector<Mat>& frames)
	{

		vector<Mat> frames_undistorted;
		for ( int iteration = 0; iteration < 3; iteration++ )
		{

			//corregimos las imagenes con los parametros de calibracion inical
			undistort_frames(frames, frames_undistorted, cameraMatrix, distCoeffs);


			vector< vector<Point2f> > image_points;
			vector<Mat> rechoiced_frames;

			float colinearity = 0.0f;
			int countCol = 0;
			//buscar los puntos de control refinados
			for (int i = 0; i < frames.size(); i++ )
			{

				vector< Point2f > control_points_refine;
				bool found = get_control_points_refine( frames_undistorted[i], control_points_refine, cameraMatrix, distCoeffs );


				if ( found )
				{
					rechoiced_frames.push_back(frames[i]);
					image_points.push_back( control_points_refine );

					//metric
					//colinearity += computeColinearError(control_points_refine, pattern_size);
					countCol++;



					ProcManager pcx;
					Mat copy = frames[i].clone();
					pcx.drawControlPointsCross(copy, control_points_refine);
					imshow("refined", copy);
					waitKey(200);


				}
			}



			//colinearity /= ((float) countCol);

			//actualizar la lista de frames, solo por sea el caso algun frame se descarte por perdida de deteccion
			frames = rechoiced_frames;

			//calibramos con opencv
			vector<float> reprojErrs;
			double rms;
			bool is_calibrate = opencv_calibration(frames[0].size(), cameraMatrix, distCoeffs, image_points, rvecs, tvecs, reprojErrs, rms );


			colinearity = computeColinearError(image_points, pattern_size);

			//medimos los resultados
			cout << "frames used: " << countCol << endl;
			cout << "rms iteration " << iteration << ": " << rms << endl;
			//float colinearity = AVGcolinearityFrames(frames_undistorted);
			cout << "colinearity iteration " << iteration << ": " << colinearity << endl;
		}

		//vizualizar la calibracion
		undistort_frames(frames, frames_undistorted, cameraMatrix, distCoeffs);
		for ( int i = 0; i < frames.size(); i++ )
		{
			imshow("original", frames[ i ]);
			imshow("calibrated", frames_undistorted[ i ]);
			waitKey();
		}

	}
};
}