#pragma once

#include <chrono>

#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "Ellipse.h"


using namespace std;
using namespace cv;
using namespace chrono;

#define INF 9999;

namespace vision
{

/*
bool lessCompareX(const RotatedRect &a , const RotatedRect &b)
{
	return a.center.x > b.center.x;
}

bool lessCompareY(const RotatedRect &a , const RotatedRect &b)
{
	return a.center.y > b.center.y;
}
*/

bool lessCompareX(const Ellipse &a , const Ellipse &b)
{
	return a.element.center.x > b.element.center.x;
}

bool lessCompareY(const Ellipse &a , const Ellipse &b)
{
	return a.element.center.y > b.element.center.y;
}

class ProcManager
{
public:
	float time;
	vector<Ellipse> lsDetection;
	bool isTracking;

	static ProcManager* INSTANCE;

	ProcManager() {
		isTracking = false;
	}
	static void create()
	{
		if ( ProcManager::INSTANCE != NULL )
		{
			delete ProcManager::INSTANCE;
		}
		ProcManager::INSTANCE = new ProcManager();
	}

	bool numberChilds(vector<Vec4i>& hierarchy, int index)
	{
		int child = hierarchy[index][2];
		int i = 0;
		while (child != -1)
		{
			child = hierarchy[child][2];
			i++;
			if (i > 2) break;
		}

		return i;
	}

	const float distance(const Point2f &a, const Point2f &b)
	{
		return sqrt( pow(a.x - b.x, 2) + pow (a.y - b.y, 2) );
	}

	const float distance(const Ellipse &a, const Ellipse &b)
	{
		return sqrt( pow(a.element.center.x - b.element.center.x, 2) + pow (a.element.center.y - b.element.center.y, 2) );
	}

	const int mostNearX(vector< Ellipse > &ls, const float xValue)
	{
		float minDis = ls[0].distanceX(xValue);

		int index = 0;
		for (int i = 1; i < ls.size(); i++)
		{
			float distX = ls[i].distanceX(xValue);
			if ( distX <  minDis )
			{
				minDis = distX;
				index = i;
			}
		}
		return index;
	}

	const int mostNearY(vector< Ellipse > &ls, const float yValue)
	{
		float minDis = ls[0].distanceY(yValue);

		int index = 0;
		for (int i = 1; i < ls.size(); i++)
		{
			float distY = ls[i].distanceY(yValue);
			if ( distY <  minDis )
			{
				minDis = distY;
				index = i;
			}
		}
		return index;
	}
	const int mostFar(vector<Ellipse> &ls, Ellipse eEllipse)
	{
		float minDis = 0;
		int index = -1;

		for (int i = 0; i < ls.size() ; i++)
		{
			if ( !eEllipse.equalCenter(ls[i]) )
			{
				float dist = eEllipse.distance(ls[i]);
				if ( dist > minDis )
				{
					minDis = dist;
					index = i;
				}
			}
		}
		return index;
	}

	void drawEllipse(Mat & image, RotatedRect& ellipse)
	{
		Point2f rect_points[4]; ellipse.points( rect_points );
		for ( int k = 0; k < 4; k++ )
		{
			line( image, rect_points[k], rect_points[(k + 1) % 4], Scalar(0, 255, 255), 5, 8 );
		}
	}

	void tracking(Mat& image)
	{
		if (!isTracking || true)
		{
			isTracking = true;

			//buscar el elemento mas cercano al 0 en x
			int p0 = mostNearX(lsDetection, 0.0f);
			int p1 = mostFar(lsDetection, lsDetection[p0]);
			int p2 = mostNearY(lsDetection, 0.0f);
			int p3 = mostFar(lsDetection, lsDetection[p2]);

			drawEllipse(image,lsDetection[p0].element);
			drawEllipse(image,lsDetection[p1].element);
			drawEllipse(image,lsDetection[p2].element);
			drawEllipse(image,lsDetection[p3].element);

		}
	}

	void preProcessing(Mat& image )
	{

		lsDetection.clear();

		vector<Ellipse> lsEllipses, lsEllipsesAux;

		high_resolution_clock::time_point t1 = high_resolution_clock::now();

		Size size(640, 480); //the dst image size,e.g.100x100
		resize(image, image, size);

		RNG rng(12345);
		int max_thresh = 255;
		Mat _procImage;

		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy, hierarchyFilters;

		cvtColor(image, _procImage, CV_BGR2GRAY, 1 );
		GaussianBlur( _procImage, _procImage, Size( 15, 15), 0, 0 );
		adaptiveThreshold(_procImage , _procImage, max_thresh, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 15, 2);

		imshow("adaptiveThreshold", _procImage);
		//Canny( _procImage, _procImage, 100, 200, 3 );
		findContours( _procImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );


		Mat drawing = Mat::zeros( _procImage.size(), CV_8UC3 );

		Scalar color = Scalar( 0, 0, 255 );

		vector<int> filters;
		vector<int> nNeighsCenterNear;
		vector<RotatedRect> original(contours.size());
		for ( int i = 0; i < contours.size(); i++ )
		{

			//if ( contours[i].size() > 5 && numberChilds(hierarchy , i) == 1 )
			if ( contours[i].size() > 5)
			{

				filters.push_back(i);
				nNeighsCenterNear.push_back(0);
				original[i] = minAreaRect( Mat(contours[i]) );
				//drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
			}

		}


		for (int i = 0; i < filters.size(); i++)
		{
			RotatedRect rEle = minAreaRect( Mat(contours[filters[i]]) );
			//minRect.push_back( rEle );

			Ellipse element(filters[i], rEle);
			lsEllipses.push_back(element);
		}


		if (lsEllipses.empty()) return;

		for (int i = 0; i < lsEllipses.size(); i++)
		{
			for (int j = 0; j < lsEllipses.size(); j++)
			{
				if ( i != j )
				{
					if (distance( lsEllipses[i], lsEllipses[j] ) < 2.0f )
					{

						/*
						float wDiff = abs(	max(minRect[i].size.width, minRect[i].size.height) -
						                    max(minRect[j].size.width, minRect[j].size.height) );
						*/
						float wDiff = abs(	max(lsEllipses[i].element.size.width, lsEllipses[i].element.size.height) -
						                    max(lsEllipses[j].element.size.width, lsEllipses[j].element.size.height) );


						float hDiff = abs(	min(lsEllipses[i].element.size.width, lsEllipses[i].element.size.height) -
						                    min(lsEllipses[j].element.size.width, lsEllipses[j].element.size.height) );

						//cout << wDiff << " " << hDiff << endl;

						if ( wDiff > 4 && wDiff < 25
						        && hDiff > 4 && hDiff < 25)
						{


							nNeighsCenterNear[i] += 1;
						}
					}
				}
			}
			if (nNeighsCenterNear[i] > 0)
			{
				lsEllipsesAux.push_back(lsEllipses[i]);
				//aux.push_back(minRect[i]);
				nNeighsCenterNear[i] = 0;
			}
		}


		lsEllipses = lsEllipsesAux;
		//minRect = aux;

		if (lsEllipses.empty()) return;

		/*
		sort(minRect.begin(), minRect.end(), lessCompareX);
		float medianaX = minRect[minRect.size() / 2].center.x;
		float medianaSizeW = 10;//minRect[minRect.size()/2].size.width;


		sort(minRect.begin(), minRect.end(), lessCompareY);
		float medianaY = minRect[minRect.size() / 2].center.y;
		float medianaSizeH = 10;//minRect[minRect.size()/2].size.height;*/

		sort(lsEllipses.begin(), lsEllipses.end(), lessCompareX);
		float medianaX = lsEllipses[lsEllipses.size() / 2].element.center.x;
		float medianaSizeW = 10;


		sort(lsEllipses.begin(), lsEllipses.end(), lessCompareY);
		float medianaY = lsEllipses[lsEllipses.size() / 2].element.center.y;
		float medianaSizeH = 10;


		RotatedRect mediana;
		mediana.center.x = medianaX;
		mediana.center.y = medianaY;
		mediana.size.width = medianaSizeW;
		mediana.size.height = medianaSizeH;


		//cout << "minrec :" << minRect.size() << endl;

		int r = 0;
		for (r = 1; r < 200; r++)
		{
			lsEllipsesAux.clear();
			for ( int i = 0; i < lsEllipses.size(); i++ )
			{
				//cout<< aux.size()<<endl;
				if ( distance(lsEllipses[i].element.center , mediana.center ) < r )
				{
					lsEllipsesAux.push_back(lsEllipses[i]);
				}
			}

			if (lsEllipsesAux.size() >= 40)
			{
				//cout << "r" << r << endl;
				break;
			}

		}

		mediana.size.width = 2 * r;
		mediana.size.height = 2 * r;

		lsEllipses = lsEllipsesAux;

		//ellipse( image, mediana , Scalar(0, 255, 0) , 1, 8 );


		for (int i = 0; i < lsEllipses.size(); i++)
		{

			int child = hierarchy[ lsEllipses[i].index ][2];

			if ( child != -1 )
			{

				float x = (original[child].center.x + lsEllipses[i].element.center.x) / 2;
				float y = (original[child].center.y + lsEllipses[i].element.center.y) / 2;

				RotatedRect ellipseCenter;

				ellipseCenter.center.x = x;
				ellipseCenter.center.y = y;

				ellipseCenter.size.width = 5;
				ellipseCenter.size.height = 5;

				Ellipse element = Ellipse(0, ellipseCenter);
				lsDetection.push_back(element);

				//ellipses.push_back(ellipseCenter);
			}

			/*
			Point2f rect_points[4]; lsEllipses[i].element.points( rect_points );
			for ( int k = 0; k < 4; k++ )
			{
				line( image, rect_points[k], rect_points[(k + 1) % 4], color, 1, 8 );
			}*/
		}


		for (int i = 0; i < lsDetection.size(); i++)
		{

			Point2f rect_points[4]; lsDetection[i].element.points( rect_points );
			for ( int k = 0; k < 4; k++ )
			{
				line( image, rect_points[k], rect_points[(k + 1) % 4], Scalar(255, 255, 0), 5, 8 );
			}
		}

		//imshow("_procImage", drawing);




		high_resolution_clock::time_point t2 = high_resolution_clock::now();
		duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

		tracking(image);

		time = time_span.count();


	}

};
ProcManager* ProcManager::INSTANCE = NULL;
}