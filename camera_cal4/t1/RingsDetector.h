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
class RingsDetector
{
public:
	Rect roi;
	
	RingsDetector()
	{

	}

	const float distance(const Point2f &a, const Point2f &b)
	{
		return sqrt( pow(a.x - b.x, 2) + pow (a.y - b.y, 2) );
	}

	const float distance(const Ellipse &a, const Ellipse &b)
	{
		return sqrt( pow(a.element.center.x - b.element.center.x, 2) + pow (a.element.center.y - b.element.center.y, 2) );
	}


	int numberChilds(vector<Vec4i>& hierarchy, int index)
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

	bool findPattern( Mat& image, vector<Point2f> &pointBuf )
	{
		pointBuf.clear();

		vector<Ellipse> lsDetection;
		
		vector<Ellipse> lsEllipses, lsEllipsesAux;

		RNG rng(12345);

		int max_thresh = 255;
		Mat _procImage;
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy, hierarchyFilters;
		cvtColor(image, _procImage, CV_BGR2GRAY, 1 );
		//_procImage = _procImage(roi);
		GaussianBlur( _procImage, _procImage, Size( 15, 15), 0, 0 );

		adaptiveThreshold(_procImage , _procImage, max_thresh, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 5, 2);
		//imshow("_procImage", _procImage);

		findContours( _procImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );


		Mat drawing = Mat::zeros( _procImage.size(), CV_8UC3 );

		Scalar color = Scalar( 0, 0, 255 );

		vector<int> filters;
		vector<int> nNeighsCenterNear;
		vector<RotatedRect> original(contours.size());
		for ( int i = 0; i < contours.size(); i++ )
		{

			if ( contours[i].size() > 5)
			{

				filters.push_back(i);
				nNeighsCenterNear.push_back(0);
				original[i] = minAreaRect( Mat(contours[i]) );
			}

		}


		for (int i = 0; i < filters.size(); i++)
		{
			RotatedRect rEle = minAreaRect( Mat(contours[filters[i]]) );

			Ellipse element(filters[i], rEle);
			lsEllipses.push_back(element);
		}

		if (lsEllipses.empty()) return false;



		for (int i = 0; i < lsEllipses.size(); i++)
		{
			for (int j = 0; j < lsEllipses.size(); j++)
			{
				if ( i != j )
				{
					if (distance( lsEllipses[i], lsEllipses[j] ) < 2.0f )
					{


						/*
						float wDiff = abs(	max(lsEllipses[i].element.size.width, lsEllipses[i].element.size.height) -
						                    max(lsEllipses[j].element.size.width, lsEllipses[j].element.size.height) );

						float hDiff = abs(	min(lsEllipses[i].element.size.width, lsEllipses[i].element.size.height) -
						                    min(lsEllipses[j].element.size.width, lsEllipses[j].element.size.height) );


						if ( wDiff > 4 && wDiff < 30 && hDiff > 2 && hDiff < 50)
						{
							nNeighsCenterNear[i] += 1;
						}*/
						nNeighsCenterNear[i] += 1;

					}
				}
			}
			if (nNeighsCenterNear[i] > 0)
			{
				lsEllipsesAux.push_back(lsEllipses[i]);
				nNeighsCenterNear[i] = 0;
			}
		}


		//filtrar por la medianas del size
		vector<Ellipse> lsMedianaChild;

		for (int i = 0; i < lsEllipsesAux.size(); i++)
		{
			int child = hierarchy[ lsEllipsesAux[i].index ][2];
			int nChilds = numberChilds(hierarchy, lsEllipsesAux[i].index);
			if ( nChilds == 1 )
			{
				lsMedianaChild.push_back(lsEllipsesAux[i]);

			}
		}

		sort(lsMedianaChild.begin(), lsMedianaChild.end(), lessCompareW);
		float mediaW = lsMedianaChild[lsMedianaChild.size() / 2].element.size.width;
		sort(lsMedianaChild.begin(), lsMedianaChild.end(), lessCompareH);
		float mediaH = lsMedianaChild[lsMedianaChild.size() / 2].element.size.height;

		lsMedianaChild.clear();
		for (int i = 0; i < lsEllipsesAux.size(); i++)
		{
			int child = hierarchy[ lsEllipsesAux[i].index ][2];
			int nChilds = numberChilds(hierarchy, lsEllipsesAux[i].index);
			if ( nChilds == 1 )
			{
				float diffSize = abs(lsEllipsesAux[i].element.size.width + lsEllipsesAux[i].element.size.height - mediaW - mediaH);
				if (diffSize < 10)
				{
					lsMedianaChild.push_back(lsEllipsesAux[i]);
				}
			}
		}

		lsEllipses = lsMedianaChild;

		if (lsEllipses.empty()) return false;


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


		int r = 0;
		for (r = 1; r < 300; r++)
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

			if (lsEllipsesAux.size() >= 20)
			{
				//cout << "r" << r << endl;
				break;
			}

		}

		mediana.size.width = 2 * r;
		mediana.size.height = 2 * r;

		lsEllipses = lsEllipsesAux;

		for (int i = 0; i < lsEllipses.size(); i++)
		{

			int child = hierarchy[ lsEllipses[i].index ][2];

			int nChilds = numberChilds(hierarchy, lsEllipses[i].index);

			//roi Correction
			lsEllipses[i].element.center.x += roi.x;
			lsEllipses[i].element.center.y += roi.y;
			original[child].center.x += roi.x;
			original[child].center.y += roi.y;

			if ( nChilds == 1 )
			{

				lsEllipses[i].element.center.x = ( lsEllipses[i].element.center.x + original[child].center.x ) / 2.0f;
				lsEllipses[i].element.center.y = ( lsEllipses[i].element.center.y + original[child].center.y ) / 2.0f;
				lsDetection.push_back(lsEllipses[i]);
			}

		}

		for(int i = 0; i<lsDetection.size(); i++)
		{
			ellipse( image, lsDetection[i].element , Scalar(0, 0, 0) , -1, 50 );
		}
		
		Size mPatternSize(5, 4);
		bool found = findCirclesGrid( image, mPatternSize, pointBuf );

		return found;

		//if(found)
		//	drawChessboardCorners( image, mPatternSize, Mat(pointBuf), found );

	}
};
}