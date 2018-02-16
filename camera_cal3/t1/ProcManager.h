#pragma once

#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


using namespace std;
using namespace cv;

namespace vision
{
class ProcManager
{
public:
	static ProcManager* INSTANCE;

	ProcManager() {}
	static void create()
	{
		if ( ProcManager::INSTANCE != NULL )
		{
			delete ProcManager::INSTANCE;
		}
		ProcManager::INSTANCE = new ProcManager();
	}

	bool haveMoreTwoChilds(vector<Vec4i>& hierarchy, int index)
	{
		int child = hierarchy[index][2];
		int i = 0;
		while (child != -1)
		{
			child = hierarchy[child][2];
			i++;
			if (i > 2) break;
		}
		if (i > 2 || i == 0) return true;
		return false;
	}

	const float distance(const Point2f &a, const Point2f &b)
	{
		return sqrt( pow(a.x - b.x, 2) + pow (a.y - b.y, 2) );
	}


	void preProcessing(Mat& image)
	{
		RNG rng(12345);
		int max_thresh = 255;
		Mat _procImage;

		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		cvtColor(image, _procImage, CV_BGR2GRAY, 1 );
		GaussianBlur( _procImage, _procImage, Size( 15, 15), 0, 0 );
		adaptiveThreshold(_procImage , _procImage, max_thresh, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 15, 2);
		Canny( _procImage, _procImage, 100, 200, 3 );

		findContours( _procImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );



		Mat drawing = Mat::zeros( _procImage.size(), CV_8UC3 );

		Scalar color = Scalar( 0, 0, 255 );

		vector<int> filters;
		vector<int> nNeighsCenterNear;
		vector<RotatedRect> minRect, aux;
		for ( int i = 0; i < contours.size(); i++ )
		{
			int parent = hierarchy[i][3];
			if ( contours[i].size() > 5 && !haveMoreTwoChilds(hierarchy , i) )
			{
				filters.push_back(i);
				nNeighsCenterNear.push_back(0);
				//drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
			}

		}


		for (int i = 0; i < filters.size(); i++)
		{
			minRect.push_back( minAreaRect( Mat(contours[filters[i]]) ) );
		}

		filters.clear();
		for (int i = 0; i < minRect.size(); i++)
		{
			for (int j = 0; j < minRect.size(); j++)
			{
				if ( i != j )
				{
					if (distance( minRect[i].center, minRect[j].center ) < 2.0f )
					{
						float wDiff = abs(	max(minRect[i].size.width, minRect[i].size.height) - 
											max(minRect[j].size.width, minRect[j].size.height) );


						float hDiff = abs(	min(minRect[i].size.width, minRect[i].size.height) -
											min(minRect[j].size.width, minRect[j].size.height) );
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
				aux.push_back(minRect[i]);
				nNeighsCenterNear[i] = 0;
				
				Point2f rect_points[4]; minRect[i].points( rect_points );
				for ( int k = 0; k < 4; k++ )
				{
					line( drawing, rect_points[k], rect_points[(k + 1) % 4], color, 1, 8 );
				}
			}
		}
		minRect = aux;
		nNeighsCenterNear.clear();

		
		for (int i = 0; i < minRect.size(); i++)
		{
			nNeighsCenterNear.push_back(0);
		}

		if (minRect.empty()) return;


		imshow("_procImage", drawing);

	}

};
ProcManager* ProcManager::INSTANCE = NULL;
}