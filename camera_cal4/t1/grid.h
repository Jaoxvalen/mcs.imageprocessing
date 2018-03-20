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

#include "Utils.h"


using namespace cv;
using namespace std;

namespace vision
{
class grid
{
private:
	Size s_image;
	Size s_grid;
	vector< vector<int> > mfill;
	Mat canvas, percent;
public:

	grid(Size s_image, Size s_grid)
	{
		this->s_image = s_image;
		this->s_grid = s_grid;

		//init mfill
		mfill.resize(s_grid.width);
		for (int i = 0; i < mfill.size() ; i++)
		{
			mfill[i].resize(s_grid.height);
			for ( int j = 0 ; j < mfill[i].size(); j++ )
			{
				mfill[i][j] = 0;
			}
		}

		//create canvas
		canvas = Mat(s_image.height, s_image.width, CV_8UC3, Scalar(0, 0, 0));

		//draw lines grid
		float unit_w = s_image.width / s_grid.width;
		float unit_h = s_image.height / s_grid.height;
		for (int i = 1; i <= s_grid.width; i++)
		{
			line( canvas, Point2f( i * unit_w, 0 ), Point2f( i * unit_w, s_image.height ), Scalar( 0, 0, 255 ), 1, 4 );
		}
		for (int i = 1; i <= s_grid.height; i++)
		{
			line( canvas, Point2f( 0, i * unit_h ), Point2f( s_image.width, i * unit_h ), Scalar( 0, 0, 255 ), 1, 4 );
		}
	}
	const Point2i getCordsArea(const Point2f& point)
	{
		float unit_w = float(s_image.width) / float(s_grid.width);
		float unit_h = float(s_image.height) / float(s_grid.height);

		Point2i coord;

		coord.x = int(point.x / unit_w);
		coord.y = int(point.y / unit_h);

		return coord;
	}
	void fillPoint(const Point2f& point)
	{
		circle( canvas, point , 1 , Scalar(255, 255, 255) , 1, 1 );
		Point2i coord = getCordsArea(point);

		if( canvas.at<Vec3b>(point.y, point.x) != Vec3b(255, 255, 255))
		{
			mfill[coord.x][coord.y]++;	
		}
		
	}
	void fillPoints(const vector<Point2f>& points)
	{
		for(int i = 0; i<points.size(); i++)
		{
			fillPoint(points[i]);
		}
	}

	void show()
	{
		imshow("grid", canvas);

	}
};
}