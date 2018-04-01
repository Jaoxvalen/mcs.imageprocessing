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


struct grid_cell
{
	vector<int> indices;
};

class grid
{
private:
	Size s_image;
	Size s_grid;
	int n_frames;
	//vector< vector<int> > mfill;

	vector< vector< grid_cell > > mfill;

	vector<int> selected_indices;



	Mat canvas, percent;
public:

	grid(Size s_image, Size s_grid, int n_frames)
	{

		selected_indices.resize(0);
		correct_size_grid(s_grid, n_frames);

		this->s_image = s_image;
		this->s_grid = s_grid;
		this->n_frames = n_frames;



		//init mfill
		//mfill.resize(s_grid.width);
		mfill.resize(s_grid.width);
		for (int i = 0; i < mfill.size() ; i++)
		{
			mfill[i].resize(s_grid.height);

			for ( int j = 0 ; j < mfill[i].size(); j++ )
			{
				mfill[i][j] = grid_cell();
				mfill[i][j].indices.resize(0);
			}

			//mfill[i].resize(s_grid.height);

			/*
			for ( int j = 0 ; j < mfill[i].size(); j++ )
			{
				mfill[i][j] = 0;
			}*/
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


	void correct_size_grid(Size& s_grid, int n_frames)
	{

		int totalgrid = s_grid.width * s_grid.height;
		int n_framepercell = n_frames / (float)totalgrid;


		while ( n_framepercell == 0  )
		{
			s_grid.width--;
			s_grid.height--;
			cout << "El grid tiene mas celdas que frames, se redimensionara" << endl;
			cout << "size " << s_grid.width << "x" << s_grid.height << endl;

			totalgrid = s_grid.width * s_grid.height;
			n_framepercell = n_frames / (float)totalgrid;

		}
	}

	void take_frames()
	{
		int totalgrid = s_grid.width * s_grid.height;
		int n_framepercell = n_frames / (float)totalgrid;

		int take_frames = 0;

		while ( n_frames >  take_frames)
		{
			int takes = 0;
			for ( int i = 0; i < s_grid.width; i++ )
			{
				for ( int j = 0; j < s_grid.height; j++ )
				{
					cout << "Celda " << i << "," << j << endl;
					takes = getRandomFrames( Point2i(i, j) , 1 );
				}
			}
			if(takes>0)
			{
				take_frames += takes;
			}

		}



	}

	int getRandomFrames(Point2i cell, int num_frames)
	{
		vector<int> indices = mfill[cell.x][cell.y].indices;
		//
		std::random_shuffle ( indices.begin(), indices.end() );

		//cout<<"size celda: "<<indices.size()<<endl;
		int takes = 0;

		for (int i = 0; i < indices.size(); i++)
		{
			int index_frame = indices[i];
			bool exist = false;

			for (int j = 0; j < selected_indices.size(); j++)
			{
				if ( index_frame == selected_indices[j])
				{
					exist = true;
					break;
				}
			}

			if (!exist)
			{
				selected_indices.push_back(index_frame);
				cout << "frame tomado: " << index_frame << endl;
				takes++;
			}
			if (takes == num_frames) break;
		}

		return takes;

	}





	//void fillPoint(const Point2f& point)
	void fillPoint(const Point2f& point, int index )
	{

		circle( canvas, point , 1 , Scalar(255, 255, 255) , 1, 1 );

		Point2i coord = getCordsArea(point);

		mfill[coord.x][coord.y].indices.push_back(index);

		/*
		if ( canvas.at<Vec3b>(point.y, point.x) != Vec3b(255, 255, 255))
		{
			mfill[coord.x][coord.y]++;
		}*/

	}
	void fillPoints(const vector<Point2f>& points, int index)
	{

		fillPoint(points[6], index);
		fillPoint(points[7], index);
		fillPoint(points[8], index);
		fillPoint(points[11], index);
		fillPoint(points[12], index);
		fillPoint(points[13], index);

		/*
		for(int i = 0; i<points.size(); i++)
		{
			fillPoint(points[i], index);
		}*/


		/*
		for(int i = 0; i<points.size(); i++)
		{
			fillPoint(points[i]);
		}*/



		/*
		// define a polygon (as a vector of points)
		vector<Point> contour;
		contour.push_back((Point)points[0]);
		contour.push_back((Point) points[4]);
		contour.push_back((Point) points[19]);
		contour.push_back((Point) points[15]);
		// create a pointer to the data as an array of points (via a conversion to
		// a Mat() object)
		const cv::Point *pts = (const cv::Point*) Mat(contour).data;
		int npts = Mat(contour).rows;
		// draw the polygon
		fillPoly(canvas, &pts, &npts, 1,
		          Scalar(255, 255, 255), // colour RGB ordering (here = green)
		          1);
		*/


	}

	void show()
	{
		imshow("grid", canvas);

	}
};
}