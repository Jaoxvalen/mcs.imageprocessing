#pragma once


#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


using namespace std;
using namespace cv;

class Ellipse
{
public:

	RotatedRect element;
	int index;
	float distanceTo;

	Ellipse(int index, RotatedRect element)
	{
		this->element = element;
		this->index = index;
	}
	
	const float distance(const Ellipse &other)
	{
		return sqrt( pow(element.center.x - other.element.center.x, 2) + pow (element.center.y - other.element.center.y, 2) );
	}

	const float distance(const float x, const float y)
	{
		return sqrt( pow(element.center.x - x, 2) + pow (element.center.y - y, 2) );
	}

	const float distance(Point2f &ele)
	{
		return sqrt( pow(element.center.x - ele.x, 2) + pow (element.center.y - ele.y, 2) );
	}

	const float distanceX(const float x)
	{
		return abs(element.center.x - x);
	}

	const float distanceY(const float y)
	{
		return abs(element.center.y - y);
	}
	const bool equalCenter(const Ellipse &other)
	{
		return element.center.x == other.element.center.x && element.center.y == other.element.center.y;
	}
	const bool equalCenter(const RotatedRect &other)
	{
		return element.center.x == other.center.x && element.center.y == other.center.y;
	}
	const float getMagnitude()
	{
		return sqrt(element.center.x * element.center.x + element.center.y * element.center.y);
	}
	const Point2f getNormalized()
	{
		return (element.center/getMagnitude());
	}
};