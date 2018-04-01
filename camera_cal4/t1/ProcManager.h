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

bool thanCompareIndex(const Ellipse &a, const Ellipse &b)
{
	return a.index>b.index;
}

bool lessCompareW(const Ellipse &a , const Ellipse &b)
{
	return a.element.size.width < b.element.size.width;
}
bool lessCompareH(const Ellipse &a , const Ellipse &b)
{
	return a.element.size.height < b.element.size.height;
}

bool lessCompareX(const Ellipse &a , const Ellipse &b)
{
	return a.element.center.x < b.element.center.x;
}

bool lessCompareY(const Ellipse &a , const Ellipse &b)
{
	return a.element.center.y < b.element.center.y;
}

bool thanCompareDistanceTo(const Ellipse &a, const Ellipse &b)
{
	return a.distanceTo > b.distanceTo;
}

bool lessCompareDistanceTo(const Ellipse &a, const Ellipse &b)
{
	return a.distanceTo < b.distanceTo;
}

class ProcManager
{
public:
	float time;
	vector<Ellipse> lsDetection;
	vector< Ellipse > lsSort, lsSort2;
	vector< Ellipse > lsRecovery;
	bool isTracking, isRecovery;
	int p[4];
	//vector<Point2f> ROI;
	Rect roi;
	bool isFirtsPre;
	//static ProcManager* INSTANCE;
	int countFailROI;

	ProcManager() {
		isTracking = false;
		isFirtsPre = true;
		isRecovery = false;
		countFailROI = 0;
	}

	/*
	static void create()
	{
		if ( ProcManager::INSTANCE != NULL )
		{
			delete ProcManager::INSTANCE;
		}
		ProcManager::INSTANCE = new ProcManager();
	}
	*/

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

	void drawEllipse(Mat &image, RotatedRect& ellipse)
	{
		Point2f rect_points[4]; ellipse.points( rect_points );
		for ( int k = 0; k < 4; k++ )
		{
			line( image, rect_points[k], rect_points[(k + 1) % 4], Scalar(0, 255, 255), 5, 8 );
		}
	}

	RotatedRect media(vector<Ellipse> &ls )
	{
		RotatedRect element;
		element.center.x = 0;
		element.center.y = 0;
		for (int i = 0; i < ls.size() ; i++ )
		{
			element.center += ls[i].element.center;
		}
		element.center = element.center / (float)ls.size();
		element.size.width = 10;
		element.size.height = 10;

		return element;
	}

	void getDistances(vector<Ellipse> &ls, RotatedRect& e)
	{
		for (int i = 0; i < ls.size(); i++ )
		{
			ls[i].distanceTo = ls[i].distance(e.center);
		}
	}
	float getTriangleArea(Point2f &p0, Point2f &p1, Point2f &p2)
	{
		return 	abs	(
		            (p0.x * p1.y) + (p1.x * p2.y) + (p2.x * p0.y) -
		            (p0.x * p2.y) - (p2.x * p1.y) - (p1.x * p0.y)
		        ) / 2.0f;
	}

	bool inLine(Point2f a, Point2f b, Point2f x, float delta)
	{
		float distanceAB = distance(a, b);
		float distanceAX = distance(a, x);
		float distanceXB = distance(x, b);
		if ( abs((distanceAX + distanceXB) - distanceAB) < delta )
		{
			return true;
		}
		return false;
	}

	vector< Ellipse > getInLine(vector<Ellipse> &ls, Ellipse a, Ellipse b)
	{

		vector< Ellipse > lsInline;
		for (int i = 0; i < ls.size(); i++)
		{
			if 	( inLine( 	a.element.center,
			                b.element.center,
			                ls[i].element.center, 3.0f ) )
			{
				ls[i].distanceTo = ls[i].distance(a);
				ls[i].index = i;
				lsInline.push_back(ls[i]);
			}

		}
		sort(lsInline.begin(), lsInline.end(), lessCompareDistanceTo);

		return lsInline;

	}

	bool tracking(Mat& image)
	{

		//cout << "lsRecovery.size() " << lsRecovery.size() << endl;

		if ( lsDetection.size() != 20)
		{
			if (isTracking)
			{
				isRecovery = true;
			}
			return false;
		}



		if (!isTracking)
		{
			lsSort.clear();

			isTracking = true;

			RotatedRect eMedia = media(lsDetection);
			//drawEllipse(image, eMedia);
			getDistances(lsDetection, eMedia);
			sort(lsDetection.begin(), lsDetection.end(), thanCompareDistanceTo);


			//ordenar para que siempre detecte el primero
			//por ahora el mas cercano al x=0 e y=0

			float minD = INF;
			int indexD = 0;
			for(int i=0; i<20 ; i++)
			{
				float D = lsDetection[i].distance(0.0f,0.0f);
				if(D < minD)
				{
					indexD = i;
					minD = D;
				}
			}
			//cout<<"index "<<indexD<<endl;


			//corners
			RotatedRect point0, point1, point2, point3;
			//int p[4];
			//p[0] = 0;
			p[0] = indexD;


			point0 = lsDetection[p[0]].element;

			for (int i = 1; i < lsDetection.size(); i++)
			{
				if (lsDetection[p[0]].distanceTo < lsDetection[i].distance(point0.center))
				{
					p[1] = i;
					point1 = lsDetection[i].element;
					break;
				}
			}

			//buscar el triangulo de mayor area
			float maxArea = 0;
			for (int i = 0; i < lsDetection.size(); i++)
			{
				if ( !lsDetection[i].equalCenter(point0) && !lsDetection[i].equalCenter(point1) )
				{
					float area = getTriangleArea(eMedia.center, point0.center, lsDetection[i].element.center);
					if ( area > maxArea )
					{
						p[2] = i;
						point2 = lsDetection[i].element;
						maxArea = area;
					}
				}
			}

			//buscar el cuadrilatero de mayor area
			maxArea = 0;
			for (int i = 0; i < lsDetection.size(); i++)
			{
				if ( !lsDetection[i].equalCenter(point0) && !lsDetection[i].equalCenter(point1)
				        && !lsDetection[i].equalCenter(point2) )
				{
					float area =
					    getTriangleArea( point0.center, point1.center, lsDetection[i].element.center) +
					    getTriangleArea( point1.center, point2.center , lsDetection[i].element.center);

					if ( area > maxArea )
					{
						p[3] = i;
						point3 = lsDetection[i].element;
						maxArea = area;
					}
				}
			}

			//seleccionamos los corners pares frente a frente y sea la primera fila de 4
			for (int i = 1; i < 4; i++)
			{
				if ( !(inLine( lsDetection[p[0]].element.center,
				               lsDetection[p[i]].element.center, eMedia.center, 3.0f ) ) )
				{

					//check 2 inline --->
					//0
					//x
					//x
					//0
					int n = 0;
					for (int k = 0; k < lsDetection.size(); k++)
					{
						if ( !lsDetection[k].equalCenter(lsDetection[ p[0] ]) &&
						        !lsDetection[k].equalCenter(lsDetection[ p[i] ]))
						{
							if 	( inLine( 	lsDetection[ p[0] ].element.center,
							                lsDetection[ p[i] ].element.center,
							                lsDetection[k].element.center, 3.0f ) )
							{
								n++;
							}
						}

					}

					if (n == 2)
					{
						//hacemos el swap para que queden 0 y 1 con 2 y 3
						int temp = p[1];
						p[1] = p[i];
						p[i] = temp;
						break;
					}
				}
			}

			//seleccionamos los corners pares frente a frente y sea la primera columna de 5
			for (int i = 1; i < 4; i++)
			{
				if ( !(inLine( lsDetection[p[0]].element.center,
				               lsDetection[p[i]].element.center, eMedia.center, 3.0f ) ) )
				{
					//check 3 inline ---> 0 x x x 0
					
					int n = 0;
					for (int k = 0; k < lsDetection.size(); k++)
					{
						if ( !lsDetection[k].equalCenter(lsDetection[ p[0] ]) &&
						        !lsDetection[k].equalCenter(lsDetection[ p[1] ]) &&
						        !lsDetection[k].equalCenter(lsDetection[ p[i] ]))
						{
							if 	( inLine( 	lsDetection[ p[0] ].element.center,
							                lsDetection[ p[i] ].element.center,
							                lsDetection[k].element.center, 3.0f ) )
							{
								n++;
							}
						}

					}

					if (n == 3)
					{
						//hacemos el swap para que queden 0 y 1 con 2 y 3
						int temp = p[2];
						p[2] = p[i];
						p[i] = temp;
						break;
					}
				}
			}

			//drawEllipse(image, lsDetection[p[0]].element);
			//drawEllipse(image, lsDetection[p[1]].element);

			//drawEllipse(image, lsDetection[p[2]].element);
			//drawEllipse(image, lsDetection[p[3]].element);

			vector< Ellipse > lsInlineA = getInLine(lsDetection,
			                                        lsDetection[p[0]], lsDetection[p[1]]);

			vector< Ellipse > lsInlineB = getInLine(lsDetection,
			                                        lsDetection[p[2]], lsDetection[p[3]]);

			if (lsInlineA.size() != lsInlineB.size())
			{
				if (lsSort.empty())
				{
					//lsSort = lsRecovery;
					//isTracking = false;
					isRecovery = true;
				}
				return false;
			}

			int indice = 0;
			for (int i = 0; i < lsInlineA.size(); i++)
			{
				vector< Ellipse > lsTemp = getInLine(lsDetection,
				                                     lsInlineA[i], lsInlineB[i]);

				for (int j = 0; j < lsTemp.size(); j++)
				{
					lsTemp[j].index = indice;
					lsSort.push_back(lsTemp[j]);
					indice++;
				}
			}
			if (lsSort.size() == 20)
				lsRecovery = lsSort;
		}
		else // if tracking
		{

			//obtenemos los valores de p[] nuevos luego debemos hacer el matcth con el ultimo bueno
			if (isRecovery)
			{

				cout<<"in recovery"<<endl;

				lsSort.clear();
				isRecovery = false;


				RotatedRect eMedia = media(lsDetection);
				//drawEllipse(image, eMedia);
				getDistances(lsDetection, eMedia);
				sort(lsDetection.begin(), lsDetection.end(), thanCompareDistanceTo);

				//corners
				RotatedRect point0, point1, point2, point3;
				//int p[4];
				p[0] = 0;

				point0 = lsDetection[0].element;

				for (int i = 1; i < lsDetection.size(); i++)
				{
					if (lsDetection[0].distanceTo < lsDetection[i].distance(point0.center))
					{
						p[1] = i;
						point1 = lsDetection[i].element;
						break;
					}
				}

				//buscar el triangulo de mayor area
				float maxArea = 0;
				for (int i = 0; i < lsDetection.size(); i++)
				{
					if ( !lsDetection[i].equalCenter(point0) && !lsDetection[i].equalCenter(point1) )
					{
						float area = getTriangleArea(eMedia.center, point0.center, lsDetection[i].element.center);
						if ( area > maxArea )
						{
							p[2] = i;
							point2 = lsDetection[i].element;
							maxArea = area;
						}
					}
				}

				//buscar el cuadrilatero de mayor area
				maxArea = 0;
				for (int i = 0; i < lsDetection.size(); i++)
				{
					if ( !lsDetection[i].equalCenter(point0) && !lsDetection[i].equalCenter(point1)
					        && !lsDetection[i].equalCenter(point2) )
					{
						float area =
						    getTriangleArea( point0.center, point1.center, lsDetection[i].element.center) +
						    getTriangleArea( point1.center, point2.center , lsDetection[i].element.center);

						if ( area > maxArea )
						{
							p[3] = i;
							point3 = lsDetection[i].element;
							maxArea = area;
						}
					}
				}
				//hasta aqui tenemos los nuevos p[]
				//debemos buscar sus más cercanos con lsCorrection y hacer el match

				float distMin0 =  INF;
				float distMin1 =  INF;
				float distMin2 =  INF;
				float distMin3 =  INF;

				int index0, index1, index2, index3;
				for (int i = 0; i < 4; i++)
				{
					float distAux0 = lsRecovery[0].distance(lsDetection[p[i]]);
					

					if ( distAux0 < distMin0 )
					{
						distMin0 = distAux0;
						index0 = i;
					}
				}

				for (int i = 0; i < 4; i++)
				{
					if (i == index0 ) continue;
					float distAux1 = lsRecovery[15].distance(lsDetection[p[i]]);

					if ( distAux1 < distMin1 )
					{
						distMin1 = distAux1;
						index1 = i;
					}
				}

				for (int i = 0; i < 4; i++)
				{
					if (i == index0 || i == index1  ) continue;
					float distAux2 = lsRecovery[4].distance(lsDetection[p[i]]);

					if ( distAux2 < distMin2 )
					{
						distMin2 = distAux2;
						index2 = i;
					}
				}

				for (int i = 0; i < 4; i++)
				{
					if (i == index0 || i == index1 || i == index3  ) continue;
					float distAux3 = lsRecovery[19].distance(lsDetection[p[i]]);

					if ( distAux3 < distMin3 )
					{
						distMin3 = distAux3;
						index3 = i;
					}
				}

				int pAux[4];

				pAux[0] = p[index0];
				pAux[1] = p[index1];
				pAux[2] = p[index2];
				pAux[3] = p[index3];
				//reordenamos p
				p[0] = pAux[0];
				p[1] = pAux[1];
				p[2] = pAux[2];
				p[3] = pAux[3];


				//dibujar los puntos de ls recovery
				/*
				ellipse( image, lsRecovery[0].element , Scalar(255, 255, 0) , 5, 8 );
				ellipse( image, lsRecovery[16].element , Scalar(255, 255, 0) , 5, 8 );
				ellipse( image, lsRecovery[3].element , Scalar(255, 255, 0) , 5, 8 );
				ellipse( image, lsRecovery[19].element , Scalar(255, 255, 0) , 5, 8 );

				putText(image,"0" ,lsRecovery[0].element.center , FONT_HERSHEY_PLAIN, 1,Scalar(255, 0, 255), 2);
				putText(image, "1" ,lsRecovery[16].element.center , FONT_HERSHEY_PLAIN, 1,Scalar(255, 0, 255), 2);
				putText(image, "2" ,lsRecovery[3].element.center , FONT_HERSHEY_PLAIN, 1,Scalar(255, 0, 255), 2);
				putText(image, "3" ,lsRecovery[19].element.center , FONT_HERSHEY_PLAIN, 1,Scalar(255, 0, 255), 2);

				//dibujar los puntos calculados
				ellipse( image, lsDetection[p[0]].element , Scalar(255, 255, 255) , 5, 8 );
				ellipse( image, lsDetection[p[1]].element , Scalar(255, 255, 255) , 5, 8 );
				ellipse( image, lsDetection[p[2]].element , Scalar(255, 255, 255) , 5, 8 );
				ellipse( image, lsDetection[p[3]].element , Scalar(255, 255, 255) , 5, 8 );

				putText(image, "0" ,lsDetection[p[0]].element.center , FONT_HERSHEY_PLAIN, 1,Scalar(255, 0, 255), 2);
				putText(image, "1" ,lsDetection[p[1]].element.center , FONT_HERSHEY_PLAIN, 1,Scalar(255, 0, 255), 2);
				putText(image, "2" ,lsDetection[p[2]].element.center , FONT_HERSHEY_PLAIN, 1,Scalar(255, 0, 255), 2);
				putText(image, "3" ,lsDetection[p[3]].element.center , FONT_HERSHEY_PLAIN, 1,Scalar(255, 0, 255), 2);
				*/


				//ahora el algoritmo es el mismo que antes del tracking

				vector< Ellipse > lsInlineA = getInLine(lsDetection,
				                                        lsDetection[p[0]], lsDetection[p[1]]);

				vector< Ellipse > lsInlineB = getInLine(lsDetection,
				                                        lsDetection[p[2]], lsDetection[p[3]]);

				//cout<<"lsDetection.size() "<<lsDetection.size()<<endl;
				if (lsInlineA.size() != lsInlineB.size())
				{
					if (lsSort.empty())
					{
						//isTracking = true;
						isRecovery = true;
						//cout << "recovery fail" << endl;
					}
					return false;
				}

				int indice = 0;
				for (int i = 0; i < lsInlineA.size(); i++)
				{
					vector< Ellipse > lsTemp = getInLine(lsDetection,
					                                     lsInlineA[i], lsInlineB[i]);

					for (int j = 0; j < lsTemp.size(); j++)
					{
						lsTemp[j].index = indice;
						lsSort.push_back(lsTemp[j]);
						indice++;
					}
				}
				if (lsSort.size() == 20)
					lsRecovery = lsSort;



			}
			else//if( isRecovery )
			{

				float minDist0 = INF;
				float minDist1 = INF;
				float minDist2 = INF;
				float minDist3 = INF;
				for (int i = 0; i < lsDetection.size(); i++)
				{
					float dist0 = lsDetection[i].distance(lsSort[0]);
					if (dist0 < minDist0)
					{
						p[0] = i;
						minDist0 = dist0;
					}

					float dist1 = lsDetection[i].distance(lsSort[15]);
					if (dist1 < minDist1)
					{
						p[1] = i;
						minDist1 = dist1;
					}

					float dist2 = lsDetection[i].distance(lsSort[4]);
					if (dist2 < minDist2)
					{
						p[2] = i;
						minDist2 = dist2;
					}

					float dist3 = lsDetection[i].distance(lsSort[19]);
					if (dist3 < minDist3)
					{
						p[3] = i;
						minDist3 = dist3;
					}

				}

				lsSort.clear();
				vector< Ellipse > lsInlineA = getInLine(lsDetection,
				                                        lsDetection[p[0]], lsDetection[p[1]]);

				vector< Ellipse > lsInlineB = getInLine(lsDetection,
				                                        lsDetection[p[2]], lsDetection[p[3]]);

				if (lsInlineA.size() != lsInlineB.size())
				{
					//isRecovery = true;
					//lsSort = lsRecovery;
					isTracking = false;
					return false;
				}

				int indice = 0;
				for (int i = 0; i < lsInlineA.size(); i++)
				{
					vector< Ellipse > lsTemp = getInLine(lsDetection,
					                                     lsInlineA[i], lsInlineB[i]);

					for (int j = 0; j < lsTemp.size(); j++)
					{
						lsTemp[j].index = indice;
						lsSort.push_back(lsTemp[j]);
						indice++;
					}
				}

				if (lsSort.size() == 20)
					lsRecovery = lsSort;

			}


		}




		//sort( lsSort.begin(), lsSort.end(), thanCompareIndex );
		//cout<<"--------------------------------"<<endl;

		/*
		for (int i = 0; i < lsSort.size(); ++i)
		{

			//cout<<"index "<<lsSort[i].index<<endl;

			if (i != lsSort.size() - 1)
				line( image, lsSort[i].element.center, lsSort[i + 1].element.center, Scalar(0, 0, 255), 1, 8 );

				
			putText(image, to_string( lsSort[i].index ) ,
			        lsSort[i].element.center , FONT_HERSHEY_PLAIN, 1,
			        Scalar(255, 0, 255), 2);
		}*/

		return true;
	}

	void drawControlPointsNumbers(Mat& image, vector<Point2f> points )
	{
		for (int i = 0; i < points.size(); ++i)
		{
			if (i != points.size() - 1)
				line( image, points[i], points[i + 1], Scalar(0, 0, 255), 1, 8 );

				
			putText(image, to_string( i ) ,
			        points[i] , FONT_HERSHEY_PLAIN, 1,
			        Scalar(255, 0, 255), 2);
		}
	}

	void drawControlPoints(Mat& image, vector<Point2f> points, bool tracking = true )
	{
		drawChessboardCorners( image, Size(5,4), Mat(points), tracking );
		//drawControlPointsCross(image, points);
	}

	void drawControlPointsCross(Mat& image, vector<Point2f>& points, Scalar color = Scalar(0,0,255), float size = 0 )
	{
		//drawControlPoints(image, points, true);
		
		for (int i = 0; i < points.size(); ++i)
		{
			Point2f p1(points[i].x, points[i].y - size);
			Point2f p2(points[i].x, points[i].y + size);
			Point2f p3(points[i].x-size, points[i].y);
			Point2f p4(points[i].x+size, points[i].y);
			line( image, p1, p2, color, 2, 8 );
			line( image, p3, p4, color, 2, 8 );
		}
	}

	void updateROI(Mat& image)
	{

		if ( lsDetection.size() < 20) return;


		int minX = lsDetection[p[0]].element.center.x;
		int minY = lsDetection[p[0]].element.center.y;
		int maxX = lsDetection[p[0]].element.center.x;
		int maxY = lsDetection[p[0]].element.center.y;

		for ( int i = 1; i < 4; i++ )
		{
			float tempX = lsDetection[p[i]].element.center.x;
			float tempY = lsDetection[p[i]].element.center.y;

			if ( tempX < minX )
			{
				minX = tempX;
			}
			if ( tempY < minY )
			{
				minY = tempY;
			}


			if ( tempX > maxX )
			{
				maxX = tempX;
			}
			if ( tempY > maxY )
			{
				maxY = tempY;
			}
		}

		//cout<<"! "<<minX<<" "<<minY<<endl;
		//cout<<"! "<<maxX<<" "<<maxY<<endl;

		//joao: int paddingX = (maxX - minX)/5.0f;
		//joao: int paddingY = (maxY - minY)/5.0f;
		int paddingX = (maxX - minX)/2.0f; //40;
		int paddingY = (maxY - minY)/2.0f;//40;

		//cout<<"padding "<<paddingX<<" "<<paddingY<<endl;

		roi.x = max( minX - paddingX , 0 );
		roi.y = max( minY - paddingY , 0 );

		roi.width = maxX - roi.x + paddingX;
		roi.height = maxY - roi.y + paddingY;

		if (roi.x + roi.width > image.cols)
		{
			roi.width = image.cols - roi.x;
		}

		if (roi.y + roi.height > image.rows)
		{
			roi.height = image.rows - roi.y;
		}

		/*
		cout<<"JOA"<<endl;
		cout<<roi.x<<" "<<roi.y<<endl;
		cout<<roi.width<<" "<<roi.height<<endl;
		*/


	}

	bool findConcentrics( Mat& image, vector<Point2f> &pointBuf )
	{
		//view = image.clone();
		return preProcessing( image, pointBuf );
	}

	bool preProcessing(Mat& image, vector<Point2f> &pointBuf)
	{

		
		
		pointBuf.clear();

		lsDetection.clear();

		vector<Ellipse> lsEllipses, lsEllipsesAux;//, lsOriginal;

		high_resolution_clock::time_point t1 = high_resolution_clock::now();

		//Size size(640, 480); //the dst image size,e.g.100x100
		//resize(image, image, size);

		//cout<<"aqui1"<<endl;
		if (isFirtsPre)
		{
			countFailROI = 0;
			roi.x = 0.0f;
			roi.y = 0.0f;
			roi.width = image.cols;
			roi.height = image.rows;
			isFirtsPre = false;

		}
		//cout<<"aqui2"<<endl;


		RNG rng(12345);
		int max_thresh = 255;
		Mat _procImage;


		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy, hierarchyFilters;

		cvtColor(image, _procImage, CV_BGR2GRAY, 1 );


		_procImage = _procImage(roi);

		GaussianBlur( _procImage, _procImage, Size( 15, 15), 0, 0 );
		//joao: adaptiveThreshold(_procImage , _procImage, max_thresh, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 9, 2);
		adaptiveThreshold(_procImage , _procImage, max_thresh, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 5, 2);


		//imshow("_procImage", _procImage);

		//Todo: erosion y dilatacion

		//imshow("adaptiveThreshold", _procImage);
		//Canny( _procImage, _procImage, 100, 200, 3 );
		findContours( _procImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

		//lsOriginal.resize( contours.size() );

		Mat drawing = Mat::zeros( _procImage.size(), CV_8UC3 );

		Scalar color = Scalar( 0, 0, 255 );

		vector<int> filters;
		vector<int> nNeighsCenterNear;
		vector<RotatedRect> original(contours.size());
		for ( int i = 0; i < contours.size(); i++ )
		{

			// joao: if ( contours[i].size() > 5 )
			if ( contours[i].size() > 1)
			{

				filters.push_back(i);
				nNeighsCenterNear.push_back(0);
				original[i] = minAreaRect( Mat(contours[i]) );
				//drawContours( image, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
				//imshow("im", image);

			}

		}


		for (int i = 0; i < filters.size(); i++)
		{
			RotatedRect rEle = minAreaRect( Mat(contours[filters[i]]) );
			//minRect.push_back( rEle );

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

						//cout<<distance( lsEllipses[i], lsEllipses[j] )<<endl;

						//nNeighsCenterNear[i] += 1;


						
						float wDiff = abs(	max(lsEllipses[i].element.size.width, lsEllipses[i].element.size.height) -
						                    max(lsEllipses[j].element.size.width, lsEllipses[j].element.size.height) );


						float hDiff = abs(	min(lsEllipses[i].element.size.width, lsEllipses[i].element.size.height) -
						                    min(lsEllipses[j].element.size.width, lsEllipses[j].element.size.height) );

						//cout << wDiff << " " << hDiff << endl;

						if ( wDiff > 4 && wDiff < 30 && hDiff > 2 && hDiff < 50)
						{

							//cout<<"size ellipse "<<lsEllipses[i].element.size.width<<" "<<lsEllipses[i].element.size.height<<endl;
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

		//todo: !=40 revisar
		if (lsEllipsesAux.size()<40 )
		{
			countFailROI ++;

			if(countFailROI>8)
			{
				isFirtsPre = true;
			}
			return false;
		}

		//cout<<"aqui2"<<endl;

		//filtrar por la medianas del size
		vector<Ellipse> lsMedianaChild;

		for(int i=0; i<lsEllipsesAux.size(); i++)
		{
			int child = hierarchy[ lsEllipsesAux[i].index ][2];
			int nChilds = numberChilds(hierarchy, lsEllipsesAux[i].index);
			if ( nChilds == 1 )
			{
				lsMedianaChild.push_back(lsEllipsesAux[i]);

			}
		}



		//cout<<"lsMedianaChild.size() "<<lsMedianaChild.size()<<endl;

		sort(lsMedianaChild.begin(), lsMedianaChild.end(), lessCompareW);
		float mediaW = lsMedianaChild[lsMedianaChild.size()/2].element.size.width;
		sort(lsMedianaChild.begin(), lsMedianaChild.end(), lessCompareH);
		float mediaH = lsMedianaChild[lsMedianaChild.size()/2].element.size.height;

		//cout<<"media W H "<<mediaW<<" "<<mediaH<<endl;

		lsMedianaChild.clear();
		for(int i=0; i<lsEllipsesAux.size(); i++)
		{
			int child = hierarchy[ lsEllipsesAux[i].index ][2];
			int nChilds = numberChilds(hierarchy, lsEllipsesAux[i].index);
			if ( nChilds == 1 )
			{
				float diffSize = abs(lsEllipsesAux[i].element.size.width + lsEllipsesAux[i].element.size.height - mediaW - mediaH);
				if(diffSize < 10)
				{
					lsMedianaChild.push_back(lsEllipsesAux[i]);
				}
			}
		}

		//-----------------------------
		//cout<<"oooohwwjj"<<endl;


		lsEllipses = lsMedianaChild; //lsEllipsesAux;
		//cout<<"size "<<lsEllipses.size()<<endl;
		//minRect = aux;

		if (lsEllipses.empty()) return false;



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


		//cout << lsEllipsesAux.size() << endl;
		mediana.size.width = 2 * r;
		mediana.size.height = 2 * r;

		lsEllipses = lsEllipsesAux;

		//ellipse( image, mediana , Scalar(0, 255, 0) , 1, 8 );





		for (int i = 0; i < lsEllipses.size(); i++)
		{

			int child = hierarchy[ lsEllipses[i].index ][2];

			int nChilds = numberChilds(hierarchy, lsEllipses[i].index);

			//roi Correction
			lsEllipses[i].element.center.x += roi.x;
			lsEllipses[i].element.center.y += roi.y;
			original[child].center.x += roi.x;
			original[child].center.y += roi.y;

			//if ( child != -1 )
			if ( nChilds == 1 )
			{
				//cout<<"child "<<i<<" "<<child<<endl;

				/*
				float x = (original[child].center.x + lsEllipses[i].element.center.x) / 2;
				float y = (original[child].center.y + lsEllipses[i].element.center.y) / 2;

				RotatedRect ellipseCenter;

				ellipseCenter.center.x = x;
				ellipseCenter.center.y = y;

				ellipseCenter.size.width = 5;
				ellipseCenter.size.height = 5;

				Ellipse element = Ellipse(0, ellipseCenter);

				//lsDetection.push_back(element);*/

				lsEllipses[i].element.center.x = ( lsEllipses[i].element.center.x + original[child].center.x )/2.0f;			
				lsEllipses[i].element.center.y = ( lsEllipses[i].element.center.y + original[child].center.y )/2.0f;



				//lsEllipses[i].element.size.width = 1;
				//lsEllipses[i].element.size.height = 1;




				lsDetection.push_back(lsEllipses[i]);

				//ellipses.push_back(ellipseCenter);
			}

			//no borrar
			//ellipse( view, lsEllipses[i].element , Scalar(0, 255, 0) , 1, 8 );
			//ellipse( view, original[child] , Scalar(255, 255, 0) , 1, 8 );
		}




		//cout << "size lsDetection " << lsDetection.size() << endl;
		for (int i = 0; i < lsDetection.size(); i++)
		{
			//no borrar0
			//ellipse( image, lsDetection[i].element , Scalar(255, 255, 0) , 2, 8 );
		}

		
		//waitKey();

		high_resolution_clock::time_point t2 = high_resolution_clock::now();
		duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

		bool ret =  tracking(image);



		if(lsDetection.size() == 20 && !isRecovery)
		{
			updateROI(image);
		}
		else
		{
			isFirtsPre = true;
		}
		


		time = time_span.count();
		//cout << time << endl;

		//llenamos pointbuff
		if(ret)
		{
			for(int i = 0 ; i<20; i++)
			{
				pointBuf.push_back(lsSort[i].element.center);
			}
		}



		return ret;



	}

};
//ProcManager* ProcManager::INSTANCE = NULL;
}