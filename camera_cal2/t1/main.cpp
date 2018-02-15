#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>

#include <cmath>

#include <chrono>

using namespace std;
using namespace cv;


using namespace chrono;


struct InfoEllipse
{
	RotatedRect* ellipse ;
	int index;
	int corner;
	float weight;
	int indexTrack;

	InfoEllipse(int _index, RotatedRect* _ellipse, int _indexTrack=-1)
	{
		index = _index;
		ellipse = _ellipse;
		weight = 0;
		corner = -1;
		indexTrack = _indexTrack;
	}

	float distance_center(InfoEllipse* other)
	{
		return sqrt( pow(ellipse->center.x - other->ellipse->center.x, 2) + pow (ellipse->center.y - other->ellipse->center.y, 2) );
	}

	float distance_size(InfoEllipse* other)
	{
		return sqrt( 	pow(ellipse->size.width - other->ellipse->size.width, 2)
		                + pow (ellipse->size.height - other->ellipse->size.height, 2) );
	}

	float sum_size()
	{
		return ellipse->size.width + ellipse->size.height;
	}
};


int black_level_binary;
int min_size_pattern;
bool isInitTrack = false;

vector<InfoEllipse*> lsTrackAnt;

void captureFrames(const string& path_video, const string& path_to_frames, bool show);
Mat getBinaryImage(const Mat& image);
void detect_pattern(Mat& image);
void cost_func(vector<InfoEllipse*> &ls_info_ellipse);
void func_radio_center(InfoEllipse *item, vector<InfoEllipse*> &ls_info_ellipse);
void func_radio_neigh(InfoEllipse *item, vector<InfoEllipse*> &ls_info_ellipse);
bool compare_center_x(InfoEllipse *A, InfoEllipse *B);
bool compare_center_y(InfoEllipse *A, InfoEllipse *B);
bool compare_size(InfoEllipse *A, InfoEllipse *B);
float distance_center(RotatedRect &a, RotatedRect &b);
float distance_center(RotatedRect* a, float x, float y);
void drawLines(vector<InfoEllipse*> selecteds);


int main()
{

	black_level_binary = 80;
	min_size_pattern = 5;


	/*captureFrames("../res/videos/calibration_kinectv2.avi", "../res/images/frames/", true);*/



	
	Mat frame;
	frame = imread( "../res/images/frames/frame634.jpg");

	if (!frame.data )
	{
		cout << "Image no read" << endl;
		return -1;
	}


	detect_pattern(frame);
	imshow("w", frame);

	waitKey(); // waits to display frame*/
	


	
	


	
	//string filename("../res/videos/calibration_kinectv2.avi");

	//string filename("../res/videos/calibration_ps3eyecam.avi");
	//VideoCapture capture(filename.c_str());


/*
	VideoCapture capture(0);

	Mat frame;

	if ( !capture.isOpened() )
	{
		cout << "Error when reading steam_avi" << endl;
		return -1;
	}

	int i = 0;
	namedWindow( "w", 1);

	//VideoWriter video("out_msps3.avi",CV_FOURCC('M','J','P','G'),60, Size(640,480),true);
	for ( ; ; )
	{
		capture >> frame;

		if (frame.empty())
		{
			break;
		}

		

		detect_pattern(frame);

		imshow("frame" , frame);
		video.write(frame);

		//imwrite( "../res/images/frames_info/img"+to_string(i)+".jpg", frame );

		waitKey(10); // waits to display frame

		//cout<<i<<endl;
		i++;

	}

*/
	return 0;
}

Mat getBinaryImage(const Mat& image)
{
	Mat hsv, mask;
	Mat dst = image;
	cvtColor(dst, dst, CV_BGR2GRAY, 1 );
	return dst;
}

//--------------------------------------------------------------------------------------
bool unavez = true;


struct Objtracking
{
	RotatedRect ellipse;
	int indexTrack;

	float distance_center(Objtracking other)
	{
		return sqrt( pow(ellipse.center.x - other.ellipse.center.x, 2) + pow (ellipse.center.y - other.ellipse.center.y, 2) );
	}

};

vector<Objtracking> lsAnterior;
vector<Objtracking> lsActual;

void tracking(vector<InfoEllipse*> selecteds , Mat& image)
{


	if (selecteds.size() == 20)
	{
		if (!isInitTrack)
		{
			//cout<<"mi primera vez"<<endl;
			sort(selecteds.begin(), selecteds.end(), compare_center_x);
			sort(selecteds.begin(), selecteds.end(), compare_center_y);
			for (int i = 0; i < selecteds.size() ; i++)
			{
				//selecteds[i]->indexTrack = i;
				//putText(image, to_string(i) , selecteds[i]->ellipse->center , FONT_HERSHEY_PLAIN, 1,  Scalar(0,0,255), 2);
				Objtracking t;
				t.ellipse = *(selecteds[i]->ellipse);
				t.indexTrack = i;
				lsActual.push_back(t);

				//putText(image, to_string( lsActual[i].indexTrack ) , lsActual[i].ellipse.center , FONT_HERSHEY_PLAIN, 1,  Scalar(0, 255, 0), 2);
			}
			isInitTrack = true;

			/*
			for( int i = 0 ; i<selecteds.size(); i++)
			{
				cout<<selecteds[i]->ellipse->center <<endl;
			}*/

		}

		else
		{


			for (int i = 0; i < selecteds.size() ; i++)
			{
				Objtracking t;
				t.ellipse = *(selecteds[i]->ellipse);
				lsActual.push_back(t);
			}
			//if(!unavez) return;
			//cout<<"------------------"<<endl;

			/*
			for( int i = 0 ; i<selecteds.size(); i++)
			{
				cout<<selecteds[i]->ellipse->center <<endl;
			}
			*/

			if (lsAnterior.size() > 0)
			{
				
				//logica

				for(int j = 0; j<lsActual.size() ; j++)
				{
					float distancia = lsAnterior[j].distance_center(lsActual[j]);
					int indexSelected = j;


					for (int i = 0; i < selecteds.size(); i++)
					{
						float distAct = lsAnterior[j].distance_center(lsActual[i]);

						if (distAct < distancia)
						{
							distancia = distAct;
							indexSelected = i;
						}
					}

					//lsActual[indexSelected].ellipse = lsAnterior[0].ellipse;
					lsActual[indexSelected].indexTrack = lsAnterior[j].indexTrack;

					putText(image, to_string( lsActual[indexSelected].indexTrack ) , lsActual[indexSelected].ellipse.center , FONT_HERSHEY_PLAIN, 1,  Scalar(0, 255, 0), 2);
				}
				

				//cout<<selecteds[indexSelected]->indexTrack<<endl;

			}
		}


		lsAnterior = lsActual;

		//cout<<lsAnterior[0].ellipse.center<<endl;

		lsActual.clear();
		
		/*lsTrackAnt.clear();

		
		for(int i=0; i<selecteds.size(); i++)
		{
			RotatedRect* x = new RotatedRect(selecteds[i]->ellipse->center,selecteds[i]->ellipse->size, selecteds[i]->ellipse->angle );
			lsTrackAnt.push_back(new InfoEllipse(selecteds[i]->index, x, selecteds[i]->indexTrack));
		}*/
		
		//cout<<lsTrackAnt.size()<<endl;

	}






}

void detect_pattern(Mat& image)
{

	high_resolution_clock::time_point t1 = high_resolution_clock::now();
	


	Size size(640, 480); //the dst image size,e.g.100x100
	resize(image, image, size);

	Mat src_gray = getBinaryImage(image);

	vector<InfoEllipse*> ls_info_ellipse, selecteds;
	int thresh = 100;
	int max_thresh = 255;
	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	GaussianBlur( src_gray, src_gray, Size( 11, 11), 0, 0 );

	adaptiveThreshold(src_gray , src_gray, max_thresh, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 15, 2);
	//threshold( src_gray, src_gray, 100, max_thresh, THRESH_BINARY );

	imshow("thresh", src_gray);

	/// Find contours
	findContours( src_gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	vector<RotatedRect> minEllipse( contours.size() );
	vector<int> ie;

	for ( int i = 0; i < contours.size(); i++ )
	{
		if ( contours[i].size() > 5)
		{
			minEllipse[i] = fitEllipse( Mat(contours[i]) );
			if ( minEllipse[i].size.width + minEllipse[i].size.height > min_size_pattern)
			{
				ls_info_ellipse.push_back(new InfoEllipse(i, &minEllipse[i]));
			}
		}
	}


	cost_func(ls_info_ellipse);



	high_resolution_clock::time_point t2 = high_resolution_clock::now();
	duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

	float fps = (1.0f / time_span.count());


	putText(image, "FPS: "+to_string(fps) , Point2f(20.0f,20.0f) , FONT_HERSHEY_PLAIN, 1.5,  Scalar(0, 255, 0, 100), 2);

	for ( int i = 0; i < ls_info_ellipse.size(); i++ )
	{

		Scalar red = Scalar( 0, 0, 255 );
		Scalar blue = Scalar( 255, 0, 0 );
		if (ls_info_ellipse[i]->weight > 1)
		{
			ellipse( image, *(ls_info_ellipse[i]->ellipse), blue, 1, 8 );
			int child = hierarchy[ls_info_ellipse[i]->index][2];
			if (child != -1)
			{
				float x = (minEllipse[child].center.x + ls_info_ellipse[i]->ellipse->center.x) / 2;
				float y = (minEllipse[child].center.y + ls_info_ellipse[i]->ellipse->center.y) / 2;

				ls_info_ellipse[i]->ellipse->center.x = x;
				ls_info_ellipse[i]->ellipse->center.y = y;
				ls_info_ellipse[i]->ellipse->size.width = 5;
				ls_info_ellipse[i]->ellipse->size.height = 5;
				selecteds.push_back(ls_info_ellipse[i]);
				ellipse( image, *(ls_info_ellipse[i]->ellipse), red, 5, 8 );
			}
		}
	}

	if (selecteds.size() > 0)
	{
		//tracking(selecteds , image);
	}

}

//funciones objetivos

void cost_func(vector<InfoEllipse*> &ls_info_ellipse)
{
	for ( int i = 0; i < ls_info_ellipse.size(); i++ )
	{
		func_radio_center(ls_info_ellipse[i], ls_info_ellipse);
	}

	for ( int i = 0; i < ls_info_ellipse.size(); i++ )
	{
		func_radio_neigh(ls_info_ellipse[i], ls_info_ellipse);
	}


	//mediana heuristic
	/*
	if (ls_info_ellipse.size() >= 2)
	{

		float mx, my;
		sort(ls_info_ellipse.begin(), ls_info_ellipse.end(), compare_center_x);
		mx = ls_info_ellipse[ls_info_ellipse.size() / 2]->ellipse->center.x;
		sort(ls_info_ellipse.begin(), ls_info_ellipse.end(), compare_center_y);
		my = ls_info_ellipse[ls_info_ellipse.size() / 2]->ellipse->center.y;
		//cout << "mx : " << mx << ", my :" << my << endl;


		float size_median = 0;
		sort(ls_info_ellipse.begin(), ls_info_ellipse.end(), compare_size);
		size_median = ls_info_ellipse[ls_info_ellipse.size() / 2]->ellipse->size.width + ls_info_ellipse[ls_info_ellipse.size() / 2]->ellipse->size.height;

		//cout<<"size_median : "<<size_median<<endl;

		vector<InfoEllipse*> lsEllipses_aux;
		for (int i = 0; i < ls_info_ellipse.size(); i++)
		{
			if ( (ls_info_ellipse[i]->sum_size()<size_median+50 && ls_info_ellipse[i]->sum_size()>size_median-50) )
			{
				lsEllipses_aux.push_back(ls_info_ellipse[i]);
			}
		}

		ls_info_ellipse = lsEllipses_aux;
	}
	*/

}

void func_radio_center(InfoEllipse *item, vector<InfoEllipse*> &ls_info_ellipse)
{
	float w_radio_center = 1;
	float radio_center = 1.0f;
	int count_neigh_center = 0;

	//heuristica de distancia entre centros
	for (int i = 0; i < ls_info_ellipse.size(); i++)
	{
		InfoEllipse* neigh = ls_info_ellipse[i];
		if ( item->distance_center(neigh) < radio_center && neigh != item)
		{

			count_neigh_center++;
		}
	}

	//premiacion al candidato
	if (count_neigh_center == 1)
	{
		item->weight += w_radio_center;
	}
}

void func_radio_neigh(InfoEllipse *item, vector<InfoEllipse*> &ls_info_ellipse)
{
	float w_radio_center = 1;
	float radio_center = 20.0f;
	int count_neigh_center = 0;

	for (int i = 0; i < ls_info_ellipse.size(); i++)
	{
		InfoEllipse* neigh = ls_info_ellipse[i];
		if ( item->distance_center(neigh) < radio_center && neigh != item && neigh->weight > 0)
		{
			count_neigh_center++;
		}
	}

	//premiacion al candidato
	if (count_neigh_center > 0)
	{
		item->weight += w_radio_center;
	}
}



void captureFrames(const string& path_video, const string& path_to_frames, bool show)
{

	//"../res/videos/Kinect2_rgb.avi"
	string filename = path_video;
	VideoCapture capture(filename.c_str());
	Mat frame;

	if ( !capture.isOpened() )
	{
		cout << "Error when reading steam_avi" << endl;
	}

	namedWindow( "w", 1);
	int i = 0;

	cout << "Reading frames" << endl;
	for ( ; ; )
	{
		string path = path_to_frames + string("frame") + to_string(i) + string(".jpg");
		capture >> frame;
		if (frame.empty())
			break;

		if (show)
		{
			imshow("w", frame);
			waitKey(10); // waits to display frame
		}
		imwrite( path, frame );
		i++;
	}

	cout << "all frames was readed" << endl;

	waitKey(0);
}


bool compare_center_x(InfoEllipse *A, InfoEllipse *B)
{
	return A->ellipse->center.x < B->ellipse->center.x;
}


bool compare_center_y(InfoEllipse *A, InfoEllipse *B)
{
	return A->ellipse->center.y < B->ellipse->center.y;
}

bool compare_size(InfoEllipse *A, InfoEllipse *B)
{
	float sizeA = A->ellipse->size.width + A->ellipse->size.height;
	float sizeB = B->ellipse->size.width + B->ellipse->size.height;
	return sizeA < sizeB;
}


float distance_center(RotatedRect &a, RotatedRect &b)
{
	return sqrt( pow(a.center.x - b.center.x, 2)
	             + pow (a.center.y - b.center.y, 2) );
}

float distance_center(RotatedRect* a, float x, float y)
{
	return sqrt( pow(a->center.x - x, 2)
	             + pow (a->center.y - y, 2) );
}