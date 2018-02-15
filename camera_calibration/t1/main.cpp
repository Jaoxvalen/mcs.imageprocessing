#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>

#include <cmath>

using namespace std;
using namespace cv;


struct InfoEllipse
{
	RotatedRect* ellipse ;
	int index;

	float weight;
	float distance_neigh;

	InfoEllipse(int _index, RotatedRect* _ellipse)
	{
		index = _index;
		ellipse = _ellipse;
		weight = 0;
		distance_neigh = 0;
	}

	float distance_center(InfoEllipse* other)
	{
		return sqrt( pow(ellipse->center.x-other->ellipse->center.x, 2) 
			+ pow (ellipse->center.y-other->ellipse->center.y, 2) );
	}

	float distance_size(InfoEllipse* other)
	{
		return sqrt( 	pow(ellipse->size.width-other->ellipse->size.width, 2) 
						+ pow (ellipse->size.height-other->ellipse->size.height, 2) );
	}

	float sum_size()
	{
		return ellipse->size.width + ellipse->size.height;
	}

	
};



#define S (IMAGE_WIDTH/8)
#define T (0.5f)
void adaptiveThreshold(unsigned char* input, unsigned char* bin, int IMAGE_WIDTH, int IMAGE_HEIGHT)
{
	unsigned long* integralImg = 0;
	int i, j;
	long sum=0;
	int count=0;
	int index;
	int x1, y1, x2, y2;
	int s2 = S/2;

	// create the integral image
	integralImg = (unsigned long*)malloc(IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(unsigned long*));

	for (i=0; i<IMAGE_WIDTH; i++)
	{
		// reset this column sum
		sum = 0;

		for (j=0; j<IMAGE_HEIGHT; j++)
		{
			index = j*IMAGE_WIDTH+i;

			sum += input[index];
			if (i==0)
				integralImg[index] = sum;
			else
				integralImg[index] = integralImg[index-1] + sum;
		}
	}

	// perform thresholding
	for (i=0; i<IMAGE_WIDTH; i++)
	{
		for (j=0; j<IMAGE_HEIGHT; j++)
		{
			index = j*IMAGE_WIDTH+i;

			// set the SxS region
			x1=i-s2; x2=i+s2;
			y1=j-s2; y2=j+s2;

			// check the border
			if (x1 < 0) x1 = 0;
			if (x2 >= IMAGE_WIDTH) x2 = IMAGE_WIDTH-1;
			if (y1 < 0) y1 = 0;
			if (y2 >= IMAGE_HEIGHT) y2 = IMAGE_HEIGHT-1;
			
			count = (x2-x1)*(y2-y1);

			// I(x,y)=s(x2,y2)-s(x1,y2)-s(x2,y1)+s(x1,x1)
			sum = integralImg[y2*IMAGE_WIDTH+x2] -
				  integralImg[y1*IMAGE_WIDTH+x2] -
				  integralImg[y2*IMAGE_WIDTH+x1] +
				  integralImg[y1*IMAGE_WIDTH+x1];

			if ((long)(input[index]*count) < (long)(sum*(1.0-T)))
				bin[index] = 0;
			else
				bin[index] = 255;
		}
	}

	free (integralImg);
}

void captureFrames(const string& path_video, const string& path_to_frames, bool show);
Mat getBinaryImage(const Mat& image);
void detect_pattern(Mat& image);
int n_neights(InfoEllipse *item, vector<InfoEllipse*> &ls_info_ellipse, float radio);
void cost_func(vector<InfoEllipse*> &ls_info_ellipse);
void func_radio_center(InfoEllipse *item, vector<InfoEllipse*> &ls_info_ellipse);
void func_dist_neigh(InfoEllipse *item, vector<InfoEllipse*> &ls_info_ellipse);
InfoEllipse* calc_media_size(vector<InfoEllipse*> &ls_info_ellipse);
void func_size(InfoEllipse *item, InfoEllipse *media_size);
int count_jerarquia(InfoEllipse* item, vector<Vec4i> hierarchy, int type);
InfoEllipse* calc_mediana_size(vector<InfoEllipse*> &ls_info_ellipse);
void func_mediana_center(InfoEllipse *item, InfoEllipse *mediana);

bool comp_size(InfoEllipse *a, InfoEllipse *b)
{
	return ( a->ellipse->size.width + a->ellipse->size.height ) > ( b->ellipse->size.width + b->ellipse->size.height )  ;
}


bool comp_dist_neigh(InfoEllipse *a, InfoEllipse *b)
{
	return ( a->distance_neigh ) > (  b->distance_neigh )  ;
}

bool comp_x(InfoEllipse *a, InfoEllipse *b)
{
	return ( a->ellipse->center.x ) > (  b->ellipse->center.x );
}

bool comp_y(InfoEllipse *a, InfoEllipse *b)
{
	return ( a->ellipse->center.y ) > (  b->ellipse->center.y );
}


int black_level_binary;
int min_size_pattern;
float scale_factor;
int main()
{


	/*KINECT 2 PARAMETERS**/
	black_level_binary = 80;
	min_size_pattern = 12;

	/***/

	/*captureFrames("../res/videos/calibration_kinectv2.avi", "../res/images/frames/", true);*/



	/*
	Mat frame;
	frame = imread( "../res/images/frames/frame283.jpg");

	if (!frame.data )
	{
		cout << "Image no read" << endl;
		return -1;
	}


	//detect_pattern(frame);
	imshow("w", frame);

	waitKey(); // waits to display frame
	


	
	//VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'),10, Size(960,540),true);

*/


	string filename("../res/videos/calibration_kinectv2.avi");

	//string filename("../res/videos/calibration_ps3eyecam.avi");
	//VideoCapture capture(filename.c_str());

	VideoCapture capture(0);

	Mat frame;

	if ( !capture.isOpened() )
	{
		cout << "Error when reading steam_avi" << endl;
		return -1;
	}
	
	int i = 0;
	namedWindow( "w", 1);
	for ( ; ; )
	{
		capture >> frame;
		if (frame.empty())
		{
			break;
		}
		detect_pattern(frame);
		imshow("w", frame);
		//video.write(frame);
		waitKey(10); // waits to display frame

		//cout<<i<<endl;
		i++;

	}
	
	
	

	return 0;
}

Mat getBinaryImage(const Mat& image)
{
	Mat hsv, mask;

	Mat dst = image;
	
	/*
	Mat dst(image.rows, image.cols, CV_8UC3, Scalar(255, 255, 255));
	cv::cvtColor(image, hsv, COLOR_RGB2HSV, 4);

	inRange(hsv, Scalar(0, 0, 0), Scalar(180, 255, 80), mask);
	bitwise_and(image, image, dst, mask = mask);*/
	cvtColor(dst, dst, CV_BGR2GRAY, 1 );

	return dst;
}

void detect_pattern(Mat& image)
{




	Size size(960,540);//the dst image size,e.g.100x100
	resize(image, image, size);

	Mat src_gray = getBinaryImage(image);

	//cout<<image.size<<endl;



	/*
	Mat imgt = image.clone();
	cvtColor(imgt, imgt, CV_BGR2GRAY, 1 );
	IplImage* binImg;
	binImg = cvCreateImage(cvSize(imgt.cols, imgt.rows), 8, 1);
	adaptiveThreshold((unsigned char*)(imgt.data), (unsigned char*)binImg->imageData, imgt.rows, imgt.cols);
	cvShowImage("Output", binImg);
	*/

	vector<InfoEllipse*> ls_info_ellipse;
	int thresh = 100;
	int max_thresh = 255;
	RNG rng(12345);
	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	//threshold_output = src_gray;

	/// Detect edges using Threshold
	//threshold( src_gray, src_gray, thresh, max_thresh, THRESH_BINARY );

	adaptiveThreshold(src_gray , src_gray, max_thresh, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 15, 2);

	imshow("binary", src_gray);

	/// Find contours
	findContours( src_gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	vector<RotatedRect> minEllipse( contours.size() );
	vector<int> ie;

	for ( int i = 0; i < contours.size(); i++ )
	{
		if ( contours[i].size() > 5)
		{
			minEllipse[i] = fitEllipse( Mat(contours[i]) );
			if( minEllipse[i].size.width+ minEllipse[i].size.height>min_size_pattern)
				ls_info_ellipse.push_back(new InfoEllipse(i, &minEllipse[i]));
		}
	}

	//Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
	
	cost_func(ls_info_ellipse);

	//cout<<"n ellipses "<<ls_info_ellipse.size()<<endl;
	for ( int i = 0; i < ls_info_ellipse.size(); i++ )
	{
		//Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255) );
		

		
		
		//int n_childs = count_jerarquia(ls_info_ellipse[i], hierarchy, 2);
		//int n_same = count_jerarquia(ls_info_ellipse[i], hierarchy, 0) + count_jerarquia(ls_info_ellipse[i], hierarchy, 1);
		//int n_parents = count_jerarquia(ls_info_ellipse[i], hierarchy, 3);

		//cout<<"childs "<<n_childs<<" same "<<n_same<<" parents "<<n_parents<<endl;


		Scalar color = Scalar( 0, 0, 255 );
		//if(n_neights(ls_info_ellipse[i], ls_info_ellipse, 1.0f) > 0)

		//cout<<ls_info_ellipse[i]->ellipse->size<<endl;
		//cout<<ls_info_ellipse[i]->distance_neigh<<endl;
		//if(ls_info_ellipse[i]->weight>0 && n_childs==0 && n_same==0 && n_parents>1)
		if(ls_info_ellipse[i]->weight>0)
		{
			ellipse( image, *(ls_info_ellipse[i]->ellipse), color, 1, 8 );
		}
		
	}
}

//funciones objetivos

int count_jerarquia(InfoEllipse* item, vector<Vec4i> hierarchy, int type)
{
	int child = hierarchy[item->index][type];
	int c=0;
	while(child!=-1)
	{
		c++;
		child = hierarchy[child][type];
		if(c>2)
		{
			break;	
		} 
	}
	cout<<c<<endl;
	return c;
}

void cost_func(vector<InfoEllipse*> &ls_info_ellipse)
{
	

	InfoEllipse* mediana = calc_mediana_size(ls_info_ellipse);

	for ( int i = 0; i<ls_info_ellipse.size(); i++ )
	{
		func_radio_center(ls_info_ellipse[i], ls_info_ellipse);
		//func_mediana_center(ls_info_ellipse[i], mediana);
		//func_dist_neigh(ls_info_ellipse[i], ls_info_ellipse);
	}

	//sort(ls_info_ellipse.begin(), ls_info_ellipse.end(), comp_dist_neigh);
	/*
	int umbralsize = 20;
	vector<InfoEllipse*> final_cand;

	vector<InfoEllipse*> candidates;
	//ordenamos por suma de sizes
	for(int i=0; i<ls_info_ellipse.size(); i++)
	{
		InfoEllipse* actual = ls_info_ellipse[i];
		if(actual->weight>0)
		{
			candidates.push_back(actual); 
		}
	}

	sort(candidates.begin(), candidates.end(), comp_size );//joao

	if(candidates.size()>0)
	{
		float oldValue = candidates[0]->ellipse->size.width + candidates[0]->ellipse->size.height;
		int count_break=0;
		int index = 0;
		for( int i = 1; i<candidates.size(); i++ )
		{
			float actualValue = candidates[i]->ellipse->size.width + candidates[i]->ellipse->size.height;
			final_cand.push_back(candidates[i]);
			if(abs(oldValue - actualValue)>umbralsize)
			{
				count_break++;
			}
			if(count_break>4)
			{
				index = i;
				break;
			}
			//cout<<"size"<<candidates[i]->ellipse->size<<endl;
		}
		ls_info_ellipse = final_cand;

	}*/



	/*
	InfoEllipse* media = calc_media_size(ls_info_ellipse);
	cout<<media->ellipse->size<<endl;

	for ( int i = 0; i<ls_info_ellipse.size(); i++ )
	{
		func_size(ls_info_ellipse[i], media);
	}*/

	

}

void func_dist_neigh(InfoEllipse *item, vector<InfoEllipse*> &ls_info_ellipse)
{
	float w_radio_center = 1;
	float radio_center = 2.0f;
	int count_neigh_center = 0;

	//heuristica de distancia entre centros
	for(int i=0; i<ls_info_ellipse.size(); i++)
	{
		InfoEllipse* neigh = ls_info_ellipse[i];
		//cout<<item->distance_center(neigh)<<endl;

		if(neigh!=item)
		{
			item->distance_neigh+=item->distance_center(neigh);
		}
		

		/*if( item->distance_center(neigh)<radio_center && neigh!=item)
		{
			count_neigh_center++;
		}*/
	}

	//premiacion al candidato
	/*if(count_neigh_center>0)
	{
		item->weight+=w_radio_center;
	}*/
}


void func_mediana_center(InfoEllipse *item, InfoEllipse *mediana)
{


	int count = 0;
	cout<<mediana->distance_center(item)<<endl;
	if( mediana->distance_center(item)>450 )
	{
		count++;
	}

	//premiacion al candidato
	if(count>0)
	{
		item->weight+=1;
	}
}


void func_radio_center(InfoEllipse *item, vector<InfoEllipse*> &ls_info_ellipse)
{
	float w_radio_center = 1;
	float radio_center = 1.0f;
	int count_neigh_center = 0;

	//heuristica de distancia entre centros
	for(int i=0; i<ls_info_ellipse.size(); i++)
	{
		InfoEllipse* neigh = ls_info_ellipse[i];
		//cout<<item->distance_center(neigh)<<endl;
		if( item->distance_center(neigh)<radio_center && neigh!=item)
		{
			count_neigh_center++;
		}
	}

	//premiacion al candidato
	if(count_neigh_center>0)
	{
		item->weight+=w_radio_center;
	}
}



InfoEllipse* calc_media_size(vector<InfoEllipse*> &ls_info_ellipse)
{
		//media de los sizes de los lideres
	float acum_x = 0;
	float acum_y = 0;
	int counter = 0;
	int umbralsize = 20;

	vector<InfoEllipse*> candidates;
	//ordenamos por suma de sizes
	for(int i=0; i<ls_info_ellipse.size(); i++)
	{
		InfoEllipse* actual = ls_info_ellipse[i];
		if(actual->weight>0)
		{
			candidates.push_back(actual); 
		}
	}

	sort(candidates.begin(), candidates.end(), comp_size );//joao


	if(candidates.size()>0)
	{
		float oldValue = candidates[0]->ellipse->size.width + candidates[0]->ellipse->size.height;
		int count_break=0;
		int index = 0;
		for( int i = 1; i<candidates.size(); i++ )
		{
			float actualValue = candidates[i]->ellipse->size.width + candidates[i]->ellipse->size.height;
			if(abs(oldValue - actualValue)>umbralsize)
			{
				count_break++;
			}
			if(count_break>1)
			{
				index = i;
				break;
			}
			//cout<<"size"<<candidates[i]->ellipse->size<<endl;
		}

		for(int i=0; i<candidates.size(); i++)
		{
			InfoEllipse* actual = candidates[i];
			if(actual->weight>0)
			{
				acum_x += actual->ellipse->size.width;
				acum_y += actual->ellipse->size.height;
				counter++;
			}
		}
	}
	else
	{
		for(int i=0; i<ls_info_ellipse.size(); i++)
		{
			InfoEllipse* actual = ls_info_ellipse[i];
			if(actual->weight>0)
			{
				acum_x += actual->ellipse->size.width;
				acum_y += actual->ellipse->size.height;
				counter++;
			}
		}
	}
	

	acum_x = acum_x/(float)counter;
	acum_y = acum_y/(float)counter;

	InfoEllipse* media = new InfoEllipse(-1, new RotatedRect(Point2f(0,0), Size2f(acum_x, acum_y), 0));
	return media;
}

void func_size(InfoEllipse *item, InfoEllipse *media_size)
{
	float value = 40.0f;
	//premiacion al candidato
	float distance = item->distance_size(media_size);
	//cout<<"dista:"<<distance<<endl;
	if(distance <= value)
	{
		item->weight+=1;
	}
}

//fin funciones objetivos

int n_neights(InfoEllipse *item, vector<InfoEllipse*> &ls_info_ellipse, float radio)
{
	int count = 0;
	for(int i=0; i<ls_info_ellipse.size(); i++)
	{
		InfoEllipse* neigh = ls_info_ellipse[i];
		//cout<<item->distance_center(neigh)<<endl;
		if( item->distance_center(neigh)<radio && neigh!=item)
		{
			count++;
		}
	}
	return count;
}


InfoEllipse* calc_mediana_size(vector<InfoEllipse*> &ls_info_ellipse)
{

	float x, y;

	sort(ls_info_ellipse.begin(), ls_info_ellipse.end(), comp_x);
	x = ls_info_ellipse[ls_info_ellipse.size()/2]->ellipse->center.x;

	sort(ls_info_ellipse.begin(), ls_info_ellipse.end(), comp_y);
	y = ls_info_ellipse[ls_info_ellipse.size()/2]->ellipse->center.y;

	InfoEllipse* mediana = new InfoEllipse(-1, new RotatedRect(Point2f(0,0), Size2f(x, y), 0));

	return mediana;

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