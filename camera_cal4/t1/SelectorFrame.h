#include <map>
#include "RingsDetector.h"
#include "ProcManager.h"

using namespace std;
using namespace cv;


namespace vision
{

struct classcomp {
  bool operator() (const Rect& lhs, const Rect& rhs) const
  {
    if (lhs.x != rhs.x ) return lhs.x < rhs.x;
    if (lhs.y != rhs.y ) return lhs.y < rhs.y;
    return true;
  }
};



static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners)
{
  corners.clear();
  for ( int i = 0; i < boardSize.height; ++i )
    for ( int j = 0; j < boardSize.width; ++j )
      corners.push_back(Point3f(j * squareSize, i * squareSize, 0));
}


void get_FramesBySquare(map<Rect, vector<pair<Mat, int> >, classcomp> &_r2mat, int _framesBySquare,
                        vector<int> & _frameIndex, Mat camera_matrix = Mat() ,
                        Mat dist_coeffs = Mat(), double min_rot = 0.0, double max_rot = 0.0) {

  vector<int> indexes;

  for (auto &  _pair : _r2mat) {
    //by square
    int frames_by_square = 0;
    if (!camera_matrix.empty() && !dist_coeffs.empty() ) {
      //Output rotation and translation
      cv::Mat rotation_vector;
      cv::Mat translation_vector;

      for (pair<Mat, int> _pm : _pair.second) {
        //by frame

        vector<Point2f> image_points;
        //fill image points

        RingsDetector rd;

        Mat clone = _pm.first;

        //imshow("_pm.first", _pm.first);
        //waitKey();

        bool found = rd.findPattern(clone, image_points);

        cout << "found: " << found << endl;


        vector<Point3f> model_points;
        //Fill model points
        double squareSize  = 44;
        calcBoardCornerPositions(Size(5, 4), squareSize, model_points);




        //Solve for pose

        cout << "size model image: " << model_points.size() << " " << image_points.size() << endl;

        solvePnP(model_points, image_points, camera_matrix, dist_coeffs,
                 rotation_vector, translation_vector);

        const double*angles = rotation_vector.ptr<double>();

        cout << "mat rotation: " << endl << rotation_vector << endl;


        double angleX = rotation_vector.at<double>(0, 0) * (180.0 / 3.141592653589793238463);
        double angleY = rotation_vector.at<double>(0, 1) * (180.0 / 3.141592653589793238463);
        double angleZ = rotation_vector.at<double>(0, 2) * (180.0 / 3.141592653589793238463);




        //angles on x
        if ( abs(angles[0]) > min_rot  && abs(angles[0]) < max_rot &&
             abs(angles[1]) > min_rot && abs(angles[1]) < max_rot &&
             abs(angles[2]) > min_rot && abs(angles[2]) < max_rot


           ) {

           cout<< angleX<<" "<<angleY<<" "<<angleZ<<endl;
          //imshow("_pm.first", _pm.first);
          //waitKey(0);


          indexes.push_back(_pm.second);
          cout << "_pm.second " << _pm.second << endl;



          frames_by_square++;
        }
        if (frames_by_square >= _framesBySquare) {
          break;
        }

      }

      continue;
    }
    else {
      cout << "no camera matrix given " << endl;
    }



  }
  _frameIndex =  indexes;

}

void get_centroid(vector<Point2f>& _pointBuf, Point2f& centroid)
{
  centroid.x = 0;
  centroid.y = 0;


  for ( int i = 0; i < _pointBuf.size(); i++)
  {
    centroid += _pointBuf[i];
  }
  centroid /= (float)_pointBuf.size();
}

void getIndexesFromVideo(const string & _filename, vector<int>& indexes, Mat camera_matrix = Mat(),
                         Mat dist_coeffs = Mat(), int nFrames = 54) {

  VideoCapture cap(_filename);
  if (!cap.isOpened()) return ;

  int sh = 3;
  int sw = 3;

  float dh = 480 / sh;
  float dw = 640 / sw;
  map<Rect, vector< pair<Mat, int> > , classcomp> r2mat;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      r2mat[Rect(j * dw, i * dh, dw, dh)] = vector<pair<Mat, int>>();
    }
  }



  int global_index = 0;
  for (;;) {
    Mat frame;
    cap >> frame;


    waitKey(2);

    if (frame.empty()) break;
    Point2f centroid;
    vector<Point2f> _pointBuf;

    RingsDetector rd;
    Mat copy = frame.clone();
    bool found = rd.findPattern(copy, _pointBuf);

    Mat copy2;

    //bool found = findConcentricCirclesCenters(frame, Size(5, 4), _pointBuf);
    if (found) {


      copy2 = frame.clone();
      ProcManager ch;
      ch.drawControlPoints(copy2, _pointBuf);

      //imshow("video", copy2);


      get_centroid(_pointBuf, centroid);
      for (auto & _pair : r2mat) {
        if (_pair.first.contains(centroid)) {
          //_pair.second.push_back( centroid);
          _pair.second.push_back(make_pair(frame, global_index));

          cout << global_index << endl;

          break;
        }
      }

    }
    global_index++;



    /*
    if( global_index >200 )
    {
      break;
    }*/

  }

  for( auto _pair : r2mat )
  {
    cout<<_pair.first<<"   "<<_pair.second.size()<<endl;
  }



  int framesBySquare = nFrames / (sh * sw);

  get_FramesBySquare(r2mat, framesBySquare, indexes, camera_matrix , dist_coeffs, 0.0, 0.3);

}

}