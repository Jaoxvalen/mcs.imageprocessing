#ifndef FILTERS_H
#define FILTERS_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <random>
#include <QImage>
#include "utils.h"
#include "FFT.h"

using namespace cv;
using namespace std;

class Image
{

    bool _have_image = false;
public:
    Mat _mat;
    Image()
    {

    }
    Image(string path)
    {
        _mat = imread(path.c_str(), IMREAD_COLOR);
        _have_image = true;
        if (_mat.empty())
        {
                cout << "Could not open or find the image" << endl;
                _have_image = false;
        }
    }
    void Save(string namefile)
    {
        if(_have_image)
        imwrite( namefile.c_str(), _mat );
    }
    bool have_image()
    {
        return _have_image;
    }
    QImage to_QImage()
    {
        Mat mat;
        cvtColor(_mat,mat, CV_BGR2RGB);
        return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888).copy();
    }
    //fft
    void fft()
    {
        Mat I;
        cvtColor(_mat, I, CV_BGR2GRAY);
        Mat padded;                            //expand input image to optimal size
        int m = getOptimalDFTSize( I.rows );
        int n = getOptimalDFTSize( I.cols ); // on the border add zero values
        copyMakeBorder(I, padded, 0, m - I.rows, 0, n - I.cols, BORDER_CONSTANT, Scalar::all(0));
        Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
        Mat complexI;
        merge(planes, 2, complexI);
        dft(complexI, complexI);
        // compute the magnitude and switch to logarithmic scale
        // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
        split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
        magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
        Mat magI = planes[0];

        magI += Scalar::all(1);                    // switch to logarithmic scale
        log(magI, magI);

        // crop the spectrum, if it has an odd number of rows or columns
        magI = magI(Rect(0, 0, magI.cols & -2, magI.rows & -2));

        // rearrange the quadrants of Fourier image  so that the origin is at the image center
        int cx = magI.cols/2;
        int cy = magI.rows/2;

        Mat q0(magI, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
        Mat q1(magI, Rect(cx, 0, cx, cy));  // Top-Right
        Mat q2(magI, Rect(0, cy, cx, cy));  // Bottom-Left
        Mat q3(magI, Rect(cx, cy, cx, cy)); // Bottom-Right

        Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
        q0.copyTo(tmp);
        q3.copyTo(q0);
        tmp.copyTo(q3);

        q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
        q2.copyTo(q1);
        tmp.copyTo(q2);

        normalize(magI, magI, 0, 1, CV_MINMAX); // Transform the matrix with float values into a
                                                // viewable image form (float between values 0 and 1).
        imshow("magnitude", magI);
    }
    //utils
    int** calcHistogram(const Mat &pImage) {
        int** rHistogram;
        rHistogram = new int*[3];
        rHistogram[0] = new int[255]; //B
        rHistogram[1] = new int[255]; //G
        rHistogram[2] = new int[255]; //R

        for (int i = 0; i < 255; i++) {
            rHistogram[0][i] = 0;
            rHistogram[1][i] = 0;
            rHistogram[2][i] = 0;
        }

        for (int i = 0; i < pImage.rows; i++) {
            for (int j = 0; j < pImage.cols; j++) {
                Vec3b color = pImage.at<Vec3b>(i, j);
                int iB = (int) color[0];
                int iG = (int) color[1];
                int iR = (int) color[2];

                rHistogram[0][iB] = rHistogram[0][iB] + 1;
                rHistogram[1][iG] = rHistogram[1][iG] + 1;
                rHistogram[2][iR] = rHistogram[2][iR] + 1;
            }
        }
        return rHistogram;
    }
    int** calcFuncEq(int** pHistogram, int np) {
        int** functionEq;
        functionEq = new int*[3];
        functionEq[0] = new int[255]; //B
        functionEq[1] = new int[255]; //G
        functionEq[2] = new int[255]; //R

        int acumuladoB = pHistogram[0][0];
        int acumuladoG = pHistogram[1][0];
        int acumuladoR = pHistogram[2][0];

        functionEq[0][0] = 0;
        functionEq[1][0] = 0;
        functionEq[2][0] = 0;
        for (int i = 1; i < 254; i++) {
            functionEq[0][i] = (float) acumuladoB * 255 / (float) np;
            functionEq[1][i] = (float) acumuladoG * 255 / (float) np;
            functionEq[2][i] = (float) acumuladoR * 255 / (float) np;
            acumuladoB = acumuladoB + pHistogram[0][i];
            acumuladoG = acumuladoG + pHistogram[1][i];
            acumuladoR = acumuladoR + pHistogram[2][i];
        }
        functionEq[0][255] = 255;
        functionEq[1][255] = 255;
        functionEq[2][255] = 255;

        return functionEq;
    }
    //filters
    void rotateImg(float angle)
    {
        angle *= -1;
        Point2f src_center(_mat.cols/2.0F, _mat.rows/2.0F);
        Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
        warpAffine(_mat, _mat, rot_mat, _mat.size());
    }

    void resizeImag(int factor)
    {
        float fFactor = (float)factor/100.0;
        cv::Size s = _mat.size();
        int width = (float)s.width*fFactor;
        int height = (float)s.height*fFactor;
        if(width>1 && height>1)
        {
            Size size(width,height);
            resize(_mat,_mat,size);//resize image
        }


    }

    void equalize() {
        //calculamos la funcion equalizadora
        int** func = calcFuncEq(calcHistogram(_mat), _mat.rows * _mat.cols);

        for (int i = 0; i < _mat.rows; i++) {
            for (int j = 0; j < _mat.cols; j++) {
                Vec3b color = _mat.at<Vec3b>(i, j);
                int iB = (int) color[0];
                int iG = (int) color[1];
                int iR = (int) color[2];
                Vec3b colorEq;
                colorEq[0] = (unsigned char) (func[0][iB]);
                colorEq[1] = (unsigned char) (func[1][iG]);
                colorEq[2] = (unsigned char) (func[2][iR]);
                _mat.at<Vec3b>(i, j) = colorEq;
            }
        }
        delete[] func;
    }
    void filter_mediana(int sizeConv) {
        Mat copy = _mat.clone();

        vector<int> BNeigh;
        vector<int> GNeigh;
        vector<int> RNeigh;

        for (int i = 0; i < _mat.rows; i++) {
            for (int j = 0; j < _mat.cols; j++) {
                Vec3b colornew;
                for (int k = 0; k < sizeConv; k++) {
                    for (int l = 0; l < sizeConv; l++) {
                        int indX = i - ((float) k / (float) 2);
                        int indY = j - ((float) l / (float) 2);
                        if (indX >= 0 && indX < _mat.rows
                            && indY >= 0 && indY <= _mat.cols) {
                            Vec3b color = _mat.at<Vec3b>(indX, indY);
                            BNeigh.push_back((int) color[0]);
                            GNeigh.push_back((int) color[1]);
                            RNeigh.push_back((int) color[2]);

                        }

                    }
                }

                sort (BNeigh.begin(), BNeigh.end(), less<int>());
                sort (GNeigh.begin(), GNeigh.end(), less<int>());
                sort (RNeigh.begin(), RNeigh.end(), less<int>());

                int m = BNeigh.size()/2;

                int Bs = BNeigh[m];
                int Gs = GNeigh[m];
                int Rs = RNeigh[m];

                colornew[0]=(unsigned char)Bs;
                colornew[1]=(unsigned char)Gs;
                colornew[2]=(unsigned char)Rs;
                copy.at<Vec3b>(i, j) = colornew;

                BNeigh.clear();
                GNeigh.clear();
                RNeigh.clear();

            }
        }
        _mat = copy;
    }
    void filter_noise(float percent)
    {

        //Utils::initialize_randomness(-1);
        vector<point> pixelsNoise;
        for (int i = 0; i < _mat.rows; i++) {
            for (int j = 0; j < _mat.cols; j++) {
                point p;
                p.x = i;
                p.y = j;
                pixelsNoise.push_back(p);
            }
        }


        int nNoise = percent * _mat.cols * _mat.rows;

        random_shuffle(pixelsNoise.begin(), pixelsNoise.end());


        for (int i = 0; i < nNoise; i++) {
            int x=pixelsNoise[i].x;
            int y=pixelsNoise[i].y;

            Vec3b color = _mat.at<Vec3b>(x, y);
            int iB = (int) color[0];
            int iG = (int) color[1];
            int iR = (int) color[2];
            Vec3b colorEq;
            int val = 255;
            if (Utils::randint(0, 1) == 0) {
                val = 0;
            }
            colorEq[0] = (unsigned char) val;
            colorEq[1] = (unsigned char) val;
            colorEq[2] = (unsigned char) val;
            _mat.at<Vec3b>(x, y) = colorEq;
        }
    }
    void filter_media(int sizeConv) {
        Mat copy = _mat.clone();
        for (int i = 0; i < _mat.rows; i++) {
            for (int j = 0; j < _mat.cols; j++) {
                int BAcum = 0;
                int GAcum = 0;
                int RAcum = 0;
                int c = 0;
                Vec3b colornew;
                for (int k = 0; k < sizeConv; k++) {
                    for (int l = 0; l < sizeConv; l++) {
                        int indX = i - ((float) k / (float) 2);
                        int indY = j - ((float) l / (float) 2);
                        if (indX >= 0 && indX < _mat.rows
                            && indY >= 0 && indY <= _mat.cols) {
                            Vec3b color = _mat.at<Vec3b>(indX, indY);
                            BAcum += (int) color[0];
                            GAcum += (int) color[1];
                            RAcum += (int) color[2];
                            c++;
                        }

                    }
                }
                BAcum /= c;
                GAcum /= c;
                RAcum /= c;
                colornew[0]=(unsigned char)BAcum;
                colornew[1]=(unsigned char)GAcum;
                colornew[2]=(unsigned char)RAcum;
                copy.at<Vec3b>(i, j) = colornew;
            }
        }
        _mat = copy;
    }
};
#endif // FILTERS_H
