#ifndef FFT_H
#define FFT_H


#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <math.h>
#include <iostream>

using namespace cv;
using namespace std;

struct COMPLEX
 {
public:
    double real, imag;
    COMPLEX(){}
    COMPLEX(double x, double y)
    {
        real = x;
        imag = y;
    }
    float Magnitude()
    {
        return ((float)sqrt(real * real + imag * imag));
    }
    float Phase()
    {
         return ((float)atan(imag / real));
    }
};


class FFT
{

public:
    FFT(Mat image) {
        //convertimos la imagen a greyScale
        Mat greyMat;
        cvtColor(image, greyMat, CV_BGR2GRAY);

        //namedWindow("original", WINDOW_AUTOSIZE);
        //imshow("original", greyMat);

        cv::Size s = image.size();
        int Width = s.width;
        int Height = s.height;

        //Initializing Fourier Transform Array
        int i,j;
        COMPLEX** Fourier = new COMPLEX* [Width];
        //COMPLEX** Output = new COMPLEX* [Width];
        for(int i=0; i<Width; i++)
        {
            Fourier[i] = new COMPLEX[Height];
            //Output[i] = new COMPLEX[Height];
        }
        //Copy Image Data to the Complex Array
        for (i=0;i<Width;i++)
           for (j = 0; j < Height; j++)
           {
               Vec3b color = image.at<Vec3b>(j, i);
               int pixelValue = (int)color[0];
               Fourier[i][j].real =(double) pixelValue;
               Fourier[i][j].imag = 0;
           }
        //Calling Forward Fourier Transform
        FFT2D( Fourier, Width, Height, 1);

        Mat FourierPlot, PhasePlot;

        FFTPlot(Width, Height, Fourier, greyMat, FourierPlot, PhasePlot);

        namedWindow("original", WINDOW_AUTOSIZE);
        imshow("original", FourierPlot);

        //delete[] Fourier;
        //delete[] Output;
    }
    Mat Displayimage(const Mat& grey, int** image)
    {
        cout<<image[0][0]<<endl;
        int i,j;
        //Mat output =grey.clone();

        Mat output = Mat(grey.cols ,grey.rows, CV_8UC3);

        cv::Size s = grey.size();
        int Width = s.width;
        int Height = s.height;

        for (i=0;i<Width;i++)
           for (j = 0; j < Height; j++)
           {
               Vec3b colorEq;
               colorEq[0] = (unsigned char) image[i][j];
               colorEq[1] = (unsigned char) image[i][j];
               colorEq[2] = (unsigned char) image[i][j];
               output.at<Vec3b>(i, j) = colorEq;
           }
        return output;

    }

    void FFTPlot(int nx, int ny, COMPLEX** Output, Mat& grey, Mat& FourierPlot, Mat& PhasePlot)
    {
        int i, j;
        float max;

        cv::Size s = grey.size();
        int Width = s.width;
        int Height = s.height;

        float** FFTLog = new float* [nx];
        float** FFTPhaseLog = new float* [nx];
        float** FourierMagnitude = new float* [nx];
        float** FourierPhase = new float* [nx];

        int** FFTNormalized = new int* [nx];
        int** FFTPhaseNormalized = new int* [nx];

        for(int i=0; i<nx; i++)
        {
            FFTLog[i] = new float[ny];
            FFTPhaseLog[i] = new float[ny];
            FourierMagnitude[i] = new float[ny];
            FourierPhase[i] = new float[ny];

            FFTNormalized[i] = new int[ny];
            FFTPhaseNormalized[i] = new int[ny];
        }
        for(i=0;i<Width;i++)
        {
            for (j = 0; j < Height; j++)
            {
                FourierMagnitude[i][j] = Output[i][j].Magnitude();
                FourierPhase[i][j] = Output[i][j].Phase();
                FFTLog[i][j] = (float)log(1 + FourierMagnitude[i][j]);
                FFTPhaseLog[i][j] = (float)log(1 + abs(FourierPhase[i][j]));
            }
        }
        //Generating Magnitude Bitmap
        max = FFTLog[0][0];
        for(i=0;i<Width;i++)
            for (j = 0; j < Height; j++)
            {
                if (FFTLog[i][j] > max)
                    max = FFTLog[i][j];
            }

        for(i=0;i<Width;i++)
            for (j = 0; j < Height; j++)
            {
                FFTLog[i][j] = FFTLog[i][j] / max;
            }

        for(i=0;i<Width;i++)
            for (j = 0; j < Height; j++)
            {
                FFTNormalized [i][j]=(int)(1000*FFTLog[i][j]);
            }

        //Transferring Image to Fourier Plot
        FourierPlot = Displayimage(grey, FFTNormalized);//todo

        //generating phase Bitmap
        max = FFTPhaseLog[0][0];
        for (i = 0; i < Width; i++)
            for (j = 0; j < Height; j++)
            {
                if (FFTPhaseLog[i][j] > max)
                    max = FFTPhaseLog[i][j];
            }
        for (i = 0; i < Width; i++)
            for (j = 0; j < Height; j++)
            {
                FFTPhaseLog[i][j] = FFTPhaseLog[i][j] / max;
            }
        for (i = 0; i < Width; i++)
            for (j = 0; j < Height; j++)
            {
                FFTPhaseNormalized[i][j] = (int)(2000 * FFTLog[i][j]);
            }

        //Transferring Image to Fourier Plot
        PhasePlot = Displayimage(grey, FFTPhaseNormalized);

    }


    void FFT2D ( COMPLEX** c, int nx, int ny, int dir )
    {
        int i,j;
        int m;//Power of 2 for current number of points
        double* real;
        double* imag;
        COMPLEX ** output;
        output = c;

        // Transform the Rows
        real = new double[nx];
        imag = new double[nx];

        for (j=0;j<ny;j++)
        {
            for (i=0;i<nx;i++)
            {
                real[i] = c[i][j].real;
                imag[i] = c[i][j].imag;
            }
            // Calling 1D FFT Function for Rows
            m = (int)log2((double)nx);//Finding power of 2 for current number of points e.g. for nx=512 m=9
            FFT1D(dir , m , real , imag);

            for (i=0;i<nx;i++)
            {
                output[i][j].real = real[i];
                output[i][j].imag = imag[i];
            }
        }

        // Transform the columns
        real = new double[ny];
        imag = new double[ny];

        for (i=0;i<nx;i++)
        {
            for (j=0;j<ny;j++)
            {
                real[j] = output[i][j].real;
                imag[j] = output[i][j].imag;
            }
            // Calling 1D FFT Function for Columns
            m = (int)log2((double)ny);//Finding power of 2 for current number of points e.g. for nx=512 m=9
            FFT1D(dir,m,real,imag);
            for (j=0;j<ny;j++)
            {
                output[i][j].real = real[j];
                output[i][j].imag = imag[j];
            }
        }
    }

    void FFT1D(int dir, int m, double* x, double* y)
    {
        long nn, i, i1, j, k, i2, l, l1, l2;
        double c1, c2, tx, ty, t1, t2, u1, u2, z;
        // Calculate the number of points
        nn = 1;
        for (i = 0; i < m; i++)
        {
            nn *= 2;
        }
        // Do the bit reversal
        i2 = nn >> 1;
        j = 0;
        for (i = 0; i < nn - 1; i++)
        {
            if (i < j)
            {
                tx = x[i];
                ty = y[i];
                x[i] = x[j];
                y[i] = y[j];
                x[j] = tx;
                y[j] = ty;
            }
            k = i2;
            while (k <= j)
            {
                j -= k;
                k >>= 1;
            }
            j += k;
        }
        // Compute the FFT
        c1 = -1.0;
        c2 = 0.0;
        l2 = 1;
        for (l = 0; l < m; l++)
        {
            l1 = l2;
            l2 <<= 1;
            u1 = 1.0;
            u2 = 0.0;
            for (j = 0; j < l1; j++)
            {
                for (i = j; i < nn; i += l2)
                {
                    i1 = i + l1;
                    t1 = u1 * x[i1] - u2 * y[i1];
                    t2 = u1 * y[i1] + u2 * x[i1];
                    x[i1] = x[i] - t1;
                    y[i1] = y[i] - t2;
                    x[i] += t1;
                    y[i] += t2;
                }
                z = u1 * c1 - u2 * c2;
                u2 = u1 * c2 + u2 * c1;
                u1 = z;
            }
            c2 = sqrt((1.0 - c1) / 2.0);
            if (dir == 1)
                c2 = -c2;
            c1 = sqrt((1.0 + c1) / 2.0);
        }
        // Scaling for forward transform
        if (dir == 1)
        {
            for (i = 0; i < nn; i++)
            {
                x[i] /= (double)nn;
                y[i] /= (double)nn;

            }
        }
    }



};

#endif // FFT_H
