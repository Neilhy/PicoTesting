/*
 *  This code is released under the MIT License.
 *  Copyright (c) 2013 Nenad Markus
 */

#include <stdio.h>

// OpenCV 3.x required
// depending on your computer configuration (OpenCV install path), the following line might need modifications
#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui_c.h>
//#include "/usr/local/include/opencv2/highgui/highgui_c.h"

//
extern "C" {
#include "picornt.h"
}

/*
	a portable time function
*/

#ifdef __GNUC__
#include <time.h>
float getticks()
{
	struct timespec ts;

	if(clock_gettime(CLOCK_MONOTONIC, &ts) < 0)
		return -1.0f;

	return ts.tv_sec + 1e-9f*ts.tv_nsec;
}
#else
#include <windows.h>
float getticks()
{
	static double freq = -1.0;
	LARGE_INTEGER lint;

	if(freq < 0.0)
	{
		if(!QueryPerformanceFrequency(&lint))
			return -1.0f;

		freq = lint.QuadPart;
	}

	if(!QueryPerformanceCounter(&lint))
		return -1.0f;

	return (float)( lint.QuadPart/freq );
}
#endif

/*
	
*/

void* cascade = 0;

int minsize;
int maxsize;

float angle;

float scalefactor;
float stridefactor;

float qthreshold;

int usepyr;
int noclustering;
int verbose;

void process_image(IplImage* frame, int draw)
{
	int i, j;
	float t;

	uint8_t* pixels;
	int nrows, ncols, ldim;

#define MAXNDETECTIONS 2048
	int ndetections;
	float rcsq[4 * MAXNDETECTIONS];

	static IplImage* gray = 0;
	static IplImage* pyr[5] = { 0, 0, 0, 0, 0 };

	/*
	...
	*/

	//
	if (!pyr[0])
	{
		//
		gray = cvCreateImage(cvSize(frame->width, frame->height), frame->depth, 1);

		//
		pyr[0] = gray;
		pyr[1] = cvCreateImage(cvSize(frame->width / 2, frame->height / 2), frame->depth, 1);
		pyr[2] = cvCreateImage(cvSize(frame->width / 4, frame->height / 4), frame->depth, 1);
		pyr[3] = cvCreateImage(cvSize(frame->width / 8, frame->height / 8), frame->depth, 1);
		pyr[4] = cvCreateImage(cvSize(frame->width / 16, frame->height / 16), frame->depth, 1);
	}

	// get grayscale image
	if (frame->nChannels == 3) {
		cvCvtColor(frame, gray, CV_RGB2GRAY);
	}
	else {
		cvCopy(frame, gray, 0);
	}

	// perform detection with the pico library
	t = getticks();

	if (usepyr)
	{
		int nd;

		//
		pyr[0] = gray;

		pixels = (uint8_t*)pyr[0]->imageData;
		nrows = pyr[0]->height;
		ncols = pyr[0]->width;
		ldim = pyr[0]->widthStep;

		ndetections = find_objects(rcsq, MAXNDETECTIONS, cascade, angle, pixels, nrows, ncols, ldim, scalefactor, stridefactor, MAX(16, minsize), MIN(128, maxsize));

		for (i = 1; i<5; ++i)
		{
			cvResize(pyr[i - 1], pyr[i], CV_INTER_LINEAR);

			pixels = (uint8_t*)pyr[i]->imageData;
			nrows = pyr[i]->height;
			ncols = pyr[i]->width;
			ldim = pyr[i]->widthStep;

			nd = find_objects(&rcsq[4 * ndetections], MAXNDETECTIONS - ndetections, cascade, angle, pixels, nrows, ncols, ldim, scalefactor, stridefactor, MAX(64, minsize >> i), MIN(128, maxsize >> i));

			for (j = ndetections; j<ndetections + nd; ++j)
			{
				rcsq[4 * j + 0] = (1 << i)*rcsq[4 * j + 0];
				rcsq[4 * j + 1] = (1 << i)*rcsq[4 * j + 1];
				rcsq[4 * j + 2] = (1 << i)*rcsq[4 * j + 2];
			}

			ndetections = ndetections + nd;
		}
	}
	else
	{
		//
		pixels = (uint8_t*)gray->imageData;
		nrows = gray->height;
		ncols = gray->width;
		ldim = gray->widthStep;

		//
		ndetections = find_objects(rcsq, MAXNDETECTIONS, cascade, angle, pixels, nrows, ncols, ldim, scalefactor, stridefactor, minsize, MIN(nrows, ncols));
	}

	if(!noclustering)
		ndetections = cluster_detections(rcsq, ndetections);

	t = getticks() - t;

	printf("Pico time is :%f \n", t);
	// if the flag is set, draw each detection
	if(draw)
		for(i=0; i<ndetections; ++i)
			if(rcsq[4*i+3]>=qthreshold) // check the confidence threshold
				cvCircle(frame, cvPoint(rcsq[4*i+1], rcsq[4*i+0]), rcsq[4*i+2]/2, CV_RGB(255, 0, 0), 2, 8, 0); // we draw circles here since height-to-width ratio of the detected face regions is 1.0f

}

void process_webcam_frames()
{
	CvCapture* capture;

	IplImage* frame;
	IplImage* framecopy;

	int stop;

	const char* windowname = "--------------------";

	//
	capture = cvCaptureFromCAM(0);

	if(!capture)
	{
		printf("# cannot initialize video capture ...\n");
		return;
	}

	// the main loop
	framecopy = 0;
	stop = 0;

	while(!stop)
	{
		// wait 5 miliseconds
		int key = cvWaitKey(5);

		// get the frame from webcam
		if(!cvGrabFrame(capture))
		{
			stop = 1;
			frame = 0;
		}
		else
			frame = cvRetrieveFrame(capture, 1);

		// we terminate the loop if the user has pressed 'q'
		if(!frame || key=='q')
			stop = 1;
		else
		{
			// we mustn't tamper with internal OpenCV buffers
			if(!framecopy)
				framecopy = cvCreateImage(cvSize(frame->width, frame->height), frame->depth, frame->nChannels);
			cvCopy(frame, framecopy, 0);

			// webcam outputs mirrored frames (at least on my machines)
			// you can safely comment out this line if you find it unnecessary
			cvFlip(framecopy, framecopy, 1);

			// ...
			//process_image(framecopy, 1);

			// ...
			cvShowImage(windowname, framecopy);
		}
	}

	// cleanup
	cvReleaseImage(&framecopy);
	cvReleaseCapture(&capture);
	cvDestroyWindow(windowname);
}

int main(int argc, char* argv[])
{
	//
	int arg;
	char input[1024], output[1024];
		int size;
		FILE* file;

		//
		file = fopen("facefinder", "rb");

		if(!file)
		{
			printf("# cannot read cascade from '%s'\n", argv[1]);
			return 1;
		}

		//
		fseek(file, 0L, SEEK_END);
		size = ftell(file);
		fseek(file, 0L, SEEK_SET);

		//
		cascade = malloc(size);

		if(!cascade || size!=fread(cascade, 1, size, file))
			return 1;

		//
		fclose(file);
	// set default parameters
	minsize = 10;
	maxsize = 1024;

	angle = 0.0f;

	scalefactor = 1.1f;
	stridefactor = 0.1f;

	qthreshold = 5.0f;

	usepyr = 1;
	noclustering = 0;
	verbose = 0;

	//
	input[0] = 0;
	output[0] = 0;

	// parse command line arguments
	arg = 2;
	cv::Mat imgMat = cv::imread("E:\\Chrome\\face1.jpg");
	cv::Mat * imgMM = &imgMat;

		IplImage img;
		IplImage* desc;
		CvSize sz;
		double scale = 0.5;
		//
		//img = cvLoadImage("E:\\Chrome\\face1.jpg", CV_LOAD_IMAGE_COLOR);
		img =IplImage(*imgMM);

		printf("image before resize width:%d  height:%d  depth:%d\n", img.width, img.height,img.depth);
		sz.width = img.width*scale;
        sz.height = img.height*scale;
		desc = cvCreateImage(sz, img.depth, img.nChannels);
		cvResize(&img, desc, CV_INTER_CUBIC);
		printf("image after resize width:%d  height:%d\n", desc->width, desc->height);
		//cv::Mat imageMat = cv::cvarrToMat(desc, true);
		process_image(desc, 1);
		cv::Mat descMat= cv::cvarrToMat(desc, true);
		cv::cvtColor(descMat, descMat, cv::COLOR_RGB2RGBA);
		printf("descMat dims:%d and rows %d and cols %d", descMat.dims,descMat.rows,descMat.cols);
		cv::imwrite("E:\\Chrome\\face1Out.jpg", descMat);
		cv::imshow("Mat test", descMat);
		//cvSaveImage("E:\\Chrome\\face1Out.jpg", desc, 0);
		//cvShowImage("E:\\Chrome\\face1Out.jpg", desc);
		cvWaitKey(0);
		imgMat.release();
		descMat.release();
		cvReleaseImage(&desc);
	
	return 0;
}
