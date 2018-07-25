#include "stdafx.h"
#include "Unity.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <omp.h>

#define RESIZE_WIDTH 1920
#define RESIZE_HEIGHT 1080

using SensorManager = boost::shared_ptr<ar_sandbox::KinectManager>;
using DepthResizer = boost::shared_ptr<ar_sandbox::DepthFrameResizer>;
using ColorResizer = boost::shared_ptr<ar_sandbox::ColorFrameResizer>;
using HandTracker = boost::shared_ptr<ar_sandbox::HandTracker>;
using QRFrameProcessor = boost::shared_ptr<ar_sandbox::QRFrameProcessor>; // Alias for the shared pointer storage


static int RES = 0;
static bool isInited = false;
static SensorManager sensorManager;
static DepthResizer depthResizer;
static ColorResizer colorResizer;
static HandTracker handTracker;
static QRFrameProcessor qrFrameProcessor;


//static int RES = 0;
//static bool isInited = false;
//static boost::shared_ptr<ar_sandbox::KinectManager> sensorManager(new ar_sandbox::KinectManager);
//static boost::shared_ptr<ar_sandbox::DepthFrameResizer> resizer(new ar_sandbox::DepthFrameResizer);
//static boost::shared_ptr<ar_sandbox::HandTracker> handTracker(new ar_sandbox::HandTracker(cv::Size(RESIZE_HEIGHT, RESIZE_WIDTH)));

// Hanck job.
// Don't ever do this, but I'm also going to some
// an OpenCV matrices lying around as the depth frame buffer

static boost::shared_ptr<BYTE[]> colorFrameBuffer;
static boost::shared_ptr<unsigned short[]> depthFrameBuffer;
static boost::shared_ptr<unsigned short[]> depthFrameBufferClone;
static boost::shared_ptr<BYTE[]> contourFrameBuffer;
//static boost::shared_ptr<ar_sandbox::QRFrameProcessor> qrFrameProcessor;

// Environment management
void initEnv()
{
	if (!isInited)
	{
		sensorManager = boost::make_shared<ar_sandbox::KinectManager>();
		depthResizer = boost::make_shared<ar_sandbox::DepthFrameResizer>();
		colorResizer = boost::make_shared<ar_sandbox::ColorFrameResizer>();

		// Something like:
		qrFrameProcessor = boost::make_shared<ar_sandbox::QRFrameProcessor>();

		// Hand tracker needs special attention
		depthResizer->setResizeParameters(RESIZE_HEIGHT, RESIZE_WIDTH);
		colorResizer->setResizeParameters(RESIZE_HEIGHT, RESIZE_WIDTH);
		cv::Size processParams = depthResizer->getSizeParameters(); // Both have the same params, so the depth one is representative of both
		handTracker = boost::make_shared<ar_sandbox::HandTracker>(processParams);

		// Create the static buffers
		colorFrameBuffer = boost::make_shared<BYTE[]>(processParams.width * processParams.height * 4); // RGBA data
		depthFrameBuffer = boost::make_shared<unsigned short[]>(processParams.width * processParams.height);
		depthFrameBufferClone = boost::make_shared<unsigned short[]>(processParams.width * processParams.height);
		contourFrameBuffer = boost::make_shared<BYTE[]>(processParams.width * processParams.height * 3); // RGB data

		// Start the sensor
		sensorManager->initSensor();

		// Let the show begin
		isInited = true;
	}
}

void destroyEnv()
{
	if (isInited)
	{
		handTracker.reset();
		depthResizer.reset();
		colorResizer.reset();
		sensorManager.reset();
		qrFrameProcessor.reset();


		isInited = false;
	}
}

// Update functions
void updateSensor()
{
	if (isInited)
	{
		// Loop until we receive an actual frame.
		// This is probably a REALLY BAD IDEA!!!
		// If I get hiccups, I'll look here
		do
		{
			sensorManager->readMultiFrame();
		} while (sensorManager->getDepthDimensions().width <= 0);
	}
}

void updateProcessor()
{
	if (isInited)
	{
		// Process the current frame from the sensor manager
		cv::Mat depthFrame = sensorManager->getDepthMat();
		cv::Mat colorFrame = sensorManager->getColorMat();
		depthResizer->processFrame(depthFrame);
		colorResizer->processFrame(colorFrame);
		qrFrameProcessor->processFrame(colorFrame);
	}
}

void updateHandTracker()
{
	if (isInited)
	{
		// Ask the processor for its latest frame and pass it through the handtracker
		// cv::Mat depthFrame = depthResizer->getFrame();
		cv::Mat depthMatClone = cv::Mat(RESIZE_HEIGHT, RESIZE_WIDTH, CV_16U, depthFrameBufferClone.get());
		depthResizer->copyFrameBuffer(depthMatClone);
		handTracker->processFrame(depthMatClone);
	}
}

// Modifiable functions
void setResizeProcessorParams(int width, int height)
{
	if (isInited)
	{
		depthResizer->setResizeParameters(height, width);
		colorResizer->setResizeParameters(height, width);
	}
}

// Getters
bool getDepthFrame(unsigned short **interOpPtr, int *frameLength)
{
	if (isInited)
	{
		cv::Mat depthMat = cv::Mat(RESIZE_HEIGHT, RESIZE_WIDTH, CV_16U, depthFrameBuffer.get());
		depthResizer->copyFrameBuffer(depthMat);

		*interOpPtr = depthFrameBuffer.get();
		*frameLength = depthMat.rows * depthMat.cols;

		return true;
	}

	return false; // Return false if we aren't even inited
}

bool getColorFrame(BYTE **interOpPtr, int *frameLength)
{
	if (isInited)
	{
		cv::Mat colorMat = cv::Mat(RESIZE_HEIGHT, RESIZE_WIDTH, CV_8UC4, colorFrameBuffer.get());
		colorResizer->copyFrameBuffer(colorMat);

		*interOpPtr = colorFrameBuffer.get();
		*frameLength = colorMat.rows * colorMat.cols * 4;

		return true;
	}

	return false; // Return false if we aren't even inited
}

bool getContourFrame(BYTE **interOpPtr, int *frameLen)
{
	if (isInited)
	{
		cv::Mat contourMat = cv::Mat(RESIZE_HEIGHT, RESIZE_WIDTH, CV_8UC3, contourFrameBuffer.get());
		handTracker->copyFrameBuffer(contourMat);

		*interOpPtr = contourFrameBuffer.get();
		*frameLen = contourMat.rows * contourMat.cols * 3;
		return true;
	}

	return false;
}

bool getQRTraceFrame(BYTE **interOpPtr, int *frameLength)
{
	if (isInited)
	{
		
		cv::Mat TraceMat = cv::Mat(RESIZE_HEIGHT, RESIZE_WIDTH, CV_8UC4, colorFrameBuffer.get());
		qrFrameProcessor->copyFrameBuffer(TraceMat);
		TraceMat = qrFrameProcessor->Traces;
		if (TraceMat.rows != 0)
		{
			imshow("trace", TraceMat);
		}		
		*interOpPtr = colorFrameBuffer.get();
		*frameLength = TraceMat.rows * TraceMat.cols * 4;
		return true;
	}

	return false; // Return false if we aren't even inited
}

bool getQRResult(int (*qrArray)[30][6])
{
	for (int i = 0; i < 30; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			(*qrArray)[i][j] = qrFrameProcessor->allinfo[i][j];		
		}
	}
	return true;
}

bool testFun(int (*a)[10])
{
	for (int i = 0; i < 10; i++)
	{
		(*a)[i] = (*a)[i] + 1;
	}
	
	return true;
}
