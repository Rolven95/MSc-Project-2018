#include "stdafx.h"
#include "Unity.h"

#include <iostream>

#define RESIZE_WIDTH 512
#define RESIZE_HEIGHT 512

using SensorManager = boost::shared_ptr<ar_sandbox::KinectManager>;
using Resizer = boost::shared_ptr<ar_sandbox::DepthFrameResizer>;
using HandTracker = boost::shared_ptr<ar_sandbox::HandTracker>;

static int RES = 0;
static bool isInited = false;
static SensorManager sensorManager;
static Resizer resizer;
static HandTracker handTracker;

//static int RES = 0;
//static bool isInited = false;
//static boost::shared_ptr<ar_sandbox::KinectManager> sensorManager(new ar_sandbox::KinectManager);
//static boost::shared_ptr<ar_sandbox::DepthFrameResizer> resizer(new ar_sandbox::DepthFrameResizer);
//static boost::shared_ptr<ar_sandbox::HandTracker> handTracker(new ar_sandbox::HandTracker(cv::Size(RESIZE_WIDTH, RESIZE_HEIGHT)));

// Hanck job.
// Don't ever do this, but I'm also going to some
// an OpenCV matrices lying around as the depth frame buffer

static boost::shared_ptr<unsigned short[]> depthFrameBuffer;
static boost::shared_ptr<BYTE[]> contourFrameBuffer;

// Environment management
void initEnv()
{
	if (!isInited)
	{
		sensorManager = boost::make_shared<ar_sandbox::KinectManager>();
		resizer = boost::make_shared<ar_sandbox::DepthFrameResizer>();

		// Hand tracker needs special attention
		resizer->setResizeParameters(RESIZE_WIDTH, RESIZE_HEIGHT);
		cv::Size processParams = resizer->getSizeParameters();
		handTracker = boost::make_shared<ar_sandbox::HandTracker>(processParams);

		// Create the static buffers
		depthFrameBuffer = boost::make_shared<unsigned short[]>(processParams.width * processParams.height);
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
		resizer.reset();
		sensorManager.reset();

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
		resizer->processFrame(depthFrame);
	}
}

void updateHandTracker()
{
	if (isInited)
	{
		// Ask the processor for its latest frame and pass it through the handtracker
		cv::Mat depthFrame = resizer->getFrame();
		handTracker->processFrame(depthFrame);
	}
}

// Modifiable functions
void setResizeProcessorParams(int width, int height)
{
	if (isInited)
	{
		resizer->setResizeParameters(width, height);
	}
}

// Getters
bool getDepthFrame(unsigned short **interOpPtr, int *frameLength)
{
	if (isInited)
	{
		cv::Mat depthMat = cv::Mat(512, 512, CV_16U, depthFrameBuffer.get());
		resizer->copyFrameBuffer(depthMat);

		*interOpPtr = depthFrameBuffer.get();
		*frameLength = depthMat.rows * depthMat.cols;

		return true;
	}

	return false; // Return false if we aren't even inited
}

bool getContourFrame(BYTE **interOpPtr, int *frameLen)
{
	if (isInited)
	{
		cv::Mat contourMat = cv::Mat(512, 512, CV_8UC3, contourFrameBuffer.get());
		handTracker->copyFrameBuffer(contourMat);

		*interOpPtr = contourFrameBuffer.get();
		*frameLen = contourMat.rows * contourMat.cols * 3;
		return true;
	}

	return false;
}

