// HandTrackerLib.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"

// Smart pointer stuff from boost
#include <boost\smart_ptr\shared_ptr.hpp>

// Definitions of constants
#define RESIZE_WIDTH 512
#define RESIZE_HEIGHT 512

#define DO_IF_INITED(x) \
if (isInited) { \
	x \
}

// Some aliases
using SensorManager = boost::shared_ptr<ar_sandbox::KinectManager>;
using Resizer = boost::shared_ptr<ar_sandbox::DepthFrameResizer>;
using HandTracker = boost::shared_ptr<ar_sandbox::HandTracker>;

// Instantiation
static int RES = 0;
static bool isInited = false;
static SensorManager sensorManager;
static Resizer resizer;
static HandTracker handTracker;

// Implementaton
void initEnv()
{
	if (!isInited)
	{
		// Initiate the processors
		sensorManager = SensorManager(new ar_sandbox::KinectManager);
		resizer = Resizer(new ar_sandbox::DepthFrameResizer);

		// Initiate the processor and then the hand tracker
		resizer->setResizeParameters(RESIZE_WIDTH, RESIZE_HEIGHT);
		cv::Size processParams = resizer->getSizeParameters();
		handTracker = HandTracker(new ar_sandbox::HandTracker(processParams));

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
		RES = 1;

		isInited = false;
	}
}

void updateSensor()
{
	//if (isInited)
	//{
	//	// Loop to ignore null frames
	//	do
	//	{
	//		sensorManager->readMultiFrame();
	//	} while (sensorManager->getDepthDimensions().width <= 0);
	//}

	DO_IF_INITED(
		do
		{
			sensorManager->readMultiFrame();
		} while (sensorManager->getDepthDimensions().width <= 0);
	)
}

void updateProcessor()
{

}

void updateHandTracker()
{

}

void setResizeProcessorParams(int width, int height)
{

}

unsigned short * getDepthFrame()
{
	return nullptr;
}
