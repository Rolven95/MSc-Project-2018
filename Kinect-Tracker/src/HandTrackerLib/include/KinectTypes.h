#pragma once

#include "Kinect.h"

namespace ar_sandbox
{
	// Define the FrameDimensions struct
	typedef struct FrameDimensions
	{
		int width;
		int height;

		FrameDimensions(int w, int h)
		{
			width = w;
			height = h;
		}
	} FrameDimensions;

#if defined(_WIN64)

	typedef IDepthFrame KDepthFrame;
	typedef IDepthFrameReader KDepthReader;
	typedef IDepthFrameSource KDepthSource;
	typedef DepthSpacePoint KDepthSpacePoint;

	typedef IColorFrame KRGBFrame;
	typedef IColorFrameReader KRGBFrameReader;
	typedef IColorFrameSource KRGBSource;

	typedef IMultiSourceFrame KMultiFrame;
	typedef IMultiSourceFrameReader KMultiFrameReader;

	typedef IFrameDescription KFrameDescription;
	typedef IKinectSensor KSensor;
	typedef ICoordinateMapper KCoordinateMapper;

	// Extend this interface as needs be (i.e as I need more of the SDK

#endif

#if defined(__APPLE__) && defined(__MACH__)
#include <TargetConditionals.h>
#if TARGET_OS_MAC == 1

	// TODO: Create types that interface with the OpenNI SDK

#endif
#endif
}