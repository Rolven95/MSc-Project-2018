#pragma once

// This is the class for interacting with the Kinect sensor
// either through the Kinect for Windows SDK or the OpenNI SDK
//
// @author Daniel J. Finnegan

// Declare some types that will be defined 
// depending on the current platform being compiled

#if defined(_WIN64)
// I'm only supporting 64-Bit Windows
#include <Kinect.h>
#include <Ole2.h>
#include <Windows.h>
#define USING_WINDOWS
#endif

#if defined(__APPLE__) && defined(__MACH__)
#include <TargetConditionals.h>
#if TARGET_OS_MAC == 1
// At this point, we are on a mac,
// make definitions, include Mac specific headers etc.
#define USING_OSX
#endif
#endif

#include "KinectTypes.h" // Cross platform redefinitions of the Kinect interface
//#include "HandTracker.h"

// For boost magic
#include <boost\smart_ptr\make_shared.hpp>
#include <boost\smart_ptr\shared_ptr.hpp>

// For OpenCV magic
#include <opencv2\core\core.hpp>

namespace ar_sandbox
{
	// This is the class that wraps
	// over the sensor, providing a cross
	// platform API
	class KinectManager
	{
	public:
		KinectManager();
		~KinectManager();

		bool initSensor();
		bool readColorFrame(KMultiFrame *multiFrame);
		bool readDepthFrame(KMultiFrame *multiFrame, KDepthFrame **depthFrame);
		bool readMultiFrame();

		// Function for querying the current frame from the Kinect camera
		cv::Mat& getColorMat() { return rgbMat; }
		cv::Mat& getDepthMat() { return depthMat; }

		// Meta data
		FrameDimensions getDepthDimensions() { return FrameDimensions(depthFrameWidth, depthFrameHeight); }
		FrameDimensions getColorDimensions() { return FrameDimensions(colorFrameWidth, colorFrameHeight); }

	private:

		// Some functions for internal use
		void initColorData(KMultiFrame *frame);
		void initDepthData(KMultiFrame *frame);

		// Standard kinect variables
		KSensor *sensor;
		KMultiFrameReader *reader;
		KCoordinateMapper *mapper;

		// Color frame variables
		int colorFrameLengthInPixels;
		int colorFrameHeight;
		int colorFrameWidth;

		// Depth frame variables
		// we might not need this, if we want to process depth frames read-only
		int depthFrameHeight;
		int depthFrameWidth;
		int depthFramelengthInPixels;

		// The openCV buffer. Basically, use this for processing in OpenCV
		cv::Mat rgbMat;
		cv::Mat depthMat;

		// The actual buffers holding the underlying data
		boost::shared_ptr<UINT16[]> depthFrameData;
		boost::shared_ptr<BYTE[]> colorFrameData;
	};
	
	// This class does some preprocessing on the
	// depth frame before it gets passed to the hand tracker
	template <typename T>
	class DepthFrameProcessor
	{
	public:

		DepthFrameProcessor(int w, int h)
		{
			width = w;
			height = h;
		}

		~DepthFrameProcessor()
		{}

		// Methods that all derivations need
		inline void copyFrameBuffer(cv::Mat &dst) { processedFrameMat.copyTo(dst); }
		cv::Mat & getFrame() { return processedFrameMat; }

		// Pure virtual method which specifies what the processor does
		virtual void processFrame(cv::Mat & depthFrame) = 0;

	protected:

		int width;
		int height;

		boost::shared_ptr<T[]> processedBuffer;
		cv::Mat processedFrameMat;
	};

	// Processor for resizing the depth frame
	class DepthFrameResizer : public DepthFrameProcessor<unsigned short>
	{
	public:

		DepthFrameResizer();
		~DepthFrameResizer()
		{}

		// Inherited functions
		void processFrame(cv::Mat &depthFrame);

		cv::Size getSizeParameters() { return cv::Size(width, height); }
		void setResizeParameters(int w, int h);
	};

	class ColorFrameResizer : public DepthFrameProcessor<BYTE>
	{
	public:

		ColorFrameResizer();
		~ColorFrameResizer()
		{}

		void processFrame(cv::Mat &colorFrame);

		cv::Size getSizeParameters() { return cv::Size(width, height); }
		void setResizeParameters(int w, int h);
	};

	class QRFrameProcessor : public DepthFrameProcessor<BYTE>
	{
	public:

		QRFrameProcessor();
		~QRFrameProcessor()
		{}

		void processFrame(cv::Mat &colorFrame);
	};
}
