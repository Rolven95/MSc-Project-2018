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

using namespace cv;
using namespace std;

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

		cv::Size getSizeParameters() { return cv::Size(height, width); }
		void setResizeParameters(int w, int h);
	};

	class ColorFrameResizer : public DepthFrameProcessor<BYTE>
	{
	public:

		ColorFrameResizer();
		~ColorFrameResizer()
		{}

		void processFrame(cv::Mat &colorFrame);

		cv::Size getSizeParameters() { return cv::Size(height, width); }
		void setResizeParameters(int w, int h);
	};

	class QRFrameProcessor : public DepthFrameProcessor<BYTE>
	{
	public:

		QRFrameProcessor();
		~QRFrameProcessor()
		{}

		const int CV_QR_NORTH = 0;
		const int CV_QR_EAST = 1;
		const int CV_QR_SOUTH = 2;
		const int CV_QR_WEST = 3;
		float cv_returnX(Point2f X);
		float cv_returnY(Point2f Y);
		float cv_distance(Point2f P, Point2f Q);					// Get Distance between two points
		float cv_lineEquation(Point2f L, Point2f M, Point2f J);		// Perpendicular Distance of a Point J from line formed by Points L and M; Solution to equation of the line Val = ax+by+c 
		float cv_lineSlope(Point2f L, Point2f M, int& alignement);	// Slope of a line by two Points L and M on it; Slope of line, S = (x1 -x2) / (y1- y2)
		
		void cv_getVertices(vector<vector<Point> > contours, int c_id, float slope, vector<Point2f>& X);
		void cv_updateCorner(Point2f P, Point2f ref, float& baseline, Point2f& corner);
		void cv_updateCornerOr(int orientation, vector<Point2f> _IN, vector<Point2f> & _OUT);
		bool getIntersectionPoint(Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& intersection);
		float cross(Point2f v1, Point2f v2);

		void processFrame(cv::Mat &colorFrame);

	protected:
		cv::Mat Traces;
	};

	class ObjectInfo
	{
	public:

	protected:
		float X;
		float Y;
		float Depth;
		float Rotation;
		int ID;
	};
}
