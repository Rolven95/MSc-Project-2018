#pragma once

// This the hand tracker class
// Uses OpenCV to process the depth
// frame from the sensor in a cross platform
// way
//
// @author Daniel J. Finnegan
// @date April 2016

#if defined(_WIN64)
// I'm only supporting 64-Bit Windows
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

#include <opencv2\core\core.hpp> // For OpenCV types
#include <opencv2\video\background_segm.hpp> // For background segmentation

#include <boost\smart_ptr\shared_ptr.hpp> // For boost's magical pointers
#include <boost\unordered\unordered_map.hpp> // For boost hash tables

#include "KinectTypes.h" // For the FrameDimensions struct
#include "Hand.h" // For the hand struct

// need a typedef in order to use FOREACH iteration technique
typedef boost::unordered_map<cv::Point,
	ar_sandbox::Hand,
	ar_sandbox::HandKeyHasher> CentroidHandMap;

// For associating contours with hands
typedef boost::unordered_map<ar_sandbox::Hand,
	int,
	ar_sandbox::HandKeyHasher> HandContourMap;

namespace ar_sandbox
{
	class HandTracker : public DepthFrameProcessor<unsigned char>
	{
	public:
		HandTracker(cv::Size frameDimensions);
		~HandTracker();

		void processFrame(cv::Mat &frame);
		double getFPS() { return fps; }

#if defined(USING_WINDOWS)
		const static LONG contourMinAreaThreshold = 2000;
		const static unsigned short maxDepthThreshold = 700;
		const static unsigned short minDepthThreshold = 400;
		const static unsigned int interFrameCentroidDistance = 50; // The distance threshold for tracking centroids between frames
		const static float defectsDistanceThreshold;
		const static float defectsAngleThreshold; // in degrees
#elif defined(USING_OSX)
		static long contourMaxAreaThreshold = 20000;
		static long contourMinAreaThreshold = 6000;
#endif

	private:

		void drawContourMat();
		void processDepthFrame(cv::Mat & depthMat);
		void detectHands(); // This function is where the hand tracking actually happens

		// Contours and centroids for the hand tracking algorithm
		std::vector<int> contourIndices;
		std::vector<std::vector<cv::Point> > contours;
		std::vector<ar_sandbox::Hand> hands;
		std::vector<cv::Vec4i> contourHierarchy;
		std::vector<std::pair<cv::Point, double> > palm_centers;
		int latestHandID;

		// Boost frame buffers
		boost::shared_ptr<float[]> grayBuffer;
		boost::shared_ptr<float[]> maxThresholdBuffer;
		boost::shared_ptr<float[]> minThresholdBuffer;
		boost::shared_ptr<unsigned char[]> minMaxANDThresholdBuffer;

		// Some private variables that I'm sure I'll need
		cv::Mat grayMat;
		cv::Mat maxThresholdMat;
		cv::Mat minThresholdMat;
		cv::Mat minMaxANDThresholdMat;

		// Misc stuff
#if defined(USING_WINDOWS)
		UINT64 CounterStart = 0;
		double fps;
		double PCFreq = 0.0;
#elif defined(USING_OSX)
		int64_t CounterStart = 0;
		double fps;
		double PCFreq = 0.0;
#endif
	};
}



