// This is basically a run through of the Unity version

#define DO_TEST_APP

#if defined(DO_TEST_APP)
#include <Unity.h>
#else
#include <HandTrackerLib.h>
#endif

#include <opencv2\highgui\highgui.hpp>
#include <iostream>

#define RESIZE_WIDTH 1280
#define RESIZE_HEIGHT 720
#define idnumber 30
int main(int argc, char **argv)
{

#if defined(DO_TEST_APP)

	//cv::namedWindow("Depth display");
	//cv::namedWindow("Color display");
	//cv::namedWindow("HandTracker display");

	BYTE *bp = nullptr;
	unsigned short *depthsp = nullptr;
	BYTE *colorsp = nullptr;
	int *intP = new int;

	int qrresults[30][6];
	//for (int i = 0; i < 30; i++)
	//{
	//	for (int j = 0; j < 6; j++)
	//	{
	//		qrresults[i][j] = 99;
	//	}
	//}

	initEnv();

	for (;;)
	{
		updateSensor();
		updateProcessor();
		//updateHandTracker();

		//getContourFrame(&bp, intP);
		//getDepthFrame(&depthsp, intP);
		//getColorFrame(&colorsp, intP);
		getQRTraceFrame(&colorsp, intP);
		// Show all three frames in separate windows
		//cv::Mat depthFrame(512, 512, CV_16U, depthsp);
		//imshow("Depth display", depthFrame);
		//if (cv::waitKey(1) >= 0) break;
		
		getQRResult(qrresults);
		//if (getQRResult(qrresults))
		//{
		//	for (int i = 0; i < idnumber; i++) {
		//		if (qrresults[i][0] != 99 && qrresults[i][0] >=0) {
		//			printf(" data in all info : ");
		//			for (int q = 0; q < 6; q++)
		//			{
		//				printf("%d ", qrresults[i][q]);
		//			}
		//		}
		//	}
		//}

		cv::Mat colorFrame(RESIZE_HEIGHT, RESIZE_WIDTH, CV_8UC4, colorsp);
		imshow("Color display", colorFrame);
		if (cv::waitKey(1) >= 0) break;


		/*cv::Mat contourFrame(512, 512, CV_8UC3, bp);
		imshow("HandTracker display", contourFrame);
		if (cv::waitKey(1) >= 0) break;*/


	}

	delete intP;

	destroyEnv();

#else

	// Variables
	boost::shared_ptr<ar_sandbox::KinectManager> kinectManager(new ar_sandbox::KinectManager);
	boost::shared_ptr<ar_sandbox::DepthFrameResizer> depthFrameProcessor(new ar_sandbox::DepthFrameResizer);

	// Initiate the processor and then the hand tracker
	depthFrameProcessor->setResizeParameters(RESIZE_HEIGHT, RESIZE_WIDTH);
	cv::Size processParams = depthFrameProcessor->getSizeParameters();
	boost::shared_ptr<ar_sandbox::HandTracker> handTracker(new ar_sandbox::HandTracker(processParams));

	// Start the depth sensor
	bool didInit = kinectManager->initSensor();

	// Enter the main application loop if all went
	// according to plan
	if (didInit)
	{
		cv::namedWindow("Kinect RGB Frames", 1); // TODO: Look up the various flags possible here

		// Ring-a-round-a-rosey until we have data to play with
		do
		{
			kinectManager->readMultiFrame();
		} while (kinectManager->getDepthDimensions().width <= 0);

		for (;;)
		{
			// Ask the sensor...
			kinectManager->readMultiFrame();
			cv::Mat depthFrame = kinectManager->getDepthMat();

			// Process it
			depthFrameProcessor->processFrame(depthFrame);
			cv::Mat processedDepthFrame = depthFrameProcessor->getFrame();

			// Process the frame
			handTracker->processFrame(processedDepthFrame);
			cv::Mat frameToDisplay = handTracker->getContourMat();

			// If we have data to display, then display it
			if (depthFrame.data)
			{
				// Get the FPS
				//double fps = handTracker->getFPS();
				//std::ostringstream fpsStream;
				//fpsStream << "FPS: " << floorf((float) fps * 100) / 100;
				//std::string fpsStr = fpsStream.str();
				//cv::putText(frameToDisplay, fpsStr, cv::Point(10, frameToDisplay.size().height - 10),
				//	cv::HersheyFonts::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 50, 20, 0));

				// Display it
				imshow("Kinect RGB Frames", frameToDisplay);
			}

			if (cv::waitKey(1) >= 0) break;
		}
	}
	else
	{
		std::cout << "The kinect sensor couldn't be initialised" << std::endl;
		return -1;
	}

	return 0;

#endif
}



