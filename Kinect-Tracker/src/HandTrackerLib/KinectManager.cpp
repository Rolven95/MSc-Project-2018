#include "stdafx.h"

#include "KinectManager.h"

using namespace ar_sandbox;

namespace ar_sandbox
{
	KinectManager::KinectManager()
		: depthFrameWidth(0),
		depthFrameHeight(0)
	{
	}

	KinectManager::~KinectManager()
	{
		if (mapper)
			mapper->Release();

		if (sensor)
			sensor->Close();
	}

	bool KinectManager::initSensor()
	{
#if defined(USING_WINDOWS)
		if (FAILED(GetDefaultKinectSensor(&sensor))) {
			return false;
		}
		if (sensor) {
			sensor->get_CoordinateMapper(&mapper);
			sensor->Open();
			sensor->OpenMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_Depth, &reader);
			if (reader)
				return true;
			else return false;
		}
		else {
			return false;
		}

		return true;
#endif
	}

	bool KinectManager::readColorFrame(KMultiFrame *multiFrame)
	{
		return false;
	}

	bool KinectManager::readDepthFrame(KMultiFrame *multiFrame, KDepthFrame **depthFrame)
	{
		return false;
	}

	bool KinectManager::readMultiFrame()
	{
#if defined(USING_WINDOWS)

		// Frames
		KMultiFrame *multiFrame = NULL;
		KDepthFrame *depthFrame = NULL;
		KRGBFrame *colorFrame = NULL;

		if (SUCCEEDED(reader->AcquireLatestFrame(&multiFrame)))
		{
			// References
			IColorFrameReference *colorReference = NULL;
			IDepthFrameReference *depthReference = NULL;

			// Descriptions
			KFrameDescription *colorFrameDescription = NULL;
			KFrameDescription *depthFrameDescription = NULL;

			// Read color data
			if (SUCCEEDED(multiFrame->get_ColorFrameReference(&colorReference)))
			{
				if (SUCCEEDED(colorReference->AcquireFrame(&colorFrame)))
				{
					if (colorReference) colorReference->Release();

					// If our buffer is empty, we initialise it here
					if (!colorFrameData.get())
					{
						if (SUCCEEDED(colorFrame->get_FrameDescription(&colorFrameDescription)))
						{
							unsigned int frameLength;
							int height;
							int width;
							if (SUCCEEDED(colorFrameDescription->get_LengthInPixels(&frameLength)))
								colorFrameLengthInPixels = frameLength;

							if (SUCCEEDED(colorFrameDescription->get_Height(&height)))
								colorFrameHeight = height;

							if (SUCCEEDED(colorFrameDescription->get_Width(&width)))
								colorFrameWidth = width;

							// Allocate enough memory for the RGB frame data and depth space coordinates
							colorFrameData = boost::make_shared<BYTE[]>(height * width * 4);
						}
					}

					colorFrame->CopyConvertedFrameDataToArray(colorFrameHeight * colorFrameWidth * 4,
						colorFrameData.get(),
						ColorImageFormat_Bgra);
				}
			}

			// Read depth data
			if (SUCCEEDED(multiFrame->get_DepthFrameReference(&depthReference)))
			{
				if (SUCCEEDED(depthReference->AcquireFrame(&depthFrame)))
				{
					if (depthReference) depthReference->Release();

					if (!depthFrameData.get())
					{
						if (SUCCEEDED(depthFrame->get_FrameDescription(&depthFrameDescription)))
						{
							unsigned int frameLength;
							int height;
							int width;
							if (SUCCEEDED(depthFrameDescription->get_LengthInPixels(&frameLength)))
								depthFramelengthInPixels = frameLength;

							if (SUCCEEDED(depthFrameDescription->get_Height(&height)))
								depthFrameHeight = height;

							if (SUCCEEDED(depthFrameDescription->get_Width(&width)))
								depthFrameWidth = width;

							// Allocate the depth array
							depthFrameData = boost::make_shared<UINT16[]>(height * width);
						}
					}

					// Access and copy it over
					UINT depthBufferSize = 0;
					UINT16 *depthFrameBuffer = NULL;
					depthFrame->AccessUnderlyingBuffer(&depthBufferSize, &depthFrameBuffer);
					depthFrame->CopyFrameDataToArray(depthBufferSize, depthFrameData.get());
					depthFrameBuffer = NULL;
				}
			}

			// Wrap the frame data in OpenCV Mats for processing
			rgbMat = cv::Mat(colorFrameHeight, colorFrameWidth, CV_8UC4, colorFrameData.get());
			depthMat = cv::Mat(depthFrameHeight, depthFrameWidth, CV_16U, depthFrameData.get());

			if (colorFrameDescription) colorFrameDescription->Release();
			if (depthFrameDescription) depthFrameDescription->Release();
		}

		if (multiFrame) multiFrame->Release();
		if (colorFrame) colorFrame->Release();
		if (depthFrame) depthFrame->Release();

		return true;
#endif
	}

	void KinectManager::initColorData(KMultiFrame *frame)
	{
	}

	void KinectManager::initDepthData(KMultiFrame *frame)
	{
	}

	/////////////////////////////////////////////////
	/////////////////////////////////////////////////
	/////////////////////////////////////////////////

	DepthFrameResizer::DepthFrameResizer()
		: DepthFrameProcessor(512, 512)
	{
		processedBuffer = boost::make_shared<unsigned short[]>(width * height);
		processedFrameMat = cv::Mat(width, height, CV_16U, processedBuffer.get());
	}

	void DepthFrameResizer::setResizeParameters(int w, int h)
	{
		width = w;
		height = h;
		
		// Clear and then reallocate the buffer
		processedBuffer.reset();
		processedBuffer = boost::make_shared<unsigned short[]>(width * height);
		processedFrameMat = cv::Mat(width, height, CV_16U, processedBuffer.get());
	}

	// This function copies the raw depth frame data then
	// interpolates it up to size
	void DepthFrameResizer::processFrame(cv::Mat & depthFrame)
	{
		cv::resize(depthFrame, processedFrameMat, cv::Size(width, height), 0, 0, cv::INTER_CUBIC);
	}

	/////////////////////////////////////////////////
	/////////////////////////////////////////////////
	/////////////////////////////////////////////////

	ColorFrameResizer::ColorFrameResizer()
		: DepthFrameProcessor(512, 512)
	{
		processedBuffer = boost::make_shared<BYTE[]>(width * height);
		processedFrameMat = cv::Mat(width, height, CV_8UC4, processedBuffer.get());
	}

	void ColorFrameResizer::setResizeParameters(int w, int h)
	{
		width = w;
		height = h;
		
		// Clear and then reallocate the buffer
		processedBuffer.reset();
		processedBuffer = boost::make_shared<BYTE[]>(width * height * 4);
		processedFrameMat = cv::Mat(width, height, CV_8UC4, processedBuffer.get());
	}

	// This function copies the raw depth frame data then
	// interpolates it up to size
	void ColorFrameResizer::processFrame(cv::Mat & colorFrame)
	{
		cv::resize(colorFrame, processedFrameMat, cv::Size(width, height), 0, 0, cv::INTER_CUBIC);
	}
}

