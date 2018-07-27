#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "KinectManager.h"
#include <cmath>
#include <omp.h>


using namespace ar_sandbox;
using namespace cv;
using namespace std;

#define idnumber 30
#define RESIZE_WIDTH 1920
#define RESIZE_HEIGHT 1080
#define QRSIZE 128
#define blackorwhite 180
#define lifetime 5
#define lifetimeextand 5
#define updatelimit 10


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
							colorFrameData = boost::make_shared<BYTE[]>(width * height * 4);
						}
					}

					colorFrame->CopyConvertedFrameDataToArray(colorFrameWidth * colorFrameHeight * 4,
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
							depthFrameData = boost::make_shared<UINT16[]>(width * height);
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
		: DepthFrameProcessor(RESIZE_HEIGHT, RESIZE_WIDTH)
	{
		processedBuffer = boost::make_shared<unsigned short[]>(width * height);
		processedFrameMat = cv::Mat(height, width, CV_16U, processedBuffer.get());
	}

	void DepthFrameResizer::setResizeParameters(int w, int h)
	{
		width = w;
		height = h;

		// Clear and then reallocate the buffer
		processedBuffer.reset();
		processedBuffer = boost::make_shared<unsigned short[]>(width * height);
		processedFrameMat = cv::Mat(height, width, CV_16U, processedBuffer.get());
	}

	// This function copies the raw depth frame data then
	// interpolates it up to size
	void DepthFrameResizer::processFrame(cv::Mat & depthFrame)
	{
		cv::resize(depthFrame, processedFrameMat, cv::Size(height, width), 0, 0, cv::INTER_CUBIC);
	}

	/////////////////////////////////////////////////
	/////////////////////////////////////////////////
	/////////////////////////////////////////////////

	ColorFrameResizer::ColorFrameResizer()
		: DepthFrameProcessor(RESIZE_HEIGHT, RESIZE_WIDTH)
	{
		processedBuffer = boost::make_shared<BYTE[]>(width * height);
		processedFrameMat = cv::Mat(height, width, CV_8UC4, processedBuffer.get());
	}

	void ColorFrameResizer::setResizeParameters(int w, int h)
	{
		width = w;
		height = h;

		// Clear and then reallocate the buffer
		processedBuffer.reset();
		processedBuffer = boost::make_shared<BYTE[]>(width * height * 4);
		processedFrameMat = cv::Mat(height, width, CV_8UC4, processedBuffer.get());
	}

	// This function copies the raw depth frame data then
	// interpolates it up to size
	void ColorFrameResizer::processFrame(cv::Mat & colorFrame)
	{
		cv::resize(colorFrame, processedFrameMat, cv::Size(height, width), 0, 0, cv::INTER_CUBIC);
	}


	// TODO: Implement this

	QRFrameProcessor::QRFrameProcessor()
		: DepthFrameProcessor(RESIZE_HEIGHT, RESIZE_WIDTH)
	{
		processedBuffer = boost::make_shared<BYTE[]>(width * height);
		processedFrameMat = cv::Mat(height, width, CV_8UC4, processedBuffer.get());
	}

	// Function: Routine to get Distance between two points
	// Description: Given 2 points, the function returns the distance

	float QRFrameProcessor::cv_returnX(Point2f X)
	{
		float number = X.x;
		return number;
	}

	float QRFrameProcessor::cv_returnY(Point2f Y) {

		return Y.y;
	}

	float QRFrameProcessor::cv_distance(Point2f P, Point2f Q)
	{
		return sqrt(pow(abs(P.x - Q.x), 2) + pow(abs(P.y - Q.y), 2));
	}

	// Function: Perpendicular Distance of a Point J from line formed by Points L and M; Equation of the line ax+by+c=0
	// Description: Given 3 points, the function derives the line quation of the first two points,
	//	  calculates and returns the perpendicular distance of the the 3rd point from this line.

	float QRFrameProcessor::cv_lineEquation(Point2f L, Point2f M, Point2f J)
	{
		float a, b, c, pdist;

		a = -((M.y - L.y) / (M.x - L.x));
		b = 1.0;
		c = (((M.y - L.y) / (M.x - L.x)) * L.x) - L.y;

		// Now that we have a, b, c from the equation ax + by + c, time to substitute (x,y) by values from the Point J

		pdist = (a * J.x + (b * J.y) + c) / sqrt((a * a) + (b * b));
		return pdist;
	}

	// Function: Slope of a line by two Points L and M on it; Slope of line, S = (x1 -x2) / (y1- y2)
	// Description: Function returns the slope of the line formed by given 2 points, the alignement flag
	//	  indicates the line is vertical and the slope is infinity.

	float QRFrameProcessor::cv_lineSlope(Point2f L, Point2f M, int& alignement)
	{
		float dx, dy;
		dx = M.x - L.x;
		dy = M.y - L.y;

		if (dy != 0)
		{
			alignement = 1;
			return (dy / dx);
		}
		else				// Make sure we are not dividing by zero; so use 'alignement' flag
		{
			alignement = 0;
			return 0.0;
		}
	}



	// Function: Routine to calculate 4 Corners of the Marker _IN Image Space using Region partitioning
	// Theory: OpenCV Contours stores all points that describe it and these points lie the perimeter of the polygon.
	//	The below function chooses the farthest points of the polygon since they form the vertices of that polygon,
	//	exactly the points we are looking for. To choose the farthest point, the polygon is divided/partitioned into
	//	4 regions equal regions using bounding box. Distance algorithm is applied between the centre of bounding box
	//	every contour point _IN that region, the farthest point is deemed as the vertex of that region. Calculating
	//	for all 4 regions we obtain the 4 corners of the polygon ( - quadrilateral).
	void QRFrameProcessor::cv_getVertices(vector<vector<Point> > contours, int c_id, float slope, vector<Point2f>& quad)
	{
		
		Rect box;
		box = boundingRect(contours[c_id]);

		Point2f M0, M1, M2, M3;
		Point2f A, B, C, D, W, X, Y, Z;

		A = box.tl();
		B.x = box.br().x;
		B.y = box.tl().y;
		C = box.br();
		D.x = box.tl().x;
		D.y = box.br().y;


		W.x = (A.x + B.x) / 2;
		W.y = A.y;

		X.x = B.x;
		X.y = (B.y + C.y) / 2;

		Y.x = (C.x + D.x) / 2;
		Y.y = C.y;

		Z.x = D.x;
		Z.y = (D.y + A.y) / 2;

		float dmax[4];
		dmax[0] = 0.0;
		dmax[1] = 0.0;
		dmax[2] = 0.0;
		dmax[3] = 0.0;

		float pd1 = 0.0;
		float pd2 = 0.0;

		if (slope > 5 || slope < -5)
		{
			for (int i = 0; i < contours[c_id].size(); i++)
			{
				pd1 = cv_lineEquation(C, A, contours[c_id][i]);	// Position of point w.r.t the diagonal AC 
				pd2 = cv_lineEquation(B, D, contours[c_id][i]);	// Position of point w.r.t the diagonal BD

				if ((pd1 >= 0.0) && (pd2 > 0.0))
				{
					cv_updateCorner(contours[c_id][i], W, dmax[1], M1);
				}
				else if ((pd1 > 0.0) && (pd2 <= 0.0))
				{
					cv_updateCorner(contours[c_id][i], X, dmax[2], M2);
				}
				else if ((pd1 <= 0.0) && (pd2 < 0.0))
				{
					cv_updateCorner(contours[c_id][i], Y, dmax[3], M3);
				}
				else if ((pd1 < 0.0) && (pd2 >= 0.0))
				{
					cv_updateCorner(contours[c_id][i], Z, dmax[0], M0);
				}
				else
					continue;
			}
		}
		else
		{
			int halfx = (A.x + B.x) / 2;
			int halfy = (A.y + D.y) / 2;
 
			for (int i = 0; i < contours[c_id].size(); i++)
			{
				if ((contours[c_id][i].x < halfx) && (contours[c_id][i].y <= halfy))
				{
					cv_updateCorner(contours[c_id][i], C, dmax[2], M0);
				}
				else if ((contours[c_id][i].x >= halfx) && (contours[c_id][i].y < halfy))
				{
					cv_updateCorner(contours[c_id][i], D, dmax[3], M1);
				}
				else if ((contours[c_id][i].x > halfx) && (contours[c_id][i].y >= halfy))
				{
					cv_updateCorner(contours[c_id][i], A, dmax[0], M2);
				}
				else if ((contours[c_id][i].x <= halfx) && (contours[c_id][i].y > halfy))
				{
					cv_updateCorner(contours[c_id][i], B, dmax[1], M3);
				}
			}
		}

		quad.push_back(M0);
		quad.push_back(M1);
		quad.push_back(M2);
		quad.push_back(M3);

	}

	// Function: Compare a point if it more far than previously recorded farthest distance
	// Description: Farthest Point detection using reference point and baseline distance
	void QRFrameProcessor::cv_updateCorner(Point2f P, Point2f ref, float& baseline, Point2f& corner)
	{
		float temp_dist;
		temp_dist = cv_distance(P, ref);

		if (temp_dist > baseline)
		{
			baseline = temp_dist;			// The farthest distance is the new baseline
			corner = P;						// P is now the farthest point
		}

	}

	// Function: Sequence the Corners wrt to the orientation of the QR Code
	void QRFrameProcessor::cv_updateCornerOr(int orientation, vector<Point2f> _IN, vector<Point2f> &_OUT)
	{
		Point2f M0, M1, M2, M3;
		if (orientation == CV_QR_NORTH)
		{
			M0 = _IN[0];
			M1 = _IN[1];
			M2 = _IN[2];
			M3 = _IN[3];
		}
		else if (orientation == CV_QR_EAST)
		{
			M0 = _IN[1];
			M1 = _IN[2];
			M2 = _IN[3];
			M3 = _IN[0];
		}
		else if (orientation == CV_QR_SOUTH)
		{
			M0 = _IN[2];
			M1 = _IN[3];
			M2 = _IN[0];
			M3 = _IN[1];
		}
		else if (orientation == CV_QR_WEST)
		{
			M0 = _IN[3];
			M1 = _IN[0];
			M2 = _IN[1];
			M3 = _IN[2];
		}

		_OUT.push_back(M0);
		_OUT.push_back(M1);
		_OUT.push_back(M2);
		_OUT.push_back(M3);
	}

	// Function: Get the Intersection Point of the lines formed by sets of two points
	bool QRFrameProcessor::getIntersectionPoint(Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& intersection)
	{
		Point2f p = a1;
		Point2f q = b1;
		Point2f r(a2 - a1);
		Point2f s(b2 - b1);

		if (cross(r, s) == 0) { return false; }

		float t = cross(q - p, s) / cross(r, s);

		intersection = p + t*r;
		return true;
	}

	float QRFrameProcessor::cross(Point2f v1, Point2f v2)
	{
		return v1.x*v2.y - v1.y*v2.x;
	}


	Mat QRFrameProcessor::qr_warper(Point2f N, Point2f E, Point2f S, Point2f W, Mat graph) {
		Mat area, area_raw, area_gray, area_thres;
		vector<Point2f> src, dst;		// src - Source Points basically the 4 end co-ordinates of the overlay image
										// dst - Destination Points to transform overlay image
		Mat warp_matrix;
		area_raw = Mat::zeros(QRSIZE, QRSIZE, CV_8UC3);
		area = Mat::zeros(QRSIZE, QRSIZE, CV_8UC3);
		area_gray = Mat::zeros(QRSIZE, QRSIZE, CV_8UC1);
		area_thres = Mat::zeros(QRSIZE, QRSIZE, CV_8UC1);

		src.push_back(N);
		src.push_back(E);
		src.push_back(S);
		src.push_back(W);

		dst.push_back(Point2f(0, 0));
		dst.push_back(Point2f(area.cols, 0));
		dst.push_back(Point2f(area.cols, area.rows));
		dst.push_back(Point2f(0, area.rows));


		if (src.size() == 4 && dst.size() == 4)			// Failsafe for WarpMatrix Calculation to have only 4 Points with src and dst
		{
			//printf("got four");
			warp_matrix = getPerspectiveTransform(src, dst);
			warpPerspective(graph, area_raw, warp_matrix, Size(area.cols, area.rows));
			copyMakeBorder(area_raw, area, 10, 10, 10, 10, BORDER_CONSTANT, Scalar(255, 255, 255));

			cvtColor(area, area_gray, CV_RGB2GRAY);
			//imshow("area code", area_gray);
		}
		threshold(area_gray, area_gray, 127, 255, CV_THRESH_BINARY);
		return area_gray;
	}

	void QRFrameProcessor::position_finder(Point2f L, Point2f M, Point2f O, int info[5]) {
		float degree = atan2(M.x - L.x, M.y - L.y) * 180 / 3.1415926;
		info[4] = degree;
		info[0] = -1;
		info[1] = (M.x + L.x) / 2;
		info[2] = (L.y + O.y) / 2;
		info[3] = -1;
	}

	void QRFrameProcessor::code_digger(int code[], Mat qr) {
		int col =  qr.rows;//x
		int row =  qr.cols;//y
		int c_points[4] = { col / 25 * 7, col / 25 * (7 + 2) ,col / 25 * (7 + 2 + 7) ,col / 25 * (7 + 2 + 7 + 2) };
		int r_points[4] = { row / 25 * 7, row / 25 * (7 + 2) ,row / 25 * (7 + 2 + 7) ,row / 25 * (7 + 2 + 7 + 2) };
		Mat temp_mat;

		temp_mat = qr.colRange(Range(c_points[1], c_points[2])); //x
		temp_mat = temp_mat.rowRange(Range(0, r_points[0])); // 

		if (average_gray_scale(temp_mat) < blackorwhite)
			code[0] = 1;
		else
			code[0] = 0;

		//this is area 1; 
		temp_mat = qr.colRange(Range(0, c_points[0])); //x
		temp_mat = temp_mat.rowRange(Range(r_points[1], r_points[2])); // y
		if (average_gray_scale(temp_mat) < blackorwhite)
			code[1] = 1;
		else
			code[1] = 0;

		//this is area 2; 
		temp_mat = qr.colRange(Range(c_points[1], c_points[2])); //x
		temp_mat = temp_mat.rowRange(Range(r_points[1], r_points[2])); // y
		if (average_gray_scale(temp_mat) < blackorwhite)
			code[2] = 1;
		else
			code[2] = 0;
    
		//this is area 3; 
		temp_mat = qr.colRange(Range(c_points[3], col)); //x
		temp_mat = temp_mat.rowRange(Range(r_points[1], r_points[2])); // y
		if (average_gray_scale(temp_mat) < blackorwhite)
			code[3] = 1;
		else
			code[3] = 0;
	
		//this is area 4; 
		temp_mat = qr.colRange(Range(c_points[1], c_points[2])); //x
		temp_mat = temp_mat.rowRange(Range(r_points[3], row)); // y
		if (average_gray_scale(temp_mat) < blackorwhite)
			code[4] = 1;
		else
			code[4] = 0;

		//this is area 5; 
		temp_mat = qr.colRange(Range(c_points[3], col)); //x
		temp_mat = temp_mat.rowRange(Range(r_points[3], row)); // y
		if (average_gray_scale(temp_mat) < blackorwhite)
			code[5] = 1;
		else
			code[5] = 0;
	
	}

	int QRFrameProcessor::decoder(int code[]) {
		int output = 0;
		output = output + code[0] * 32;
		output = output + code[1] * 16;
		output = output + code[2] * 8;
		output = output + code[3] * 4;
		output = output + code[4] * 2;
		output = output + code[5] * 1;
		return output;
	}

	int QRFrameProcessor::average_gray_scale(Mat input) {
		int b = 0;
		int xmax, ymax;
		int total_gray_scale = 0;
		Mat graph;
		//graph = Mat::zeros(input.size(), CV_8UC1);
		graph = input;

		ymax = graph.rows - 1;
		xmax = graph.cols - 1;
		//printf("col is %d ", xmax);
		//printf("row is %d ", ymax);
		//printf("gray is %d ", int(graph.at<uchar>(0, 0)));
//#pragma omp parallel for  
		//to do:
		for (int x = 0; x < xmax; x++) {
			for (int y = 0; y < ymax; y++) {
				total_gray_scale = total_gray_scale + int(graph.at<uchar>(y, x));
				//printf("(%d,%d)", y,x);
				//printf(" %d ", int(graph.at<uchar>(y, x)));
			}
		}
		//printf("avg gray = %d " , total_gray_scale/(graph.rows*graph.cols));
		return total_gray_scale / (graph.rows*graph.cols);
		//return 0;
	}

	void QRFrameProcessor::list_insert(int input[5], int list[idnumber][6]) {
		//printf(" start inserting ");
		bool same = 0;
		for (int i = 0; i < idnumber; i++) {
			if (list[i][0] == input[0]) {
				list[i][5] = lifetimeextand + list[i][5];
				same = 1;
				//printf(" found same ");
				printf("life time of No.%d at %d is %d ", list[i][0], i, list[i][5]);

				if (list[i][5] > updatelimit) {
					list[i][1] = input[1];
					list[i][2] = input[2];
					list[i][3] = input[3];
					list[i][4] = input[4];
				}
			}
		}

		if (same == 0) {
			//printf(" no same ");
			for (int i = 0; i < idnumber; i++) {
				if (list[i][0] == 99) {
					//printf(" found umpty place ");
					list[i][0] = input[0];
					list[i][1] = input[1];
					list[i][2] = input[2];
					list[i][3] = input[3];
					list[i][4] = input[4];
					list[i][5] = lifetime;
					//printf("No. %d inserted at %d ", input[0], i);
					return;
				}
			}
		}
	}

	void QRFrameProcessor::list_manager(int list[idnumber][6]) {
		//printf(" start managing ");
		for (int i = 0; i < idnumber; i++) {
			if (list[i][0] != 99) {							

				list[i][5]--; // life time -1 per frame
				//printf(" No.%d -1,now is %d ", list[i][0], list[i][5]);

				if (list[i][5] < 0) { // when life time < 0, we reset this array.
					//printf(" No.%d expired ", list[i][0]);
					list[i][0] = 99;
					list[i][1] = 0;
					list[i][2] = 0;
					list[i][3] = 0;
					list[i][4] = 0;
					list[i][5] = 0;
				}				
			}
		}
	}


	void QRFrameProcessor::processFrame(cv::Mat & colorFrame)
	{
		

		// we crop the image to reduce the pixels
		Mat temp_mat;
		temp_mat = colorFrame.colRange(Range(800, 1400)); //x
		temp_mat = temp_mat.rowRange(Range(300, 700)); // y
		//temp_mat = colorFrame.colRange(Range(700, 1700)); //x
		//temp_mat = temp_mat.rowRange(Range(300, 1000)); // y
		temp_mat.copyTo(colorFrame);


		// Creation of Intermediate 'Image' Objects required later
		Mat gray(colorFrame.size(), CV_MAKETYPE(colorFrame.depth(), 1));			// To hold Grayscale Image
		Mat edges(colorFrame.size(), CV_MAKETYPE(colorFrame.depth(), 1));			// To hold Grayscale Image

	    Mat Traces =  Mat(colorFrame.size(), CV_8UC3);

		list_manager(allinfo);

		//imshow("colorFrame", colorFrame);

		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		vector<Point> pointsseq;    //used to save the approximated sides of each contour

		int mark, mapcounter;

		// From here, you can do all your QR processing on image
		cvtColor(colorFrame, gray, CV_RGB2GRAY);		// Convert Image captured from Image Input to GrayScale	


														//uchar pixel_value = gray.ptr<uchar>(10)[10];
														//printf(" point gray is: %d ", pixel_value);
		blur(gray, gray, Size(1, 1));
		Canny(gray, edges, 100, 200, 3);		// Apply Canny edge detection on the gray image	
		findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE); // Find contours with hierarchy


		mark = 0;								// Reset all detected marker count for this frame
		mapcounter = 0;
		// Get Moments for all Contours and the mass centers
		vector<Moments> mu(contours.size());
		vector<Point2f> mc(contours.size());
		
#pragma omp parallel for		
		for (int i = 0; i < contours.size(); i++)
		{
			mu[i] = moments(contours[i], false);
			mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
		}


		int id[idnumber]; // one dimention identifiers list//count all identifiers, and store _IN id[];
		int qrcode[idnumber / 3][3];
		int mappoint[3];
		for (int i = 0; i < idnumber; i++)
			id[i] = -1;
		for (int i = 0; i < idnumber / 3; i++)
			qrcode[i][0] = -1;
 
		for (int i = 0; i < contours.size(); i++)
		{
			//Find the approximated polygon of the contour we are examining
			approxPolyDP(contours[i], pointsseq, arcLength(contours[i], true)*0.02, true);
			if (pointsseq.size() == 4)      // only quadrilaterals contours are examined
			{
				int k = i;
				int c = 0;
				while (hierarchy[k][2] != -1)
				{
					k = hierarchy[k][2];
					c = c + 1;
				}
				if (hierarchy[k][2] != -1)
					c = c + 1;

				if (c >= 5)
				{
					id[mark] = i;
					mark = mark + 1;
				}
			}
		}

			float alldistance[idnumber][idnumber]; //theorical maxium size
			float smalldistance = 9999;

			
#pragma omp parallel for   
			for (int i = 0; i < mark; i++)  //actrual reading
			{
				for (int q = 0; q < mark; q++)
				{
					alldistance[i][q] = cv_distance(mc[id[i]], mc[id[q]]);
					if (alldistance[i][q] > 0 && alldistance[i][q] < smalldistance)
						smalldistance = alldistance[i][q];
					//printf(" %d%d = %f ", i, q, alldistance[i][q]);
				}
			}
			//printf(" small dis = %f ", smalldistance);

			int qrcounter = 0; //how many qr codes, should be mark/3
							  	 			
			for (int i = 0; i < mark; i++) //check all points
			{
				//alldistance[i][i] = -1;
				if (alldistance[i][0] != -1) {
					int min1 = -1;
					int min2 = -1;

					for (int m = 0; m < mark; m++) { //give min1 and min2 initial value, non-zero value
												
						if (min1 == -1 && m != i  && alldistance[m][0] != -1)
							min1 = m;
						else if (min2 == -1 && m != i  && alldistance[m][0] != -1)
							min2 = m;
					
					}
					if (min1 == -1 || min2 == -1) {
						//printf(" not enough ");
						continue;
					}

					for (int q = i; q < mark; q++) {
						if (alldistance[i][q] != -1 && alldistance[i][q] < alldistance[i][min1] && q != i && q != min2) {
							min1 = q;
						}
						else if (alldistance[i][q] != -1 && alldistance[i][q] < alldistance[i][min2] && q != i && q != min1) {
							min2 = q;
						}
					}

					
					if (alldistance[i][min1] > smalldistance * 1.6 || alldistance[i][min2] > smalldistance * 1.6) {
						//printf(" wrong ");
						continue;
					}

					float AB, BC, CA;//i=a min1 =b min2 =c
					AB = alldistance[i][min1];
					BC = alldistance[min1][min2];
					CA = alldistance[min2][i];
					//printf(" dis: %f %f %f ", AB, BC, CA);

					if (AB > BC && AB > CA)
					{
						qrcode[qrcounter][0] = id[min2]; qrcode[qrcounter][1] = id[i]; qrcode[qrcounter][2] = id[min1];
					}
					else if (CA > AB && CA > BC)
					{
						qrcode[qrcounter][0] = id[min1]; qrcode[qrcounter][1] = id[i]; qrcode[qrcounter][2] = id[min2];
					}
					else if (BC > AB && BC > CA)
					{
						qrcode[qrcounter][0] = id[i]; qrcode[qrcounter][1] = id[min1]; qrcode[qrcounter][2] = id[min2];
					}
					
					
					if (alldistance[i][0] == -1 || alldistance[min1][0] == -1 || alldistance[min2][0] == -1) {
						continue;
					}
					else {
						alldistance[i][0] = -1;
						alldistance[min1][0] = -1;
						alldistance[min2][0] = -1;
						qrcounter++;
					}
					

				}
			}


			if (qrcounter > 0)		// Ensure we have (atleast 3; namely A,B,C) 'Alignment Markers' discovered
			{
				//printf("\n max is %d\n", omp_get_max_threads());
#pragma omp parallel for 			
				for (int i = 0; i < qrcounter; i++) {
					float dist, slope;
					int align, orientation;
					int top, right, bottom, median1, median2, outlier;

					outlier = qrcode[i][0];
					median1 = qrcode[i][1];
					median2 = qrcode[i][2];
					top = qrcode[i][0];

					dist = cv_lineEquation(mc[median1], mc[median2], mc[outlier]);	// Get the Perpendicular distance of the outlier from the longest side			
					slope = cv_lineSlope(mc[median1], mc[median2], align);		// Also calculate the slope of the longest side

																				// Now that we have the orientation of the line formed median1 & median2 and we also have the position of the outlier w.r.t. the line
																				// Determine the 'right' and 'bottom' markers

					if (align == 0)
					{
						bottom = median1;
						right = median2;
					}
					else if (slope < 0 && dist < 0)		// Orientation - North
					{
						bottom = median1;
						right = median2;
						orientation = CV_QR_NORTH;
					}
					else if (slope > 0 && dist < 0)		// Orientation - East
					{
						right = median1;
						bottom = median2;
						orientation = CV_QR_EAST;
					}
					else if (slope < 0 && dist > 0)		// Orientation - South			
					{
						right = median1;
						bottom = median2;
						orientation = CV_QR_SOUTH;
					}

					else if (slope > 0 && dist > 0)		// Orientation - West
					{
						bottom = median1;
						right = median2;
						orientation = CV_QR_WEST;
					}


					// To ensure any unintended values do not sneak up when QR code is not present
					float area_top, area_right, area_bottom;

					if (top < contours.size() && right < contours.size() && bottom < contours.size() && contourArea(contours[top]) > 10 && contourArea(contours[right]) > 10 && contourArea(contours[bottom]) > 10)
					{

						vector<Point2f> L, M, O, tempL, tempM, tempO;
						Point2f N;

						cv_getVertices(contours, top, slope, tempL);
						cv_getVertices(contours, right, slope, tempM);
						cv_getVertices(contours, bottom, slope, tempO);

						cv_updateCornerOr(orientation, tempL, L); 			// Re-arrange marker corners w.r.t orientation of the QR code
						cv_updateCornerOr(orientation, tempM, M); 			// Re-arrange marker corners w.r.t orientation of the QR code
						cv_updateCornerOr(orientation, tempO, O); 			// Re-arrange marker corners w.r.t orientation of the QR code

						int iflag = getIntersectionPoint(M[1], M[2], O[3], O[2], N);

						//threshold(qr_gray, qr_thres, 127, 255, CV_THRESH_BINARY);

						//Draw contours on the image
						drawContours(colorFrame, contours, top, Scalar(255, 200, 0), 2, 8, hierarchy, 0);
						drawContours(colorFrame, contours, right, Scalar(0, 0, 255), 2, 8, hierarchy, 0);
						drawContours(colorFrame, contours, bottom, Scalar(255, 0, 100), 2, 8, hierarchy, 0);


						int DBG = 0;
						// Insert Debug instructions here
						if (DBG == 1)
						{					
							// Debug Prints
							// Visualizations for ease of understanding
							if (slope > 5)
								circle(Traces, Point(10, 20), 5, Scalar(0, 0, 255), -1, 8, 0);
							else if (slope < -5)
								circle(Traces, Point(10, 20), 5, Scalar(255, 255, 255), -1, 8, 0);

							// Draw contours on Trace image for analysis	
							drawContours(Traces, contours, top, Scalar(255, 0, 100,0), 1, 8, hierarchy, 0);
							drawContours(Traces, contours, right, Scalar(255, 0, 100), 1, 8, hierarchy, 0);
							drawContours(Traces, contours, bottom, Scalar(255, 0, 100), 1, 8, hierarchy, 0);

							// Draw points (4 corners) on Trace image for each Identification marker	
							circle(Traces, L[0], 2, Scalar(255, 255, 0), -1, 8, 0);
							circle(Traces, L[1], 2, Scalar(0, 255, 0), -1, 8, 0);
							circle(Traces, L[2], 2, Scalar(0, 0, 255), -1, 8, 0);
							circle(Traces, L[3], 2, Scalar(128, 128, 128), -1, 8, 0);

							//printf(" L0 x is  %d , L0 y is %d ", L[0].x, L[0].y);

							circle(Traces, M[0], 2, Scalar(255, 255, 0), -1, 8, 0);
							circle(Traces, M[1], 2, Scalar(0, 255, 0), -1, 8, 0);
							circle(Traces, M[2], 2, Scalar(0, 0, 255), -1, 8, 0);
							circle(Traces, M[3], 2, Scalar(128, 128, 128), -1, 8, 0);

							circle(Traces, O[0], 2, Scalar(255, 255, 0), -1, 8, 0);
							circle(Traces, O[1], 2, Scalar(0, 255, 0), -1, 8, 0);
							circle(Traces, O[2], 2, Scalar(0, 0, 255), -1, 8, 0);
							circle(Traces, O[3], 2, Scalar(128, 128, 128), -1, 8, 0);

							// Draw point of the estimated 4th Corner of (entire) QR Code
							circle(Traces, N, 2, Scalar(255, 255, 255), -1, 8, 0);

							//printf("L0(%f,%f), L2(%f,%f)", L[0].x, L[0].y, L[2].x, L[2].y);

							//gray.at<uchar>(10, 200) = 255;
							//printf("height is  %d ,  weight is ", int(gray.at<uchar>(700,1200)));
							//printf("row range is  %d ", gray.weight );

							//int a = average_gray_scale(L[0], L[1], L[2], L[3], image);

							// Draw the lines used for estimating the 4th Corner of QR Code
							line(Traces, M[1], N, Scalar(0, 0, 255), 1, 8, 0);
							line(Traces, O[3], N, Scalar(0, 0, 255), 1, 8, 0);
							
							// Debug Prints
						}
						else if (DBG == 2)
						{
							// Debug Prints
							// Visualizations for ease of understanding
							if (slope > 5)
								circle(colorFrame, Point(10, 20), 5, Scalar(0, 0, 255), -1, 8, 0);
							else if (slope < -5)
								circle(colorFrame, Point(10, 20), 5, Scalar(255, 255, 255), -1, 8, 0);

							// Draw contours on Trace image for analysis	
							drawContours(colorFrame, contours, top, Scalar(255, 0, 100), 1, 8, hierarchy, 0);
							drawContours(colorFrame, contours, right, Scalar(255, 0, 100), 1, 8, hierarchy, 0);
							drawContours(colorFrame, contours, bottom, Scalar(255, 0, 100), 1, 8, hierarchy, 0);

							// Draw points (4 corners) on Trace image for each Identification marker	
							circle(colorFrame, L[0], 2, Scalar(255, 255, 0), -1, 8, 0);
							circle(colorFrame, L[1], 2, Scalar(0, 255, 0), -1, 8, 0);
							circle(colorFrame, L[2], 2, Scalar(0, 0, 255), -1, 8, 0);
							circle(colorFrame, L[3], 2, Scalar(128, 128, 128), -1, 8, 0);

							//printf(" L0 x is  %d , L0 y is %d ", L[0].x, L[0].y);
							

							circle(colorFrame, M[0], 2, Scalar(255, 255, 0), -1, 8, 0);
							circle(colorFrame, M[1], 2, Scalar(0, 255, 0), -1, 8, 0);
							circle(colorFrame, M[2], 2, Scalar(0, 0, 255), -1, 8, 0);
							circle(colorFrame, M[3], 2, Scalar(128, 128, 128), -1, 8, 0);

							circle(colorFrame, O[0], 2, Scalar(255, 255, 0), -1, 8, 0);
							circle(colorFrame, O[1], 2, Scalar(0, 255, 0), -1, 8, 0);
							circle(colorFrame, O[2], 2, Scalar(0, 0, 255), -1, 8, 0);
							circle(colorFrame, O[3], 2, Scalar(128, 128, 128), -1, 8, 0);

							// Draw point of the estimated 4th Corner of (entire) QR Code
							circle(colorFrame, N, 2, Scalar(255, 255, 255), -1, 8, 0);

							//printf("L0(%f,%f), L2(%f,%f)", L[0].x, L[0].y, L[2].x, L[2].y);

							//gray.at<uchar>(10, 200) = 255;
							//printf("height is  %d ,  weight is ", int(gray.at<uchar>(700,1200)));
							//printf("row range is  %d ", gray.weight );

							//int a = average_gray_scale(L[0], L[1], L[2], L[3], image);


							// Draw the lines used for estimating the 4th Corner of QR Code
							line(colorFrame, M[1], N, Scalar(0, 0, 255), 1, 8, 0);
							line(colorFrame, O[3], N, Scalar(0, 0, 255), 1, 8, 0);
						}

						// int a = 0; // this is a landmark 
						int codes[6];
						int info_of_this_qr[5];

						Mat testmat = qr_warper(L[0], M[1], N, O[3], colorFrame);

						position_finder(L[0], M[1], O[3], info_of_this_qr);

						code_digger(codes, testmat);

						info_of_this_qr[0] = decoder(codes);	

						list_insert(info_of_this_qr, allinfo);
					}
			}
		}
		//cv::resize(colorFrame, processedFrameMat, cv::Size(height, width), 0, 0, cv::INTER_CUBIC);
		colorFrame.copyTo(processedFrameMat);
		// this is the monitor window to show color image.
		//Mat monitor(200, 135, CV_8UC3);
		//cv::resize(colorFrame, monitor, cv::Size(200, 135), 0, 0, cv::INTER_CUBIC);
		Mat monitor;
		colorFrame.copyTo(monitor);
		imshow("monitor", monitor);
	}
}
