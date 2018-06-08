//______________________________________________________________________________________
// Program : OpenCV based QR code Detection and Retrieval
// Author  : Bharath Prabhuswamy
//______________________________________________________________________________________
#define idnumber 30
#define RESIZE_WIDTH 1280
#define RESIZE_HEIGHT 720

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include "KinectManager.h"
#include <omp.h>

// define a bool to switch camera
bool switch2Kinect = 0;

using namespace cv;
using namespace std;

using SensorManager = boost::shared_ptr<ar_sandbox::KinectManager>;
using ColorResizer = boost::shared_ptr<ar_sandbox::ColorFrameResizer>;
static boost::shared_ptr<BYTE[]> colorFrameBuffer;

const int CV_QR_NORTH = 0;
const int CV_QR_EAST = 1;
const int CV_QR_SOUTH = 2;
const int CV_QR_WEST = 3;
int average_gray_scale(Point2f N, Point2f E, Point2f S, Point2f W, Mat graph);
float cv_returnX(Point2f X);
float cv_returnY(Point2f Y);
float cv_distance(Point2f P, Point2f Q);					// Get Distance between two points
float cv_lineEquation(Point2f L, Point2f M, Point2f J);		// Perpendicular Distance of a Point J from line formed by Points L and M; Solution to equation of the line Val = ax+by+c 
float cv_lineSlope(Point2f L, Point2f M, int& alignement);	// Slope of a line by two Points L and M on it; Slope of line, S = (x1 -x2) / (y1- y2)
void cv_getVertices(vector<vector<Point> > contours, int c_id,float slope, vector<Point2f>& X);
void cv_updateCorner(Point2f P, Point2f ref ,float& baseline,  Point2f& corner);
void cv_updateCornerOr(int orientation, vector<Point2f> _IN, vector<Point2f> & _OUT);
bool getIntersectionPoint(Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& intersection);
float cross(Point2f v1,Point2f v2);
// void qrdetection(int ilist[50],int qrlist[15][3]);
// Start of Main Loop
//------------------------------------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
		////////Test area

	//int s = NULL;
	//if (s == NULL)
	//	printf("s is null");
	//s = 1;
	//if (s != NULL)
	//	printf("s is not null");
	//float A = 2.777;
	//int B = A;
	//printf("in main");

		/////////////////////

		BYTE *colorData = nullptr;
		int *framelength = nullptr;

		Mat image;

		// Objects we need to interface with the Kinect
		SensorManager kinectManager = boost::make_shared<ar_sandbox::KinectManager>();
		ColorResizer colorResizer = boost::make_shared<ar_sandbox::ColorFrameResizer>();

		// initialize the variables we care about
		colorResizer->setResizeParameters(RESIZE_WIDTH, RESIZE_HEIGHT);
		colorFrameBuffer = boost::make_shared<BYTE[]>(colorResizer->getSizeParameters().width * colorResizer->getSizeParameters().height * 4); // RGBA data	

		// This is for Webcamera define
		VideoCapture capture(0);

		if (switch2Kinect)
		{
			// Starts kinect
			kinectManager->initSensor();
		}
		else
		{
			if(!capture.isOpened()) { cerr << " ERR: Unable find input Video source." << endl;
				return -1;
			}

			//Step	: Capture a frame from Image Input for creating and initializing manipulation variables
			//Info	: Inbuilt functions from OpenCV
			//Note	: 
			
				capture >> image;
			if(image.empty()){ cerr << "ERR: Unable to query image from capture device.\n" << endl;
				return -1;
			}			
		}

	// Creation of Intermediate 'Image' Objects required later
	Mat gray(image.size(), CV_MAKETYPE(image.depth(), 1));			// To hold Grayscale Image
	Mat edges(image.size(), CV_MAKETYPE(image.depth(), 1));			// To hold Grayscale Image

	Mat traces(RESIZE_WIDTH, RESIZE_WIDTH,CV_8UC3);								// For Debug Visuals
	Mat qr,qr_raw,qr_gray,qr_thres;
	    
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Point> pointsseq;    //used to save the approximated sides of each contour

	int mark, mapcounter;

	int DBG=1;						// Debug Flag
	int framenumber = 0; //debug frame number
	int key = 0;
	while(1)				// While loop to query for Image Input frame
	{
		framenumber++;
		printf("\n %d  ", framenumber);
		traces = Scalar(0,0,0);
		qr_raw = Mat::zeros(RESIZE_WIDTH, RESIZE_HEIGHT, CV_8UC3 );
	   	qr = Mat::zeros(RESIZE_WIDTH, RESIZE_HEIGHT, CV_8UC3 );
		qr_gray = Mat::zeros(RESIZE_WIDTH, RESIZE_HEIGHT, CV_8UC1);
	   	qr_thres = Mat::zeros(100, 100, CV_8UC1);

		if (switch2Kinect)
		{
			// Get the next frame from the Kinect
			do
			{
				kinectManager->readMultiFrame();
			} while (kinectManager->getDepthDimensions().width <= 0);


			// Process the color from the Kinect
			cv::Mat colorFrame = kinectManager->getColorMat();
			colorResizer->processFrame(colorFrame);

			image = cv::Mat(RESIZE_WIDTH, RESIZE_HEIGHT, CV_8UC4, colorFrameBuffer.get());
			//cv::Mat image = cv::Mat(RESIZE_WIDTH, RESIZE_HEIGHT, CV_8UC4, colorFrameBuffer.get());
			colorResizer->copyFrameBuffer(image);
		}
		else
		{
			capture >> image;						// Capture Image from Image Input
		}
		// From here, you can do all your QR processing on image

		cvtColor(image, gray, CV_RGB2GRAY);		// Convert Image captured from Image Input to GrayScale	


		//uchar pixel_value = gray.ptr<uchar>(10)[10];
		//printf(" point gray is: %d ", pixel_value);

		Canny(gray, edges, 100, 200, 3);		// Apply Canny edge detection on the gray image

		findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE); // Find contours with hierarchy

		mark = 0;								// Reset all detected marker count for this frame
		mapcounter = 0;
		// Get Moments for all Contours and the mass centers
		vector<Moments> mu(contours.size());
		vector<Point2f> mc(contours.size());
//#pragma omp parallel for  
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
//#pragma omp parallel for  
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
			else if (pointsseq.size() == 3) {
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
					mappoint[mapcounter] = i;
					mapcounter++;
					//printf(" Find a map points ");
				}
			}
		}
		if (mapcounter == 3) {
			printf("Find all map points");
			//printf(" 1. %f,%f 2. %f,%f 3. %f,%f", cv_returnX(mc[mappoint[0]]), cv_returnY(mc[mappoint[0]]), cv_returnX(mc[mappoint[1]]), cv_returnY(mc[mappoint[1]]), cv_returnX(mc[mappoint[2]]), cv_returnY(mc[mappoint[2]]));

		}

		float alldistance[idnumber][idnumber]; //theorical maxium size
		float smalldistance = 9999;

//#pragma omp parallel for  
		for (int i = 0; i < mark; i++)  //actrual reading
		{
			for (int q = 0; q < mark; q++)
			{
				alldistance[i][q] = cv_distance(mc[id[i]], mc[id[q]]);
				if (alldistance[i][q] >0 && alldistance[i][q] < smalldistance)
					smalldistance = alldistance[i][q];
				//printf(" %d%d = %f ", i, q, alldistance[i][q]);
			}
		}
		//printf(" small dis = %f ", smalldistance);

		int qrcounter = 0; //how many qr codes, should be mark/3
//#pragma omp parallel for  		
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

				alldistance[i][0] = -1;
				alldistance[min1][0] = -1;
				alldistance[min2][0] = -1;

				qrcounter++;
			}
		}

		if (qrcounter > 0)		// Ensure we have (atleast 3; namely A,B,C) 'Alignment Markers' discovered
		{
//#pragma omp parallel for  
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

					vector<Point2f> src, dst;		// src - Source Points basically the 4 end co-ordinates of the overlay image
													// dst - Destination Points to transform overlay image	

					Mat warp_matrix;

					cv_getVertices(contours, top, slope, tempL);
					cv_getVertices(contours, right, slope, tempM);
					cv_getVertices(contours, bottom, slope, tempO);

					cv_updateCornerOr(orientation, tempL, L); 			// Re-arrange marker corners w.r.t orientation of the QR code
					cv_updateCornerOr(orientation, tempM, M); 			// Re-arrange marker corners w.r.t orientation of the QR code
					cv_updateCornerOr(orientation, tempO, O); 			// Re-arrange marker corners w.r.t orientation of the QR code

					int iflag = getIntersectionPoint(M[1], M[2], O[3], O[2], N);

					threshold(qr_gray, qr_thres, 127, 255, CV_THRESH_BINARY);
					src.push_back(L[0]);
					src.push_back(M[1]);
					src.push_back(N);
					src.push_back(O[3]);



					//float test_grey = cvGetReal2D(gray, i, j);
					//qr_gray = Mat::zeros(RESIZE_WIDTH, RESIZE_HEIGHT, CV_8UC1);
					//int number = gray
					//printf(" kkkkk£º%d ",number);


					/*dst.push_back(Point2f(0, 0));
					dst.push_back(Point2f(qr.cols, 0));
					dst.push_back(Point2f(qr.cols, qr.rows));
					dst.push_back(Point2f(0, qr.rows));*/

					//if (src.size() == 4 && dst.size() == 4)			// Failsafe for WarpMatrix Calculation to have only 4 Points with src and dst
					//{
					//	warp_matrix = getPerspectiveTransform(src, dst);
					//	warpPerspective(image, qr_raw, warp_matrix, Size(qr.cols, qr.rows));
					//	copyMakeBorder(qr_raw, qr, 10, 10, 10, 10, BORDER_CONSTANT, Scalar(255, 255, 255));

					//	cvtColor(qr, qr_gray, CV_RGB2GRAY);
					//	threshold(qr_gray, qr_thres, 127, 255, CV_THRESH_BINARY);

					//	//threshold(qr_gray, qr_thres, 0, 255, CV_THRESH_OTSU);
					//	//for( int d=0 ; d < 4 ; d++){	src.pop_back(); dst.pop_back(); }
					//}

					//Draw contours on the image
					drawContours(image, contours, top, Scalar(255, 200, 0), 2, 8, hierarchy, 0);
					drawContours(image, contours, right, Scalar(0, 0, 255), 2, 8, hierarchy, 0);
					drawContours(image, contours, bottom, Scalar(255, 0, 100), 2, 8, hierarchy, 0);

					// Insert Debug instructions here
					if (DBG == 1)
					{
						// Debug Prints
						// Visualizations for ease of understanding
						if (slope > 5)
							circle(traces, Point(10, 20), 5, Scalar(0, 0, 255), -1, 8, 0);
						else if (slope < -5)
							circle(traces, Point(10, 20), 5, Scalar(255, 255, 255), -1, 8, 0);

						// Draw contours on Trace image for analysis	
						drawContours(traces, contours, top, Scalar(255, 0, 100), 1, 8, hierarchy, 0);
						drawContours(traces, contours, right, Scalar(255, 0, 100), 1, 8, hierarchy, 0);
						drawContours(traces, contours, bottom, Scalar(255, 0, 100), 1, 8, hierarchy, 0);

						// Draw points (4 corners) on Trace image for each Identification marker	
						circle(traces, L[0], 2, Scalar(255, 255, 0), -1, 8, 0);
						circle(traces, L[1], 2, Scalar(0, 255, 0), -1, 8, 0);
						circle(traces, L[2], 2, Scalar(0, 0, 255), -1, 8, 0);
						circle(traces, L[3], 2, Scalar(128, 128, 128), -1, 8, 0);

						circle(traces, M[0], 2, Scalar(255, 255, 0), -1, 8, 0);
						circle(traces, M[1], 2, Scalar(0, 255, 0), -1, 8, 0);
						circle(traces, M[2], 2, Scalar(0, 0, 255), -1, 8, 0);
						circle(traces, M[3], 2, Scalar(128, 128, 128), -1, 8, 0);

						circle(traces, O[0], 2, Scalar(255, 255, 0), -1, 8, 0);
						circle(traces, O[1], 2, Scalar(0, 255, 0), -1, 8, 0);
						circle(traces, O[2], 2, Scalar(0, 0, 255), -1, 8, 0);
						circle(traces, O[3], 2, Scalar(128, 128, 128), -1, 8, 0);

						// Draw point of the estimated 4th Corner of (entire) QR Code
						circle(traces, N, 2, Scalar(255, 255, 255), -1, 8, 0);

						//printf("L0(%f,%f), L2(%f,%f)", L[0].x, L[0].y, L[2].x, L[2].y);
						int a = average_gray_scale(L[0], L[1], L[2], L[3], gray);


						// Draw the lines used for estimating the 4th Corner of QR Code
						line(traces, M[1], N, Scalar(0, 0, 255), 1, 8, 0);
						line(traces, O[3], N, Scalar(0, 0, 255), 1, 8, 0);


						// Debug Prints
					}

				}
			}
		}

		imshow("Image", image);
		imshow("Traces", traces);
		//imshow ( "QR code", qr_thres );

		key = waitKey(1);	// OPENCV: wait for 1ms before accessing next frame

	}	// End of 'while' loop

	// Always clean up because we're _IN C++ land!
	colorResizer.reset();
	kinectManager.reset();

	return 0;
}

// End of Main Loop
//--------------------------------------------------------------------------------------


// Routines used _IN Main loops

// Function: Routine to get Distance between two points
// Description: Given 2 points, the function returns the distance


int average_gray_scale(Point2f N, Point2f E, Point2f S, Point2f W, Mat graph) {
			   
	Mat useless = Mat::zeros(RESIZE_WIDTH, RESIZE_HEIGHT, CV_8UC1);
	useless = graph;
	int edgepoints[2][2] = { N.x, N.x, N.y, N.y };//min, max, min, max
	int points[4][2];
	float lines[4][2];

	points[0][0] = N.x;
	points[0][1] = N.y;

	points[1][0] = E.x;
	points[1][1] = E.y;

	points[2][0] = S.x;
	points[2][1] = S.y;

	points[3][0] = W.x;
	points[3][1] = W.y;

	for (int i = 0; i < 4; i++) {
	 //find minx 
		if (points[i][0] < edgepoints[0][0])
			edgepoints[0][0] = points[i][0];
     //fin maxx
		if (points[i][0] > edgepoints[0][1])
			edgepoints[0][1] = points[i][0];
	//find miny
		if (points[i][1] < edgepoints[1][0])
			edgepoints[1][0] = points[i][1];
	//find maxy
		if (points[i][1] > edgepoints[1][1])
			edgepoints[1][1] = points[i][1];
	}
	//printf("   (%d,%d),(%d,%d),(%d,%d),(%d,%d)  ", points[0][0], points[0][1], points[1][0], points[1][1], points[2][0], points[2][1], points[3][0], points[3][1]);
	//printf("\n xxxxxxxx: %d %d  yyyyyy: %d %d \n", edgepoints[0][0], edgepoints[0][1], edgepoints[1][0], edgepoints[1][1]);

	lines[0][0] = (E.y-N.y)/(E.x - N.x);
	lines[0][1] = E.y - (lines[0][0]*E.x);
	
	lines[1][0] = (S.y - E.y) / (S.x - E.x);
	lines[1][1] = S.y - (lines[1][0]*S.x);

	lines[2][0] = (W.y - S.y) / (W.x - S.x);
	lines[2][1] = W.y - (lines[2][0]*W.x);

	lines[3][0] = (N.y - W.y) / (N.x - W.x);
	lines[3][1] = N.y - (lines[3][0]*N.x);

	int up, down, left, right; 
	int pk=NULL, nk=NULL; // just line number, not k or b
	for (int i = 0; i < 4; i++) {
		printf("number:%d",i);
		//printf(" \n line %d: y=%fx+%f ", i, lines[i][0], lines[i][1]);

		//positive k, nagetive k
		if (lines[i][0] >= 0) { // k>=0
			if (pk == NULL) {
				//printf("  pk is null  ");
				pk =i;
				//printf(" pk = %f ", pk);
			}
			else { // already found an positive K, now compare b
				if (lines[pk][1] >= lines[i][1]) {
					up = pk;
					down = i;
				}
				else {
					up = i;
					down = pk;
				}
			}
		}
		else if(lines[i][0]<0) {
		 // k<0
			if (nk == NULL) {
				nk = i;
				printf(" nk = %d ",nk);
			}
			else {
				if (lines[nk][1] >= lines[i][1]) {
					right = nk;
					left = i;
					printf("find all, right=%d, left=%d", right, left);
				}
				else {
					right = i;
					left = nk;
					printf("find all, right=%d, left=%d", right, left);
				}
			}
		}
	}
	printf(" up:%d, down:%d, left:%d,right:%d ",up,down,left,right);
	//printf(" lines[0][0]= %f, lines[1][0]= %f, lines[2][0]= %f, lines[3][0]= %f, lines[0][1]= %f, lines[1][1]= %f, lines[2][1]= %f, lines[3][1] = %f ", lines[0][0], lines[1][0], lines[2][0], lines[3][0], lines[0][1], lines[1][1], lines[2][1], lines[3][1]);
	int total_gray_scale = 0; 
	int total_points = 0;
	//for (int scanx = edgepoints[0][0]; scanx <= edgepoints[0][1]; scanx++) {
	//	for (int scany = edgepoints[1][0]; scany <= edgepoints[1][1]; scany++) {
	//		if ( scany>=(lines[left][0]*scanx+lines[left][1]) && scany >= (lines[down][0] * scanx + lines[down][1]) && scany <= (lines[right][0] * scanx + lines[right][1]) && scany <= (lines[up][0] * scanx + lines[up][1])) {
	//			total_gray_scale= total_gray_scale + useless.ptr<uchar>(scanx)[scany];
	//			total_points = total_points + 1; 
	//		}
	//		//printf("%d ", useless.ptr<uchar>(scanx)[scany]);
	//	}
	//	//printf("\n");
	//}
     // printf("\n total gray: %d", total_gray_scale);
	//printf("For (10,10):%d", useless.ptr<uchar>(10)[10]);
	

	return 0;
}

float cv_returnX(Point2f X) {
	float number = X.x;
	return number;
}
float cv_returnY(Point2f Y) {

	return Y.y;
}

float cv_distance(Point2f P, Point2f Q)
{
	return sqrt(pow(abs(P.x - Q.x), 2) + pow(abs(P.y - Q.y), 2));
}


// Function: Perpendicular Distance of a Point J from line formed by Points L and M; Equation of the line ax+by+c=0
// Description: Given 3 points, the function derives the line quation of the first two points,
//	  calculates and returns the perpendicular distance of the the 3rd point from this line.

float cv_lineEquation(Point2f L, Point2f M, Point2f J)
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

float cv_lineSlope(Point2f L, Point2f M, int& alignement)
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
void cv_getVertices(vector<vector<Point> > contours, int c_id, float slope, vector<Point2f>& quad)
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
void cv_updateCorner(Point2f P, Point2f ref, float& baseline, Point2f& corner)
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
void cv_updateCornerOr(int orientation, vector<Point2f> _IN, vector<Point2f> &_OUT)
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
bool getIntersectionPoint(Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& intersection)
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

float cross(Point2f v1, Point2f v2)
{
	return v1.x*v2.y - v1.y*v2.x;
}
