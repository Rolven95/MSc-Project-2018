#include "stdafx.h"
#include "HandTracker.h"

#ifndef MAX_KERNEL_LENGTH
#define MAX_KERNEL_LENGTH 5 // Play around with this to get better accuracy at the cost of runtime performance
#endif

//#define HAND_DEBUG true
#ifndef HAND_DEBUG
#define HAND_DEBUG false
#endif

#define SCALE_FACTOR 1.0/65535.0f // This scales the 16 bit depth pixel values between 1 and 0

/*
 * This is the hand tracker for the Sandbox project.
 * Note that it makes a few assumptions:
 * 1) Any object in the sensor's view that has a contour line greater
 *	  than a specified area threshold is determined to be a hand.
 * 2) The tracker fails if hands cross over one another
 * 3) The tracker also fails if detected contours are
	  self intersecting (see http://stackoverflow.com/questions/32683932/how-to-detect-contour-self-intersection-with-c-and-opencv)
 */
namespace ar_sandbox
{
	const float HandTracker::defectsAngleThreshold = 80;
	const float HandTracker::defectsDistanceThreshold = 100;

	HandTracker::HandTracker(cv::Size frameDimensions) 
		: DepthFrameProcessor(512, 512),  latestHandID(0)
	{
		// init the buffers
		int width = frameDimensions.width;
		int height = frameDimensions.height;

		// allocate buffers
		grayBuffer = boost::make_shared<float[]>(height * width); // 1 channel gray buffer gray buffer
		processedBuffer = boost::make_shared<unsigned char[]>(height * width * 3); // 3 Channel contour map buffer
		maxThresholdBuffer = boost::make_shared<float[]>(height * width); // Buffer for the thresholded data
		minThresholdBuffer = boost::make_shared<float[]>(height * width); // Buffer for the thresholded data
		minMaxANDThresholdBuffer = boost::make_shared<unsigned char[]>(height * width);

		// Set the Matrices
		processedFrameMat = cv::Mat(height, width, CV_8UC3, processedBuffer.get());
		maxThresholdMat = cv::Mat(height, width, CV_32FC1, maxThresholdBuffer.get());
		minThresholdMat = cv::Mat(height, width, CV_32FC1, minThresholdBuffer.get());
		minMaxANDThresholdMat = cv::Mat(height, width, CV_8UC1, minMaxANDThresholdBuffer.get());
		grayMat = cv::Mat(height, width, CV_32FC1, grayBuffer.get());
	}

	HandTracker::~HandTracker()
	{
	}

	void HandTracker::processFrame(cv::Mat & depthFrame)
	{
		if (depthFrame.data)
		{
			////////////////////////////
			// Start the fps counter ///
			////////////////////////////

			LARGE_INTEGER li;
			if (!QueryPerformanceFrequency(&li))
				std::cout << "QueryPerformanceFrequency failed!\n";

			PCFreq = double(li.QuadPart) / 1000.0;

			QueryPerformanceCounter(&li);
			CounterStart = li.QuadPart;

			////////////////////////////
			////////////////////////////

			// Process the depth frame first
			// This finds contours in the current frame
			processDepthFrame(depthFrame);

			// Track the centroids
			if (!HAND_DEBUG)
				detectHands();

			// Now we've tracked our centroids, so draw the contours
			drawContourMat();

			////////////////////////////
			////////////////////////////

			// Calculate the fps
			QueryPerformanceCounter(&li);
			fps = double(li.QuadPart - CounterStart) / PCFreq;
		}
	}

	void HandTracker::processDepthFrame(cv::Mat & depthFrame)
	{
		// Init all mats first
		maxThresholdMat = cv::Scalar(0);
		minThresholdMat = cv::Scalar(0);
		minMaxANDThresholdMat = cv::Scalar(0);
		grayMat = cv::Scalar(0);

		// Smooth the depth map
		for (int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2)
			cv::GaussianBlur(depthFrame, depthFrame, cv::Size(i, i), 0);

		//////////////////////////////////////////////////
		//////////////////////////////////////////////////
		//// Get the depth pixels only in the range //////
		//// between the min and max depth thresholds ////
		//// by XORing the matrices together after ///////
		//// thresholding ////////////////////////////////
		//////////////////////////////////////////////////
		//////////////////////////////////////////////////

		depthFrame.convertTo(grayMat, CV_32FC1, SCALE_FACTOR);
		cv::threshold(grayMat, maxThresholdMat, maxDepthThreshold * SCALE_FACTOR, 1, cv::THRESH_BINARY_INV);

		// Repeat the above for the maximum threshold value
		grayMat = cv::Scalar(0);
		depthFrame.convertTo(grayMat, CV_32FC1, SCALE_FACTOR);
		cv::threshold(grayMat, minThresholdMat, minDepthThreshold * SCALE_FACTOR, 1, cv::THRESH_BINARY_INV);

		// And them together
		cv::bitwise_xor(minThresholdMat, maxThresholdMat, maxThresholdMat);

		// Then convert them to our 8 bit matrix for contour processing
		maxThresholdMat.convertTo(minMaxANDThresholdMat, CV_8UC1);

		//////////////////////////////////////////////////
		//////////////////////////////////////////////////
		//////////////////////////////////////////////////
		//////////////////////////////////////////////////

		//////////////////////////////////////////////////
		//////////////////////////////////////////////////
		//// For the next bit just take contours that ////
		//// are big enough to make sense as a hand //////
		//////////////////////////////////////////////////
		//////////////////////////////////////////////////

		// Find the contours in the image
		if (!contours.empty())
		{
			std::vector<std::vector<cv::Point> >::iterator it;
			for (it = contours.begin(); it != contours.end(); ++it)
			{
				it->clear();
			}
			contours.clear(); // Clean up first!!

			contourHierarchy.clear();
		}
		cv::findContours(minMaxANDThresholdMat, contours, contourHierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

		if (HAND_DEBUG)
		{
			if (!contourIndices.empty())
				contourIndices.clear();
			for (int i = 0; i < contours.size(); ++i)
			{
				contourIndices.push_back(i);
			}
		}

		//////////////////////////////////////////////////
		//////////////////////////////////////////////////
		//////////////////////////////////////////////////
		//////////////////////////////////////////////////
	}

	double dist(cv::Point x, cv::Point y)
	{
		return (x.x - y.x)*(x.x - y.x) + (x.y - y.y)*(x.y - y.y);
	}

	std::pair<cv::Point, double> circleFromPoints(cv::Point p1, cv::Point p2, cv::Point p3)
	{
		double offset = pow(p2.x, 2) + pow(p2.y, 2);
		double bc = (pow(p1.x, 2) + pow(p1.y, 2) - offset) / 2.0;
		double cd = (offset - pow(p3.x, 2) - pow(p3.y, 2)) / 2.0;
		double det = (p1.x - p2.x) * (p2.y - p3.y) - (p2.x - p3.x)* (p1.y - p2.y);
		double TOL = 0.0000001;
		if (abs(det) < TOL) 
		{
			//std::cout << "POINTS TOO CLOSE" << std::endl; 
			return std::make_pair(cv::Point(0, 0), 0); 
		}

		double idet = 1 / det;
		double centerx = (bc * (p2.y - p3.y) - cd * (p1.y - p2.y)) * idet;
		double centery = (cd * (p1.x - p2.x) - bc * (p2.x - p3.x)) * idet;
		double radius = sqrt(pow(p2.x - centerx, 2) + pow(p2.y - centery, 2));

		return std::make_pair(cv::Point(centerx, centery), radius);
	}

	void HandTracker::detectHands()
	{
		// I'm taking contour regions that cover areas bigger than a certain threshold as valid
		if (!contourIndices.empty())
			contourIndices.clear();
		for (int i = 0; i < contours.size(); ++i)
		{
			// Compute the area
			double area = cv::contourArea(contours.at(i));

			// Only take closed contours too
			if (area > HandTracker::contourMinAreaThreshold)
				contourIndices.push_back(i);
		}

		// If we have no contours this scene, remove all tracked hands and start again
		if (contours.size() <= 0)
		{
			hands.erase(hands.begin(), hands.end());
		}
		else
		{
			// Compute the centroid set for this new frame
			std::vector<cv::Point> newCentroids(contourIndices.size());
			std::vector<ar_sandbox::HandState> newHandStates(contourIndices.size());

			// Loop through the contours in our hand set
			// and compute the centroids and the convex hull (hand state)
			for (int i = 0; i < contourIndices.size(); ++i)
			{
				std::vector<cv::Vec4i> defects;
				//std::vector<cv::Vec4i> filteredDefects;

				// Get the hand by computing convex defects and apply the
				// assumption and thresholds on certain data
				std::vector<int> hullPoints; // Use less space as we don't need the actual points themselves, just the count
				cv::convexHull(contours[contourIndices[i]], hullPoints, false, true);

				// Do the convexity defect check
				cv::convexityDefects(contours[contourIndices[i]], hullPoints, defects);
				std::vector<cv::Vec4i>::iterator defectsIt;
				//for (defectsIt = defects.begin(); defectsIt != defects.end(); ++defectsIt)
				//{
				//	cv::Vec4i v = *defectsIt;
				//	int startIndex = v[0];
				//	int endIndex = v[1];
				//	int farIndex = v[2];

				//	cv::Point startPoint = contours[contourIndices[i]][startIndex];
				//	cv::Point endPoint = contours[contourIndices[i]][endIndex];
				//	cv::Point farPoint = contours[contourIndices[i]][farIndex];

				//	// Compute distances
				//	float distStartFar = sqrtf(pow(startPoint.x - farPoint.x, 2) + pow(startPoint.y - farPoint.y, 2));
				//	float distEndFar = sqrtf(pow(endPoint.x - farPoint.x, 2) + pow(endPoint.y - farPoint.y, 2));

				//	// Get the angle in between using the dot product
				//	float dotProduct = (startPoint.x - farPoint.x) * (endPoint.x - farPoint.x) + (startPoint.y - farPoint.y) * (endPoint.y - farPoint.y);
				//	float angle = acos(dotProduct / (distStartFar * distEndFar));
				//	angle = angle * 180 / M_PI;

				//	if (distStartFar > HandTracker::defectsDistanceThreshold
				//		&& distEndFar > HandTracker::defectsDistanceThreshold
				//		&& angle < HandTracker::defectsAngleThreshold)
				//	{
				//		filteredDefects.push_back(v);
				//	}
				//}

				//if (filteredDefects.size() >= 1 && filteredDefects.size() <= 3)
				//	newHandStates[i] = HandState::HAND_OPEN;
				//else
				//	newHandStates[i] = HandState::HAND_CLOSED;

				//// Do the convexity defect check
				//std::vector<cv::Vec4i> defects;
				//std::vector<cv::Point> fingerTips;
				//cv::convexityDefects(contours[contourIndices[i]], hullPoints, defects);
				//std::vector<cv::Vec4i>::iterator defectsIt;

				//// sentinel
				//defectsIt = defects.begin();
				//cv::Vec4i v = *defectsIt;
				//int startIndex = v[0];
				//int endIndex = v[1];

				//cv::Point startPoint = contours[contourIndices[i]][startIndex];
				//cv::Point endPoint = contours[contourIndices[i]][endIndex];

				//fingerTips.push_back(startPoint);
				//for (;  defectsIt != defects.end(); ++defectsIt) // Not very often you get to do a while loop like this. Love it!
				//{
				//	endIndex = v[1];
				//	endPoint = contours[contourIndices[i]][endIndex];
				//	fingerTips.push_back(endPoint);
				//}

				//if (fingerTips.size() >= 3 && fingerTips.size() <= 7)
				//	newHandStates[i] = HandState::HAND_OPEN;
				//else
				//	newHandStates[i] = HandState::HAND_CLOSED;


				//
				// This working software for palm detection has been shamelessly copied from here:
				// https://github.com/jujojujo2003/OpenCVHandGuesture/blob/master/main.cpp
				//
				// Article here:
				// https://s-ln.in/2013/04/18/hand-tracking-and-gesture-detection-opencv/
				//

				cv::Point rough_palm_center;
				if (defects.size() >= 3)
				{
					std::vector<cv::Point> palm_points;
					for (int j = 0; j < defects.size(); j++)
					{
						int startidx = defects[j][0]; cv::Point ptStart(contours[contourIndices[i]][startidx]);
						int endidx = defects[j][1]; cv::Point ptEnd(contours[contourIndices[i]][endidx]);
						int faridx = defects[j][2]; cv::Point ptFar(contours[contourIndices[i]][faridx]);
						//Sum up all the hull and defect points to compute average
						rough_palm_center += ptFar + ptStart + ptEnd;
						palm_points.push_back(ptFar);
						palm_points.push_back(ptStart);
						palm_points.push_back(ptEnd);
					}

					//Get palm center by 1st getting the average of all defect points, this is the rough palm center,
					//Then U chose the closest 3 points ang get the circle radius and center formed from them which is the palm center.
					rough_palm_center.x /= defects.size() * 3;
					rough_palm_center.y /= defects.size() * 3;
					cv::Point closest_pt = palm_points[0];
					std::vector<std::pair<double, int> > distvec;
					for (int j = 0; j < palm_points.size(); j++)
						distvec.push_back(std::make_pair(dist(rough_palm_center, palm_points[j]), j));
					sort(distvec.begin(), distvec.end());

					//Keep choosing 3 points till you find a circle with a valid radius
					//As there is a high chance that the closes points might be in a linear line or too close that it forms a very large circle
					std::pair<cv::Point, double> soln_circle;
					for (int j = 0; j + 2 < distvec.size(); j++)
					{
						cv::Point p1 = palm_points[distvec[j + 0].second];
						cv::Point p2 = palm_points[distvec[j + 1].second];
						cv::Point p3 = palm_points[distvec[j + 2].second];
						soln_circle = circleFromPoints(p1, p2, p3);//Final palm center,radius
						if (soln_circle.second != 0)
							break;
					}

					//Find avg palm centers for the last few frames to stabilize its centers, also find the avg radius
					palm_centers.push_back(soln_circle);
					if (palm_centers.size() > 10)
						palm_centers.erase(palm_centers.begin());

					cv::Point palm_center;
					double radius = 0;
					for (int j = 0; j < palm_centers.size(); j++)
					{
						palm_center += palm_centers[j].first;
						radius += palm_centers[j].second;
					}
					palm_center.x /= palm_centers.size();
					palm_center.y /= palm_centers.size();
					radius /= palm_centers.size();

					newCentroids[i] = palm_center;

					//Draw the palm center and the palm circle
					//The size of the palm gives the depth of the hand
					//circle(frame, palm_center, 5, Scalar(144, 144, 255), 3);
					//circle(frame, palm_center, radius, Scalar(144, 144, 255), 2);

					//Detect fingers by finding points that form an almost isosceles triangle with certain thesholds
					int no_of_fingers = 0;
					for (int j = 0; j < defects.size(); j++)
					{
						int startidx = defects[j][0]; cv::Point ptStart(contours[contourIndices[i]][startidx]);
						int endidx = defects[j][1]; cv::Point ptEnd(contours[contourIndices[i]][endidx]);
						int faridx = defects[j][2]; cv::Point ptFar(contours[contourIndices[i]][faridx]);
						//X o--------------------------o Y
						double Xdist = sqrt(dist(palm_center, ptFar));
						double Ydist = sqrt(dist(palm_center, ptStart));
						double length = sqrt(dist(ptFar, ptStart));

						double retLength = sqrt(dist(ptEnd, ptFar));
						//Play with these thresholds to improve performance
						if (length <= 3 * radius&&Ydist >= 0.4*radius&&length >= 10 && retLength >= 10 && std::max(length, retLength) / std::min(length, retLength) >= 0.8)
							if (std::min(Xdist, Ydist) / std::max(Xdist, Ydist) <= 0.8)
							{
								if ((Xdist >= 0.1*radius&&Xdist <= 1.3*radius&&Xdist<Ydist) || (Ydist >= 0.1*radius&&Ydist <= 1.3*radius&&Xdist>Ydist))
									//line(frame, ptEnd, ptFar, Scalar(0, 255, 0), 1), no_of_fingers++;
									no_of_fingers++;
							}


					}

					no_of_fingers = std::min(5, no_of_fingers);
					//std::cout << "NO OF FINGERS: " << no_of_fingers << std::endl;

					if (no_of_fingers >= 1 && no_of_fingers <= 5)
						newHandStates[i] = HandState::HAND_OPEN;
					else
						newHandStates[i] = HandState::HAND_CLOSED;

				}













				// Centroids
				//cv::Moments mu = cv::moments(contours[contourIndices[i]], false);
				//newCentroids[i] = cv::Point((int)mu.m10 / mu.m00, (int)mu.m01 / mu.m00);
				/*newCentroids[i] = rough_palm_center;*/
			}

			if (hands.size() <= 0)
			{
				// We have none, so just take all the new ones
				for (int i = 0; i < newCentroids.size(); ++i)
				{
					cv::Point newC = newCentroids[i];
					HandState newState = newHandStates[i];
					Hand newHand = Hand::makeHand(newC, latestHandID++, newState);
					hands.push_back(newHand);
				}
			}
			else
			{
				if (newCentroids.size() < hands.size())
				{
					// If the new set size is smaller than the old set size, we've lost some
					std::vector<Hand>::iterator it;
					for (it = hands.begin(); it != hands.end();)
					{
						bool foundMatch = false;
						Hand oldHand = *it;
						cv::Point oldCentroid = oldHand.getCenterOfMass();
						for (int j = 0; j < newCentroids.size(); ++j)
						{
							cv::Point newC = newCentroids[j];
							float dist = sqrtf(
								pow(newC.x - oldCentroid.x, 2) +
								pow(newC.y - oldCentroid.y, 2)
								);
							if (dist < HandTracker::interFrameCentroidDistance)
							{
								foundMatch = true;
								it->setCenterOfMass(newC); // Just update the centroid for this hand
								it->setHandState(newHandStates[j]);
								break;
							}
						}

						// Lost at sea
						if (!foundMatch)
						{
							it = hands.erase(it);
							if (latestHandID >= 0)
								latestHandID--;
						}
						else
							++it;
					}
				}
				else
				{
					// Otherwise, we update the old centroid set
					for (int i = 0; i < newCentroids.size(); ++i)
					{
						bool foundMatch = false;
						cv::Point newC = newCentroids[i];

						std::vector<Hand>::iterator it;
						for (it = hands.begin(); it != hands.end(); ++it)
						{
							Hand oldHand = *it;
							cv::Point oldCentroid = oldHand.getCenterOfMass();
							float dist = sqrtf(
								pow(newC.x - oldCentroid.x, 2) +
								pow(newC.y - oldCentroid.y, 2)
								);
							if (dist < HandTracker::interFrameCentroidDistance)
							{
								foundMatch = true;
								it->setCenterOfMass(newC);
								it->setHandState(newHandStates[i]);
								break;
							}
						}

						if (!foundMatch) // If we couldn't match it, then she must be a new one
						{
							// Find the next unique ID available
							int ID = -1;
							std::vector<Hand>::iterator it;
							for (it = hands.begin(); it != hands.end(); ++it)
							{
								if (ID < it->getID())
									ID = it->getID();
							}

							Hand newHand = Hand::makeHand(newC, ID + 1, newHandStates[i]);
							hands.push_back(newHand);
						}
					}
				}
			}
		}
	}

	void HandTracker::drawContourMat()
	{
		processedFrameMat = cv::Scalar(0, 0, 0);
		for (int i = 0; i < contourIndices.size(); ++i)
		{
			/*cv::Scalar color = cv::Scalar(rand() % 255,
				rand() % 255,
				rand() % 255);*/
			cv::Scalar color = cv::Scalar(0, 0, 255);
			cv::drawContours(processedFrameMat, contours, contourIndices[i], color, 1, 8, contourHierarchy);
		}

		// Then draw the hand information
		if (!HAND_DEBUG)
		{
			std::vector<Hand>::iterator it;
			for (it = hands.begin(); it != hands.end(); ++it)
			{
				cv::Scalar color = cv::Scalar(0, 0, 255);

				Hand currentHand = *it;
				cv::Point centroid = currentHand.getCenterOfMass();
				std::string handState = currentHand.getHandState() == HandState::HAND_OPEN ? "open" : "closed";

				// Create the meta text
				std::ostringstream centroidStream;
				centroidStream << "Centroid ID: " << currentHand.getID();
				std::string centroidStr = centroidStream.str();

				std::ostringstream stateStream;
				stateStream << "Hand State: " << handState;
				std::string stateStr = stateStream.str();
				cv::Point statePoint = cv::Point(centroid.x, centroid.y - 20);

				/**
				  * Dan: May 30th 2018
				  *
				  * The compiler didn't seem to like these lines.
				  * Perhaps the latest version of OpenCV has removed the HersheyFonts library?
				  * This will need investigation
				  */
				//std::cout << handStr << std::endl;
				// cv::putText(processedFrameMat, centroidStr, centroid,
				// 	cv::HersheyFonts::FONT_HERSHEY_COMPLEX, 0.5, color);
				// cv::putText(processedFrameMat, stateStr, statePoint,
				// 	cv::HersheyFonts::FONT_HERSHEY_COMPLEX, 0.5, color);
			}
		}
	}
}

