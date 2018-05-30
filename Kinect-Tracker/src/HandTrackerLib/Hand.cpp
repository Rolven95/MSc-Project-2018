#include "stdafx.h"
#include "Hand.h"

namespace ar_sandbox
{
	// This is the only way to instantiate
	// a hand as the constructor is private
	Hand Hand::makeHand(cv::Point massCenter, int numID, HandState handState)
	{
		Hand newHand = Hand(massCenter);
		newHand.ID = numID;
		newHand.state = handState;
		return newHand;
	}

	Hand::Hand(cv::Point point)
	{
		massCenter = point;
	}

	Hand::~Hand()
	{
	}
}

