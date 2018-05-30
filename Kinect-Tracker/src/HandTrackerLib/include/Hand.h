#pragma once

// OpenCV types etc
#include <opencv2\core\core.hpp>

namespace ar_sandbox
{
	// Representation of a hand
	// A hand (for now) has an ID and an
	// XY pixel coordinate

	struct HandKeyHasher; // Forward declaration for friend access

	// A hand will be detected as open or closed
	enum HandState {
		HAND_OPEN,
		HAND_CLOSED
	};

	class Hand
	{
	public:

		// The HandKeyHasher is a friend of the hand class
		friend ar_sandbox::HandKeyHasher;
		~Hand();

		// Public functions 
		static Hand makeHand(cv::Point massCenterPoint, int numID, HandState state);
		int getID() { return ID; }

		cv::Point2f getCenterOfMass() { return massCenter; }
		void setCenterOfMass(cv::Point center) { massCenter = center; }

		HandState getHandState() { return state; }
		void setHandState(HandState s) { state = s; }

		// Overloaded == operator in order to use the hand as a map key
		bool operator== (const Hand& other) const
		{
			return massCenter.x == other.massCenter.x
				&& massCenter.y == other.massCenter.y;
		}

	private:
		Hand(cv::Point center); // private constructor

		HandState state;
		cv::Point2f massCenter;
		int ID;

		static int s_ID;
	};

	// This computes a hash function so that the hand
	// class can be used in a boost unordered_map as a key
	// Not used in the end as vectors are used rather than hands
	struct HandKeyHasher
	{
		// Very rudimentary hash function: just XORs the center and the ID
		std::size_t operator()(const Hand& h) const
		{
			return ((std::hash<float>()(h.massCenter.x)
				^ (std::hash<float>()(h.massCenter.y))
				^ (std::hash<int>()(h.ID))));
		}
	};
}


