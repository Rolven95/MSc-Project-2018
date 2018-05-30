#pragma once

// This makes the export declarations for the library
//
// @author Daniel J. Finnegan
// @date April 2016
//

#define HANDTRACKERLIB_EXPORTS

#if defined(HANDTRACKERLIB_EXPORTS)
#define HANDTRACKER_API __declspec(dllexport)
#else
#define HANDTRACKER_API __declspec(dllimport)
#endif

// Forward declarations to save me some typing
namespace ar_sandbox
{
	class HANDTRACKER_API KinectManager;
	template <typename T> class HANDTRACKER_API  DepthFrameProcessor;
	class HANDTRACKER_API DepthFrameResizer;
	class HANDTRACKER_API HandTracker;
}

#include "KinectTypes.h"
#include "KinectManager.h"
#include "HandTracker.h"

