// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files:
#include <windows.h>

// OpenCV magic
#include <opencv2\imgproc\imgproc.hpp> // For image transformations in OpenCV
#include <opencv2\features2d\features2d.hpp> // For the hand tracking features

// Boost magic
#include <boost\smart_ptr\make_shared.hpp>
#include <boost\foreach.hpp> // Because foreach iteration kicks ass!!

// std library stuff
#include <iostream>
#include <stdlib.h> // For random number generation in the coloring of contours

// All this just for PI....
#define _USE_MATH_DEFINES
#include <math.h>

// The headers from the hand tracking library
#include "HandTrackerLib.h"
