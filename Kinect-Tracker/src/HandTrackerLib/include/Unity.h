#pragma once

// This header specifies the API for Unity to call
// the handtracker code
//
// The accompanying source file instantiates static variables that the library can then call upon
// The approach I'm taking is to instantiate some variables here at the C++ level,
// then write a procedure style API where I call functions on the variables.
// I'll present pointers to the raw OpenCV matrices so the Unity layer can use them
//
// @author Daniel J. Finnegan
// @date April 2016
//

// The headers from the hand tracking library
#include "HandTrackerLib.h"

//
// @author Daniel J. Finnegan
// @date April 2016
//
extern "C"
{
	// Environment management
	void HANDTRACKER_API initEnv();
	void HANDTRACKER_API destroyEnv();

	// Update functions
	void HANDTRACKER_API updateSensor();
	void HANDTRACKER_API updateProcessor();
	void HANDTRACKER_API updateHandTracker();

	// Modifiable functions
	void HANDTRACKER_API setResizeProcessorParams(int width, int height);

	// Getters
	bool HANDTRACKER_API getDepthFrame(unsigned short **interOpPtr, int *frameLength);
	bool HANDTRACKER_API getColorFrame(BYTE **interOpPtr, int *frameLength);
	bool HANDTRACKER_API getContourFrame(BYTE **interOpPtr, int *frameLength);
	bool HANDTRACKER_API getQRTraceFrame(BYTE **interOpPtr, int *frameLength);
	bool HANDTRACKER_API getQRResult(int (*qrresult)[30][6]);
	bool HANDTRACKER_API testFun(int (*a)[10]);
}
