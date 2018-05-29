#include "include/qrtracker.hpp"

QRTracker::QRTracker(int a, int b) :
anum(a), bnum(b)
{}

QRTracker::~QRTracker()
{}

int QRTracker::computeNumber()
{
	return anum * bnum;
}