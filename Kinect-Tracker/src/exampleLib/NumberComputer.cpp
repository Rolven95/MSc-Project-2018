#include "include/NumberComputer.hpp"

NumberComputer::NumberComputer(int a, int b) :
anum(a), bnum(b)
{}

NumberComputer::~NumberComputer()
{}

int NumberComputer::computeNumber()
{
	return anum * bnum;
}