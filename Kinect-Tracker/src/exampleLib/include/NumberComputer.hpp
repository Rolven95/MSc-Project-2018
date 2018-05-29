#ifndef QR_TRACKER
#define QR_TRACKER

#ifdef WIN32
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT
#endif

class DLLEXPORT NumberComputer
{
public:
	NumberComputer(int a, int b);
	~NumberComputer();

	int computeNumber();

private:
	int anum;
	int bnum;
};

#endif