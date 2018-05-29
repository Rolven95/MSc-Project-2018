#ifndef QR_TRACKER
#define QR_TRACKER

#ifdef WIN32
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT
#endif

class DLLEXPORT QRTracker
{
public:
	QRTracker(int a, int b);
	~QRTracker();

	int computeNumber();

private:
	int anum;
	int bnum;
};

#endif