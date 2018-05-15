#ifndef KNIGHTVISION_KNIGHTCAMERA_H_
#define KNIGHTVISION_KNIGHTCAMERA_H_

#include "opencv2/opencv.hpp"
#include <thread>
#include <mutex>

using namespace cv;
using namespace std;

class KnightCamera {
public:
	KnightCamera(VideoCapture *cap);
	void start();
	void stop();
	Mat getLatestFrame();
private:
	VideoCapture *cap;
	bool runThread;
	thread grabFrameThread;
	Mat frameCapture;
	void grabFrame();
	mutex _mutex;
};

#endif /* KNIGHTVISION_KNIGHTCAMERA_H_ */
