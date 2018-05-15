#include "opencv2/opencv.hpp"
#include "KnightCamera.h"
#include "UDPSocket.h"
#include "SimpleGPIO.h"
#include <signal.h>
#include <iostream>
#include <thread>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <chrono>

//#define SAVE_JPG
//#define DEBUG

using namespace cv;
using namespace std;

Mat frame, prevFrame, img, procImg, dst, canny_output, drawing;

string saveStr;
string saveStrOri;

const double dilation_size = 1.6;
const Scalar color = Scalar(255, 255, 255);

Mat element = getStructuringElement(MORPH_RECT,
									Size(2 * dilation_size + 1, 2 * dilation_size + 1),
									Point(dilation_size, dilation_size));

int particle1 = -1;
int particle2 = -1;
int lowestDiff, currentDiff;
double averageCenterX;

const unsigned int LEDGPIO = 57;
//const unsigned int CAMGPIO = 81;

double pictureWidth;
double pictureHeight;

//double focalLength = (186.6520701734*24/8.5); //24 inch distance
double focalLength = (31.48*111)/4;
double stripeHeightImg = 0;

const int minBlobArea = 300;
const int minContourSize = 20;
//const double cameraFOVh = 170/3;
//const double cameraFOVv = 170/3;
const double cameraFOVh = 72;
const double cameraFOVv = 55;
const double centerYDeviationIn = 7;
const double allowedError = .45;
bool onTarget = false;
bool targetFound = false;
double deviation = 0;
double targetDistance = 0;
unsigned long sequence = 0;
bool runLoop = true;

int skippedCycleCounter = 0;

double heightI, widthI, angleI, heightK, widthK, angleK;

UDPSocket *turretUDP;
KnightCamera *turretCam;

VideoCapture *turretCamCap;

int init() {
	turretUDP = new UDPSocket(5801);
	turretUDP->init();
	
	gpio_export(LEDGPIO);
	gpio_set_dir(LEDGPIO, PIN_DIRECTION::OUTPUT_PIN);
	gpio_set_value(LEDGPIO, PIN_VALUE::HIGH);
	
	int deviceNum = 0;
	for (deviceNum = 0; deviceNum < 10; deviceNum++) {
		turretCamCap = new VideoCapture(deviceNum); // open the default camera
		if(turretCamCap->isOpened())  // check if we succeeded
			break;
	}
	if(!turretCamCap->isOpened())  // check if we succeeded
		return -1;
	
	system(("v4l2-ctl -d " + to_string(deviceNum) + " --set-fmt-video=width=640,height=480,pixelformat=0 --set-parm=120").c_str());
	system(("v4l2-ctl -d " + to_string(deviceNum) + " --set-ctrl=exposure_auto_priority=1,exposure_auto=1,exposure_absolute=3,brightness=-64,white_balance_temperature_auto=0,white_balance_temperature=6500").c_str());

	cout << "V4L2 done!" << endl;
	turretCamCap->set(CV_CAP_PROP_FRAME_WIDTH, 640);
	turretCamCap->set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	turretCamCap->set(CV_CAP_PROP_FPS, 120);
	 
	 
	turretCam = new KnightCamera(turretCamCap);
	
	runLoop = true;
	
	//this_thread::sleep_for(chrono::milliseconds(500));
	
	turretCam->start();
	this_thread::sleep_for(chrono::milliseconds(100));
	frame = turretCam->getLatestFrame();
	pictureWidth = 640;
	pictureHeight = 480;
	
	gpio_set_value(LEDGPIO, PIN_VALUE::LOW);
	this_thread::sleep_for(chrono::milliseconds(150));
	gpio_set_value(LEDGPIO, PIN_VALUE::HIGH);
	this_thread::sleep_for(chrono::milliseconds(150));
	gpio_set_value(LEDGPIO, PIN_VALUE::LOW);
	this_thread::sleep_for(chrono::milliseconds(150));
	gpio_set_value(LEDGPIO, PIN_VALUE::HIGH);
	this_thread::sleep_for(chrono::milliseconds(150));
	gpio_set_value(LEDGPIO, PIN_VALUE::LOW);
	this_thread::sleep_for(chrono::milliseconds(150));
	gpio_set_value(LEDGPIO, PIN_VALUE::HIGH);
	
	frame = Mat::zeros(pictureHeight, pictureWidth, CV_8UC3);
	prevFrame = Mat::zeros(pictureHeight, pictureWidth, CV_8UC3);
	
	
	cout << "Full init finished!" << endl;
	
	return 0;
}

void preExit() {
	gpio_set_value(LEDGPIO, PIN_VALUE::LOW);
	gpio_unexport(LEDGPIO);
	turretCam->stop();
	turretCamCap->release();
}

void exitFunction(int exitCode) {
	preExit();
	exit(exitCode);
}

void interruptHandler(int s){
	//printf("Caught signal %d\n",s);
	if (s == 2) {
		exitFunction(1);
	}
}

int main(int argc, char** argv) {
	//Handle Ctrl-C Signals
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = interruptHandler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);
	
	init();
	
#ifdef SAVE_JPG
	int zIndex = 0;
#endif
	while(runLoop) {
		try {
			frame = turretCam->getLatestFrame();

			if(sum(frame != prevFrame) == Scalar(0,0,0,0)) {
#ifdef DEBUG
				cout << "Skipping cycle." << endl;
#endif
				if (skippedCycleCounter++ > 5) {
					exitFunction(1);
				}
				continue;
			}
			skippedCycleCounter = 0;
			
#ifdef DEBUG
			cout << "Got frame." << endl;
#endif
			
			prevFrame = frame;
			
			threshold(frame,procImg,180,255,CV_THRESH_BINARY);
			dilate(procImg, dst, element); // Dilate image to fill in any imperfections
			cvtColor(dst, dst, COLOR_BGR2HSV);
			inRange(dst, Scalar(40, 0, 0), Scalar(65, 255, 255), dst); // HSV threshold to ID target
			Canny(dst, canny_output, 0, 0); // Detect edges of target - Add 6ms
			vector<vector<Point> > contoursVector;
			findContours(canny_output, contoursVector, noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE , Point(0, 0));
			
			for (vector<vector<Point> >::iterator it = contoursVector.begin(); it!=contoursVector.end(); )
			{
				if (it->size() < minContourSize)
					it=contoursVector.erase(it);
				else
					++it;
			}
			
			vector<RotatedRect> boxes(contoursVector.size());
			
#ifdef SAVE_JPG
			Mat drawingOut = Mat::zeros(canny_output.size(), CV_32F);
			saveStrOri = "imageStore/ImgOri" + to_string(++zIndex) + ".jpg";
			imwrite( saveStrOri, frame );
#endif
			for(int i = 0; i < contoursVector.size(); i++) {
#ifdef SAVE_JPG
				drawContours(drawingOut, contoursVector, i, color, 1, 4, noArray(), 0, Point());
				saveStrOri = "imageStore/ImgContours" + to_string(zIndex) + ".jpg";
				imwrite( saveStrOri, drawingOut );
#endif
				boxes[i] = minAreaRect(contoursVector[i]);
			}
			
			for (vector<RotatedRect>::iterator it = boxes.begin(); it!=boxes.end(); )
			{
				if (it->size.area() < minBlobArea)
					it=boxes.erase(it);
				else
					++it;
			}
			
			particle1 = -1;
			particle2 = -1;
			lowestDiff = 10000;
			currentDiff = 10000;
			stripeHeightImg = 0;
			for(int i = 0; i < boxes.size(); i++) {
				for(int k = 0; k < boxes.size(); k++) {
					heightI = 0;
					widthI = 0;
					heightK = 0;
					widthK = 0;
					if (boxes[i].size.height > boxes[i].size.width) {
						heightI = boxes[i].size.width;
						widthI = boxes[i].size.height;
					} else {
						heightI = boxes[i].size.height;
						widthI = boxes[i].size.width;
					}
					if (boxes[k].size.height > boxes[k].size.width) {
						heightK = boxes[k].size.width;
						widthK = boxes[k].size.height;
					} else {
						heightK = boxes[k].size.height;
						widthK = boxes[k].size.width;
					}
					
					angleI = boxes[i].angle;
					angleK = boxes[k].angle;
					
					if(i != k) {
						if (boxes[i].size.area() < 40)
						{
							k = (int)boxes.size() + 1;
#ifdef DEBUG
							cout << "Skipping particle i" << i << endl;
#endif
							continue;
						}
						if (boxes[k].size.area() < 40)
						{
#ifdef DEBUG
							cout << "Skipping particle k" << k << endl;
#endif
							continue;
						}
						
						
#ifdef DEBUG
						cout << "JUST INFO" << endl;
						cout << "Particle 1 Angle: " << boxes[i].angle << endl;
						cout << "Particle 2 Angle: " << boxes[k].angle << endl;
						cout << "Particle 1: " << boxes[i].size.area() << endl;
						cout << "Particle 2: " << boxes[k].size.area() << endl;
						cout << "Particle 1 height: " << heightI << endl;
						cout << "Particle 1 width: " << widthI << endl;
						cout << "Particle 2 height: " << heightK << endl;
						cout << "Particle 2 width: " << widthK << endl;
						cout << "Width Difference: " << abs(widthI - widthK) << endl;
						cout << "Center of X difference: " << lowestDiff << endl;
						cout << "Center of Y difference: " << boxes[i].center.y - boxes[k].center.y << endl;
						cout << "END JUST INFO" << endl;
#endif
						
						currentDiff = abs(boxes[i].center.x - boxes[k].center.x);
						if(currentDiff < lowestDiff) {
							lowestDiff = currentDiff;
							if(abs(widthI - widthK) < 20 && lowestDiff < 10 && widthI > heightI && widthK > heightK && abs(boxes[i].center.y - boxes[k].center.y) < 50) {
								//&& abs(angleI - angleK) < 6
								particle1 = i;
								particle2 = k;
								
								if (heightI > heightK)
									stripeHeightImg = heightI;
								else
									stripeHeightImg = heightK;
								
#ifdef DEBUG
								cout << "Particle 1 Angle: " << boxes[i].angle << endl;
								cout << "Particle 2 Angle: " << boxes[k].angle << endl;
								cout << "Particle 1: " << boxes[i].size.area() << endl;
								cout << "Particle 2: " << boxes[k].size.area() << endl;
								cout << "Particle 1 height: " << heightI << endl;
								cout << "Particle 1 width: " << widthI << endl;
								cout << "Particle 2 height: " << heightK << endl;
								cout << "Particle 2 width: " << widthK << endl;
								cout << "Width Difference: " << abs(widthI - widthK) << endl;
								cout << "Center of X difference: " << lowestDiff << endl;
								cout << "Center of Y difference: " << boxes[i].center.y - boxes[k].center.y << endl;
#endif
							}
						}
					}
				}
			}
			
			vector<RotatedRect> filteredBoxes;
			
			if(particle1 != -1 && particle2 != -1) {
				filteredBoxes.push_back(boxes[particle1]);
				filteredBoxes.push_back(boxes[particle2]);
			}
			//else
			//	continue;
			
			averageCenterX = 0;
			
#ifdef DEBUG
			
			cout << "Focal Length: " << focalLength << endl;
			cout << "stripe height: " << stripeHeightImg << endl;
#endif
			
			for(int i = 0; i < filteredBoxes.size(); i++) {
				averageCenterX += filteredBoxes[i].center.x;
			}
			
			if (particle1 < 0 || particle2 < 0 || filteredBoxes.size() < 2) {
				deviation = 0;
				targetFound = false;
				onTarget = false;
				averageCenterX = 0;
				targetDistance = 0;
			} else {
				averageCenterX /= filteredBoxes.size();
				targetDistance = 1/tan((abs(filteredBoxes[0].center.y - filteredBoxes[1].center.y)/(double)pictureHeight) * cameraFOVv * M_PI / 180) * centerYDeviationIn;
				deviation = (averageCenterX - (pictureWidth / 2)) / pictureWidth * cameraFOVh;
				targetFound = true;
				onTarget = abs(deviation) <= allowedError;
			}
			turretUDP->udpSendSynchronous(deviation, targetDistance, onTarget, targetFound, sequence++);
			
#ifdef SAVE_JPG
			for(int i = 0; i < filteredBoxes.size(); i++) {
				Point2f rect_points[4];
				filteredBoxes[i].points(rect_points);
				for(int j = 0; j < 4; j++)
					line(drawingOut, rect_points[j], rect_points[(j+1)%4], color, 1, CV_AA);
			}
			
			
			saveStr = "imageStore/Img" + to_string(zIndex) + ".jpg";
			if (filteredBoxes.size() > 0)
			{
				imwrite( saveStr, drawingOut );
			}
#endif
			
#ifdef DEBUG
			cout << "Target Distance: " << targetDistance << endl;
#endif
			
			
		} catch(exception &ex) {
			cout << "Exception main stuff:" << endl;
			cout << ex.what() << endl;
			exitFunction(2);
		}
	}
	
	preExit();
	return 0;
}

