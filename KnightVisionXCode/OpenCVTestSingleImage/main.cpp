//
//  main.cpp
//  OpenCVTestSingleImage
//
//  Created by Robert Hilton on 5/13/18.
//  Copyright © 2018 Hilton Tuning. All rights reserved.
//

#include <iostream>
#include <opencv2/opencv.hpp>

//#define DEBUG
//#define SAVE_JPG
//#define JETSON_RUN

using namespace std;
using namespace cv;

const double dilation_size = 1.75;
const Scalar color = Scalar(50, 255, 0);

const Mat dialationElement = getStructuringElement(MORPH_RECT,
									Size(2 * dilation_size + 1, 2 * dilation_size + 1),
									Point(dilation_size, dilation_size));

const Mat morphElement = getStructuringElement(MORPH_ELLIPSE,Size(5,5));

const double EXPECTED_SCALE_MIN_AREA = 8000;


int main(int argc, const char * argv[]) {

	bool runKey = true;
	Mat canny_output, dst;
#ifdef JETSON_RUN
	Mat frame = imread("/home/ubuntu/KnightVision/imageStore/ImgOri255.jpg");
#else
	Mat frame = imread("/Users/roberthilton/Desktop/KnightVisionXCode/CamImage.JPG");
#endif
	
	
	cvtColor(frame, dst, COLOR_BGR2HSV);
	Scalar hsvLowThreshold = Scalar(132, 40, 40);
	Scalar hsvHighThreshold = Scalar(154, 160, 160);
	inRange(dst, hsvLowThreshold, hsvHighThreshold, dst); // HSV threshold to ID target
	
	morphologyEx(dst, dst, MORPH_OPEN, morphElement);	//Morph open to remove unwanted particles
	dilate(dst, dst, dialationElement, Point(-1, -1), 2); // Dilate image to fill in any imperfections
	
	Canny(dst, canny_output, 0, 0); // Detect edges of target - Add 6ms
	
	vector<vector<Point> > contoursVector;
	
	findContours(canny_output, contoursVector, noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE , Point(0, 0));
	
	vector<RotatedRect> boxes;
	
	//Draw image with found contours for viewing/testing, fill vector with the smallest
	//rotated rectangles that contain the contours
	
	//Mat drawingOut = Mat::zeros(canny_output.size(), CV_32F);
	for(int i = 0; i < contoursVector.size(); i++) {
		RotatedRect tmpRect = minAreaRect(contoursVector[i]);
		
		if (tmpRect.size.area() < EXPECTED_SCALE_MIN_AREA)
			continue;
		
		boxes.push_back(tmpRect);
		//drawContours(drawingOut, contoursVector, i, color, 1, 4, noArray(), 0, Point());
	}
	
	if (boxes.size() != 2) {
		printf("Failure to identify scale swatches!");
		return 1;
	}
	
	RotatedRect leftRect, rightRect;
	
	if (boxes[0].center.x < boxes[1].center.x) {
		leftRect = boxes[0];
		rightRect = boxes[1];
	}
	else {
		leftRect = boxes[1];
		rightRect = boxes[0];
	}
	
	double scaleSlopeDeg = atan2(rightRect.center.y - leftRect.center.y, rightRect.center.x - leftRect.center.x) * 180.0 / M_PI;
	
	cout << scaleSlopeDeg << endl;
	
	//Draw outlines of selected particles and center points
	for(int i = 0; i < boxes.size(); i++) {
		Point2f rect_points[4];
		boxes[i].points(rect_points);
		circle(frame, boxes[i].center, 5, color, -1);
		for(int j = 0; j < 4; j++) {
			line(frame, rect_points[j], rect_points[(j+1)%4], color, 3, CV_AA);
		}
	}
	
	line(frame, boxes[0].center, boxes[1].center, color, 3, CV_AA);
	
	
	
#ifndef JETSON_RUN
	imshow("output", frame);
	while (runKey)
	{
		char keyCode = waitKey(30);
		if (keyCode == ' ')
			runKey = false;
	}
#endif
    return 0;
}
