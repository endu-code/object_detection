#ifndef SHAPEHANDLING_H
#define SHAPEHANDLING_H

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"


void shapeHandlingThread(cv::Mat *processed, std::vector<std::vector<cv::Point>> *contours, std::vector<cv::Vec4i> *hierarchy, std::mutex *procMutex);

#endif