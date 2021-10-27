#ifndef IMAGEPROCESS_H
#define IMAGEPROCESS_H

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <mutex>

void imageProcessThread(cv::Mat* capture, cv::Mat *processed, std::vector<std::vector<cv::Point>> *contours, std::mutex* capMutex, std::mutex* procMutex);

#endif