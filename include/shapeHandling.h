#ifndef SHAPEHANDLING_H
#define SHAPEHANDLING_H

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <atomic>


void shapeHandlingThread(cv::Mat* processed_shape, std::vector<std::vector<cv::Point>>* contours_shape, std::vector<cv::Vec4i>* hierarchy_shape, std::mutex* procMutex, std::atomic<bool> *exitsignal);

#endif