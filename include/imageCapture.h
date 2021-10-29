#ifndef IMAGECAPTURE_H
#define IMAGECAPTURE_H

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <mutex>
#include <atomic>

void imageCaptureThread(cv::Mat *capture, std::mutex *capMutex, std::atomic<bool> *exitsignal);

#endif