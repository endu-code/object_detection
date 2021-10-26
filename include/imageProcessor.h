#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <functional>
#include </home/dario/catkin_ws/src/endu_robotics/include/shapes.h>

class imageProcessor {
private:
    enum imgProcState { error = 1, ready };
    imgProcState currentState;
    contour contour;


public:

    auto greyScaleImg(cv::Mat& inputImage_)->cv::Mat;
    auto applyThreshold(cv::Mat& inputImage_)->cv::Mat;
    auto blurImage(cv::Mat& inputImage_)->cv::Mat;
    auto edgeDetecting(cv::Mat& inputImage_)->cv::Mat;
    auto findContours(cv::Mat& inputImage_)->std::vector<std::vector<cv::Point>>;

    };



auto imageProcessor::greyScaleImg(cv::Mat& inputImage_)->cv::Mat {
    cv::Mat temp_ = inputImage_;
    cv::cvtColor(temp_, inputImage_, cv::COLOR_BGR2GRAY);
    return inputImage_;
    }

auto imageProcessor::applyThreshold(cv::Mat& inputImage_)->cv::Mat {
    cv::Mat temp_ = inputImage_;
    cv::threshold(temp_, inputImage_, 175, 255, 1);
    return inputImage_;
    }

auto imageProcessor::blurImage(cv::Mat& inputImage_)->cv::Mat {
    cv::Mat temp_ = inputImage_;
    cv::blur(temp_, inputImage_, cv::Size(3, 3));
    return inputImage_;
    }

auto imageProcessor::findContours(cv::Mat& inputImage_)->std::vector<std::vector<cv::Point>>{
    cv::findContours(inputImage_, contour.getContour(), contour.getHierarchy(), cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
}

#endif
