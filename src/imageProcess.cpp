#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <functional>
#include <mutex>

void imageProcessThread(cv::Mat* capture, cv::Mat *processed, std::vector<std::vector<cv::Point>> *contours, std::mutex* capMutex, std::mutex* procMutex) {
    ROS_WARN_STREAM("Started Process Thread!");
    cv::Mat newImage, tmp;
    std::vector<std::vector<cv::Point>> tmp_contours;

    while (ros::ok) {

        capMutex->lock();
        newImage = *capture;
        capMutex->unlock();

        if (!newImage.empty()) {
            ROS_INFO_STREAM("Processing!");

            // Convert Image to Greyscale, apply a threshold, blur and edge detection
            cv::cvtColor(newImage, tmp, cv::COLOR_BGR2GRAY);
            cv::threshold(tmp, tmp, 175, 255, 1);
            cv::blur(tmp, tmp, cv::Size(3, 3));
            cv::Canny(tmp, tmp, 0, 0, 3);

            //get contours
            cv::findContours(tmp, tmp_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            procMutex->lock();
            *contours = tmp_contours;
            *processed = tmp;
            procMutex->unlock();

            cv::namedWindow( "processed img", cv::WINDOW_AUTOSIZE);
            cv::imshow("processed img", tmp);
            cv::waitKey(1);
            }
        }
    return;
    }
