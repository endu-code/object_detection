#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <functional>
#include <mutex>
#include <chrono>

void imageProcessThread(cv::Mat* capture, cv::Mat *processed, std::vector<cv::Vec4i> *hierarchy, std::vector<std::vector<cv::Point>> *contours, std::mutex* capMutex, std::mutex* procMutex, std::atomic<bool> *exitsignal) {
    ROS_WARN_STREAM("Started Process Thread!");
    cv::Mat newImage, tmp;
    std::vector<std::vector<cv::Point>> tmp_contours;
    std::vector<cv::Vec4i> tmp_hierarchy;


    while (ros::ok && !(*exitsignal)) {

        tmp_contours.clear();
        tmp_hierarchy.clear();

        capMutex->lock();
        //ROS_INFO_STREAM("imageProcessThread: capMutex lock!");
        newImage = *capture;
        capMutex->unlock();
        //ROS_INFO_STREAM("imageProcessThread: capMutex unlock!");

        if (!newImage.empty()) {
            //ROS_INFO_STREAM("imageProcessThread: Processing!");

            // Convert Image to Greyscale, apply a threshold, blur and edge detection
            cv::cvtColor(newImage, tmp, cv::COLOR_BGR2GRAY);
            // cv::threshold(tmp, tmp, 150, 255, 1);
            // cv::imshow("thresholded img", tmp);
            // cv::waitKey(1);
            cv::blur(tmp, tmp, cv::Size(3, 3));
            cv::imshow("blurred img", tmp);
            cv::waitKey(1);
            cv::Canny(tmp, tmp, 0, 0, 3);

            //get contours
            cv::findContours(tmp, tmp_contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
            procMutex->lock();
            //ROS_INFO_STREAM("imageProcessThread: procMutex lock!");
            *contours = tmp_contours;
            *processed = tmp;
            *hierarchy = tmp_hierarchy;
            procMutex->unlock();
            //ROS_INFO_STREAM("imageProcessThread: procMutex unlock!");

            cv::namedWindow( "processed img", cv::WINDOW_AUTOSIZE);
            cv::imshow("processed img", tmp);
            cv::waitKey(1);
            }
        }
    ROS_WARN_STREAM("Stopped Process Thread!");
    }
