#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <functional>
#include <mutex>
#include <chrono> 

void imageProcessThread(cv::Mat* capture,
    cv::Mat* processed,
    std::vector<cv::Vec4i>* hierarchy,
    std::vector<std::vector<cv::Point>>* contours,
    std::vector<cv::Point2f>* mass_center,
    std::mutex* capMutex,
    std::mutex* procMutex,
    std::atomic<bool>* exitsignal) {

    ROS_WARN_STREAM("Started Process Thread!");
    cv::Mat newImage, tmp;
    std::vector<std::vector<cv::Point>> tmp_contours;
    std::vector<cv::Vec4i> tmp_hierarchy;
    std::vector<cv::Moments> moments;
    std::vector<cv::Point2f> mc;


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
            cv::findContours(tmp, tmp_contours, tmp_hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
            for (int i = 0; i < tmp_contours.size(); i++) {
                moments[i] = cv::moments(tmp_contours[i], false);
                mc[i] = cv::Point2f(moments[i].m10 / moments[i].m00, moments[i].m01 / moments[i].m00);
                }

            procMutex->lock();
            //ROS_INFO_STREAM("imageProcessThread: procMutex lock!");
            *contours = tmp_contours;
            *processed = tmp;
            *hierarchy = tmp_hierarchy;
            *mass_center = mc;
            procMutex->unlock();
            //ROS_INFO_STREAM("imageProcessThread: procMutex unlock!");

            cv::namedWindow("processed img", cv::WINDOW_AUTOSIZE);
            cv::imshow("processed img", tmp);
            cv::waitKey(1);

            }
        }
    ROS_WARN_STREAM("Stopped Process Thread!");
    }
