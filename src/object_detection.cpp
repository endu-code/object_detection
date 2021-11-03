/**
 * @file videocapture.cpp
 * @author Dario Aeschlimann (dario.aeschlimann@paoluzzo.ch)
 * @brief 
 * @version 0.1
 * @date 03-11-2021
 * 
 * @copyright Copyright (c) 2021
 * 
 * @page module_name Videocapture
 */
#include <ros/ros.h>
#include <thread>
#include "imageCapture.h"
#include "imageProcess.h"
#include "shapeHandling.h"
#include "checkExit.h"
#include <atomic>
/**
 * @brief 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv) {

    ros::init(argc, argv, "videocapture");
    ROS_WARN("init done!");
    ros::NodeHandle nh_;
    ros::Rate loop_rate(10);
    std::atomic<bool> exitsignal{ false };

    cv::Mat captureImage, processImage;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::mutex capMutex, procMutex, moMutex;
    std::vector<cv::Moments> moments;
    std::vector<cv::Point2f> centerMass;

    std::thread t_captureImage(imageCaptureThread,
        &captureImage,
        &capMutex,
        &exitsignal);

    std::thread t_processImage(imageProcessThread,
        &captureImage,
        &processImage,
        &hierarchy,
        &contours,
        &centerMass,
        &capMutex,
        &procMutex,
        &exitsignal);

    std::thread t_shapeHandling(shapeHandlingThread,
        &processImage,
        &contours,
        &hierarchy,
        &centerMass,
        &procMutex,
        &moMutex,
        &exitsignal);

    std::thread t_checkExit(checkExit,
        &exitsignal);

    while (ros::ok && !exitsignal) {
        ros::spinOnce();
        loop_rate.sleep();
        }
    t_captureImage.join();
    t_processImage.join();
    t_shapeHandling.join();
    t_checkExit.join();

    nh_.shutdown();
    return 0;
    }