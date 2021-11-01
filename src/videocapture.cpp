#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <future>
#include <thread>
#include <functional>
#include "imageCapture.h"
#include "imageProcess.h"
#include "shapeHandling.h"
#include "checkExit.h"
#include <atomic>

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