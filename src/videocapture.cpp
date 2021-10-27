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

int main(int argc, char** argv) {

    ros::init(argc, argv, "videocapture");
    ROS_WARN("init done!");
    ros::NodeHandle nh_;
    ros::Rate loop_rate(10);

    cv::Mat captureImage, processImage;
    std::vector<std::vector<cv::Point>> contours;
    std::mutex capMutex, procMutex;

    std::thread t_captureImage(imageCaptureThread, &captureImage, &capMutex);
    std::thread t_processImage(imageProcessThread, &captureImage, &processImage, &contours, &capMutex, &procMutex);
    std::thread t_shapeHandling(shapeHandlingThread, &processImage, &contours, &procMutex);
    while (ros::ok) {
        ros::spinOnce();
        loop_rate.sleep();
        }
    t_captureImage.join();
    t_processImage.join();
    return 0;
    }