#ifndef CAMHANDLER_H
#define CAMHANDLER_H

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <functional>
#include <thread>
#include <future>

class camHandler {
private:
    int deviceID;
    int apiID;
    int acquired;
    int displayed;
    cv::VideoCapture cap;
    enum CamState { error = 1, streaming, paused, ready };
    CamState currentState;
    cv::Mat frame;

public:

    auto initCamera(int deviceID)->camHandler::CamState;
    auto imageStreaming() -> void;
    auto displayImage() -> void;
    auto startStreaming() -> void;
    camHandler();
    ~camHandler();

    };

auto camHandler::initCamera(int deviceID) -> camHandler::CamState {
    this->deviceID = deviceID;
    this->apiID = cv::CAP_ANY;
    cap.open(deviceID, apiID);
    if (!cap.isOpened()) {
        this->currentState = error;
        return this->currentState;
        }
    return ready;

    }

auto camHandler::imageStreaming() -> void {
    this->currentState = streaming;
    while (true)
        {
        std::cout << "Acc " << std::endl;
        cap.read(this->frame);
        this->acquired++;
        if (this->frame.empty()) {
            this->currentState = error;
            break;
            }
        };
    }

auto camHandler::displayImage() -> void {
    while (true) {
        if (!frame.empty()) {
            std::cout << "Disp " << std::endl;
            cv::imshow("Live", this->frame);
            this->displayed++;
            }
        if (cv::waitKey(5) >= 0) {
            this->currentState = paused;
            break;
            }
        };
    }
auto camHandler::startStreaming() -> void {
    auto stream = std::async(&camHandler::imageStreaming, this);
    auto disp = std::async(&camHandler::displayImage, this);
    }

camHandler::camHandler() {
    deviceID = 0;
    apiID = 0;
    acquired = 0;
    displayed = 0;
    }

camHandler::~camHandler() {

    }


#endif