#ifndef SHAPES_H // include guard
#define SHAPES_H

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <functional>

class shapes {
private:
    std::string name;
    std::vector<std::vector<cv::Point>> contour;
    cv::Scalar color;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat shapeImg;
    cv::Mat thresholdedImg;
    cv::Mat blurredImg;
    cv::Mat imgEdges;
    cv::Mat drawnContours;

public:

// Getter Methods
    std::string getName(void);
    std::vector<std::vector<cv::Point>> getContour(void);
    cv::Mat getShapeImage(void);
    cv::Mat getDrawnContours(void);
    std::vector<cv::Vec4i> getHierarchy(void);
    cv::Scalar getColor(void);

//Setter Methods
    void setContour(std::vector<std::vector<cv::Point>> contour_);
    void setColor(cv::Scalar color_);
    void setHierarchy(std::vector<cv::Vec4i> hierarchy_);
    void setShapeImage(std::string imagePath_);
    void setShapeImage(cv::Mat image_);

//class constructor
    shapes(std::string name_, std::string imagePath_, int R, int G, int B);
    };

// Method definitions

std::string shapes::getName(void) {
    return name;
    }

std::vector<std::vector<cv::Point>> shapes::getContour(void) {
    return contour;
    }

void shapes::setContour(std::vector<std::vector<cv::Point>> contour_) {
    contour = contour_;
    return;
    }

cv::Scalar shapes::getColor(void) {
    return color;
    }

void shapes::setColor(cv::Scalar color_) {
    color = color_;
    return;
    }

std::vector<cv::Vec4i> shapes::getHierarchy(void) {
    return hierarchy;
    }

void shapes::setHierarchy(std::vector<cv::Vec4i> hierarchy_) {
    hierarchy = hierarchy_;
    return;
    }

cv::Mat shapes::getShapeImage(void) {
    return shapeImg;
    }

cv::Mat shapes::getDrawnContours(void) {
    return drawnContours;
    }

void shapes::setShapeImage(std::string imagePath_) {
    shapeImg = cv::imread(imagePath_);
    return;
    }

void shapes::setShapeImage(cv::Mat image_) {
    shapeImg = image_;
    return;
    }

shapes::shapes(std::string name_, std::string imagePath_, int R, int G, int B) {
    name = name_;
    shapeImg = cv::imread(imagePath_);
    cv::threshold(shapeImg, thresholdedImg, 100, 255, 1);
    cv::blur(thresholdedImg, blurredImg, cv::Size(3, 3));
    cv::Canny(blurredImg, imgEdges, 0, 0, 3);
    cv::findContours(imgEdges, contour, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    color = cv::Scalar(R, G, B);
    drawnContours = cv::Mat::zeros(imgEdges.size(), CV_8UC3);
    for (size_t j = 0; j < contour.size(); j++) {
        cv::drawContours(drawnContours, contour, (int)j, color, 2, cv::LINE_8, hierarchy, 0);
        }
    return;
    };

    class contour{
        private:
        std::vector<std::vector<cv::Point>> contour_;
        std::vector<cv::Vec4i> hierarchy_;

        public:
        auto getContour()->std::vector<std::vector<cv::Point>>;
        auto getHierarchy()->std::vector<cv::Vec4i>;

    };

    auto contour::getContour()->std::vector<std::vector<cv::Point>>{
        return this->contour_;
    }

    auto contour::getHierarchy()->std::vector<cv::Vec4i>{
        return this->hierarchy_;
    }

#endif /* SHAPES_H */