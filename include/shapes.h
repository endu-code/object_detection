#ifndef SHAPES_H // include guard
#define SHAPES_H

/**
 * @file shapes.h
 * @author Dario Aeschlimann (dario.aeschlimann@paoluzzo.ch)
 * @brief 
 * @version 0.1
 * @date 03-11-2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <functional>

/**
 * @class 
 * 
 */

class shapes {
private:
    std::string name; ///< Defines the name of the shape
    std::vector<std::vector<cv::Point>> contour; ///<
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
/**
 * @brief 
 * 
 * @return std::string 
 */

std::string shapes::getName(void) {
    return name;
    }
/**
 * @brief 
 * 
 * @return std::vector<std::vector<cv::Point>> 
 */
std::vector<std::vector<cv::Point>> shapes::getContour(void) {
    return contour;
    }
/**
 * @brief 
 * 
 * @param contour_ 
 */
void shapes::setContour(std::vector<std::vector<cv::Point>> contour_) {
    contour = contour_;
    return;
    }
/**
 * @brief 
 * 
 * @return cv::Scalar 
 */
cv::Scalar shapes::getColor(void) {
    return color;
    }
/**
 * @brief 
 * 
 * @param color_ 
 */
void shapes::setColor(cv::Scalar color_) {
    color = color_;
    return;
    }
/**
 * @brief 
 * 
 * @return std::vector<cv::Vec4i> 
 */
std::vector<cv::Vec4i> shapes::getHierarchy(void) {
    return hierarchy;
    }
/**
 * @brief 
 * 
 * @param hierarchy_ 
 */
void shapes::setHierarchy(std::vector<cv::Vec4i> hierarchy_) {
    hierarchy = hierarchy_;
    return;
    }
/**
 * @brief 
 * 
 * @return cv::Mat 
 */
cv::Mat shapes::getShapeImage(void) {
    return shapeImg;
    }
/**
 * @brief 
 * 
 * @return cv::Mat 
 */
cv::Mat shapes::getDrawnContours(void) {
    return drawnContours;
    }
/**
 * @brief 
 * 
 * @param imagePath_ 
 */
void shapes::setShapeImage(std::string imagePath_) {
    shapeImg = cv::imread(imagePath_);
    return;
    }
/**
 * @brief 
 * 
 * @param image_ 
 */
void shapes::setShapeImage(cv::Mat image_) {
    shapeImg = image_;
    return;
    }
/**
 * @brief Construct a new shapes::shapes object
 * 
 * @param name_ 
 * @param imagePath_ 
 * @param R 
 * @param G 
 * @param B 
 */
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