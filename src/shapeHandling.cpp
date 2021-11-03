/**
 * @file shapeHandling.cpp
 * @author Dario Aeschlimann (dario.aeschlimann@paoluzzo.ch)
 * @brief 
 * @version 0.1
 * @date 01-11-2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <shapes.h>

void shapeHandlingThread(cv::Mat* processed_shape,
    std::vector<std::vector<cv::Point>>* contours_shape,
    std::vector<cv::Vec4i>* hierarchy_shape,
    std::vector<cv::Point2f>* centerMass,
    std::mutex* procMutex,
    std::mutex* moMutex,
    std::atomic<bool>* exitsignal) {
        
    ROS_WARN_STREAM("Started Shape Handling Thread!");
    cv::Mat drawnEdges, tmp;
    std::vector<std::vector<cv::Point>> tmp_contours;
    std::vector<cv::Vec4i> tmp_hierarchy;
    std::vector<cv::Point2f> tmp_centerMass;

    double result[3];
    result[0] = 1;
    result[1] = 1;
    result[2] = 1;

    shapes cloud("cloud", "/home/dario/catkin_ws/src/endu_robotics/sample/Cloud Piece.JPG", 255, 0, 0);
    shapes cross("cross", "/home/dario/catkin_ws/src/endu_robotics/sample/Cross Piece.JPG", 0, 255, 0);
    shapes straight("straight", "/home/dario/catkin_ws/src/endu_robotics/sample/Straight Piece.JPG", 0, 0, 255);

    while (ros::ok && !(*exitsignal)) {

        drawnEdges = cv::Mat::zeros(tmp.size(), CV_8UC3);
        procMutex->lock();
        tmp_contours = *contours_shape;
        tmp = *processed_shape;
        tmp_hierarchy = *hierarchy_shape;
        tmp_centerMass = *centerMass;
        procMutex->unlock();
        result[0] = 1;
        result[1] = 1;
        result[2] = 1;

        if (!tmp_contours.empty() && !tmp.empty()) {
            for (int i = 0; i < tmp_contours.size(); i++) {
                if (cv::contourArea(tmp_contours.at(i)) > 2000 && cv::contourArea(tmp_contours.at(i)) < 20000) {
                    result[0] = cv::matchShapes(tmp_contours[i], cloud.getContour()[0], 1, 0);
                    result[1] = cv::matchShapes(tmp_contours[i], cross.getContour()[0], 1, 0);
                    result[2] = cv::matchShapes(tmp_contours[i], straight.getContour()[0], 1, 0);

                    if (result[0] == std::min(result[0], (std::min(result[1], result[2])))) {
                        if (result[0] < 0.01) {
                            cv::drawContours(drawnEdges, tmp_contours, (int)i, cloud.getColor(), 2, cv::LINE_8, tmp_hierarchy, 0);
                            cv::putText(drawnEdges, cloud.getName(), tmp_contours.at(i).at(0), 1, 2, cloud.getColor());
                            cv::circle(drawnEdges, tmp_centerMass.at(i), 4, cloud.getColor());
                            }
                        }
                    else if (result[1] == std::min(result[1], (std::min(result[0], result[2])))) {
                        if (result[1] < 0.01) {
                            cv::drawContours(drawnEdges, tmp_contours, (int)i, cross.getColor(), 2, cv::LINE_8, tmp_hierarchy, 0);
                            cv::putText(drawnEdges, cross.getName(), tmp_contours.at(i).at(0), 1, 2, cross.getColor());
                            cv::circle(drawnEdges, tmp_centerMass.at(i), 4, cross.getColor());
                            }
                        }
                    else if (result[2] == std::min(result[2], (std::min(result[1], result[0])))) {
                        if (result[2] < 0.01) {
                            cv::drawContours(drawnEdges, tmp_contours, (int)i, straight.getColor(), 2, cv::LINE_8, tmp_hierarchy, 0);
                            cv::putText(drawnEdges, straight.getName(), tmp_contours.at(i).at(0), 1, 2, straight.getColor());
                            cv::circle(drawnEdges, tmp_centerMass.at(i), 4, straight.getColor());
                            }
                        }
                    }
                }
            }
        if (!drawnEdges.empty()) {
            cv::imwrite("/home/dario/catkin_ws/src/endu_robotics/sample/output.JPG", drawnEdges);
            cv::imshow("drawn edges", drawnEdges);
            cv::waitKey(1);
            }
        }
    ROS_WARN_STREAM("Stopped Shape Handling Thread!");
    }