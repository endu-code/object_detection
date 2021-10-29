#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include </home/dario/catkin_ws/src/endu_robotics/include/shapes.h>

void shapeHandlingThread(cv::Mat* processed_shape, std::vector<std::vector<cv::Point>>* contours_shape, std::vector<cv::Vec4i>* hierarchy_shape, std::mutex* procMutex, std::atomic<bool> *exitsignal) {
    ROS_WARN_STREAM("Started Shape Handling Thread!");
    cv::Mat drawnEdges, tmp;
    std::vector<std::vector<cv::Point>> tmp_contours;
    std::vector<cv::Vec4i> tmp_hierarchy;
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
        procMutex->unlock();
        result[0] = 1;
        result[1] = 1;
        result[2] = 1;

        if (!tmp_contours.empty() && !tmp.empty()) {
            for (int i = 0; i < tmp_contours.size(); i++) {
                if (cv::contourArea(tmp_contours.at(i)) > 2000 && cv::contourArea(tmp_contours.at(i)) < 15000) {
                    result[0] = cv::matchShapes(tmp_contours[i], cloud.getContour()[0], 1, 0);
                    result[1] = cv::matchShapes(tmp_contours[i], cross.getContour()[0], 1, 0);
                    result[2] = cv::matchShapes(tmp_contours[i], straight.getContour()[0], 1, 0);

                    if (result[0] == std::min(result[0], (std::min(result[1], result[2])))) {
                        if (result[0] < 0.01) {
                            cv::drawContours(drawnEdges, tmp_contours, (int)i, cloud.getColor(), 2, cv::LINE_8, tmp_hierarchy, 0);
                            cv::putText(drawnEdges, cloud.getName(), tmp_contours.at(i).at(0), 1, 2, cloud.getColor());
                            }
                        }
                    else if (result[1] == std::min(result[1], (std::min(result[0], result[2])))) {
                        if (result[1] < 0.01) {
                            cv::drawContours(drawnEdges, tmp_contours, (int)i, cross.getColor(), 2, cv::LINE_8, tmp_hierarchy, 0);
                            cv::putText(drawnEdges, cross.getName(), tmp_contours.at(i).at(0), 1, 2, cross.getColor());
                            }
                        }
                    else if (result[2] == std::min(result[2], (std::min(result[1], result[0])))) {
                        if (result[2] < 0.01) {
                            cv::drawContours(drawnEdges, tmp_contours, (int)i, straight.getColor(), 2, cv::LINE_8, tmp_hierarchy, 0);
                            cv::putText(drawnEdges, straight.getName(), tmp_contours.at(i).at(0), 1, 2, straight.getColor());
                            }
                        }
                    }
                }
            }
        if (!drawnEdges.empty()) {
            cv::imshow("drawn edges", drawnEdges);
            cv::waitKey(1);
            }
        }

    ROS_WARN_STREAM("Stopped Shape Handling Thread!");
    }
