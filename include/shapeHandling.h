#ifndef SHAPEHANDLING_H
#define SHAPEHANDLING_H

/**
 * @file shapeHandling.h
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
#include <atomic>

/**
 * @brief Thread function for handling the shape recognition and matching of the shapes.
 * 
 * @param processed_shape   Pointer to previously processed image
 * @param contours_shape    Pointer to previously detected contours
 * @param hierarchy_shape   Pointer to previously detected hierarchy of contours (Probably unnecessary since only outer contours are detected)
 * @param centerMass        Pointer to array of center of mass coordinates
 * @param procMutex         Mutex for safe Multithreading
 * @param moMutex           Mutex for safe Multithreading
 * @param exitsignal        Atomic Boolean for handling exit signal
 * 
 * 
 */

void shapeHandlingThread(cv::Mat* processed_shape,
    std::vector<std::vector<cv::Point>>* contours_shape,
    std::vector<cv::Vec4i>* hierarchy_shape,
    std::vector<cv::Point2f>* centerMass,
    std::mutex* procMutex,
    std::mutex* moMutex,
    std::atomic<bool>* exitsignal);

#endif 