#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <functional>
#include </home/dario/catkin_ws/src/endu_robotics/include/shapes.h>
#include </home/dario/catkin_ws/src/endu_robotics/include/camHandler.h>
#include </home/dario/catkin_ws/src/endu_robotics/include/imageProcessor.h>



int main(int argc, char** argv)
    {
    camHandler ch;
    ch.initCamera(0);
    ch.startStreaming();

    shapes cloud("cloud", "/home/dario/catkin_ws/src/endu_robotics/sample/Cloud Piece.JPG", 255, 0, 0);
    shapes cross("cross", "/home/dario/catkin_ws/src/endu_robotics/sample/Cross Piece.JPG", 0, 255, 0);
    shapes straight("straight", "/home/dario/catkin_ws/src/endu_robotics/sample/Straight Piece.JPG", 0, 0, 255);
    return 0;


    }