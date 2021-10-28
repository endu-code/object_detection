#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <functional>
#include </home/dario/catkin_ws/src/endu_robotics/include/shapes.h>

using namespace cv;

RNG rng(12345);

void MyLine(Mat img, Point start, Point end) {
    int thickness = 4;
    int lineType = LINE_8;

    line(img, start, end, Scalar(0, 0, 0), thickness, lineType);
    }
void camStream(int deviceID){
    VideoCapture capture(deviceID);
    Mat captureFrame;

    if (!capture.open(deviceID)){
        return;
    }

    capture >> captureFrame;
    
}


int main(int argc, char** argv)
    {
    // VideoCapture cap(2);
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    // if (!cap.open(2))
    //     return 0;

    // Load Test Image for shape detection
    Mat TestImage = imread("/home/dario/catkin_ws/src/endu_robotics/sample/Test.png");

    // Load sample images and store in Array
    //Mat shapes[3], shapes_bin[3], detected_edges, shape_edges[3], drawn_contours[3], drawnEdges;
    Mat detected_edges, drawnEdges;
    std::vector<std::vector<Point>> contours, shape_contours[3], contours_filtered;
    std::vector<Vec4i> hierarchy, shape_hierarchy[3];
    Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    double result[3];

    shapes cloud("cloud", "/home/dario/catkin_ws/src/endu_robotics/sample/Cloud Piece.JPG", 255, 0, 0);
    shapes cross("cross", "/home/dario/catkin_ws/src/endu_robotics/sample/Cross Piece.JPG", 0, 255, 0);
    shapes straight("straight", "/home/dario/catkin_ws/src/endu_robotics/sample/Straight Piece.JPG", 0, 0, 255);

    while (1)
        {
        Mat frame, frame_gray, frame_bin;
        // cap >> frame;
        frame = TestImage;

        cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
        threshold(frame_gray, frame_bin, 150, 255, 1);
        blur(frame_bin, frame_bin, Size(3, 3));
        Canny(frame_bin, detected_edges, 0, 0, 3);

        findContours(detected_edges, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        drawnEdges = Mat::zeros(detected_edges.size(), CV_8UC3);

        for (size_t i = 0; i < contours.size(); i++) {
           if (contourArea(contours.at(i)) > 2000 && contourArea(contours.at(i)) < 15000 ) {
                color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
                result[0] = matchShapes(contours[i], cloud.getContour()[0], 1, 0);
                result[1] = matchShapes(contours[i], cross.getContour()[0], 1, 0);
                result[2] = matchShapes(contours[i], straight.getContour()[0], 1, 0);

                if (result[0] == std::min(result[0], (std::min(result[1], result[2])))) {
                    if (result[0] < 0.01) {
                        drawContours(drawnEdges, contours, (int)i, cloud.getColor(), 2, LINE_8, hierarchy, 0);
                        putText(drawnEdges, cloud.getName(), contours.at(i).at(0), 1, 2, cloud.getColor());
                        }
                    }
                else if (result[1] == std::min(result[1], (std::min(result[0], result[2])))) {
                    if (result[1] < 0.01) {
                        drawContours(drawnEdges, contours, (int)i, cross.getColor(), 2, LINE_8, hierarchy, 0);
                        putText(drawnEdges, cross.getName(), contours.at(i).at(0), 1, 2, cross.getColor());
                        }
                    }
                else if (result[2] == std::min(result[2], (std::min(result[1], result[0])))) {
                    if (result[2] < 0.01) {
                        drawContours(drawnEdges, contours, (int)i, straight.getColor(), 2, LINE_8, hierarchy, 0);
                        putText(drawnEdges, straight.getName(), contours.at(i).at(0), 1, 2, straight.getColor());
                        }
                    }

                if (frame.empty()) break; // end of video stream
                imshow("frame", frame);
                imshow("frame_bin", frame_bin);
                // imshow("detected_edges", detected_edges);
                imshow("drawnCountours", drawnEdges);
                if (waitKey(10) == 27) break; // stop capturing by pressing ESC 
                }
            }
        }
    return 0;
    }