#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <functional>

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
    VideoCapture cap(2);
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if (!cap.open(2))
        return 0;

    // Load Test Image for shape detection
    //Mat TestImage = imread("/home/dario/catkin_ws/src/endu_robotics/sample/Test.png");

    // Load sample images and store in Array
    Mat shapes[3], shapes_bin[3], detected_edges, shape_edges[3], drawn_contours[3], drawnEdges;
    std::vector<std::vector<Point>> contours, shape_contours[3], contours_filtered;
    std::vector<Vec4i> hierarchy, shape_hierarchy[3];
    Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    double result[3];

    shapes[0] = imread("/home/dario/catkin_ws/src/endu_robotics/sample/Cloud Piece.JPG");
    shapes[1] = imread("/home/dario/catkin_ws/src/endu_robotics/sample/Cross Piece.JPG");
    shapes[2] = imread("/home/dario/catkin_ws/src/endu_robotics/sample/Straight Piece.JPG");

    // Apply Binary threshold to sample shapes

    for (size_t i = 0; i <= 2; i++) {
        threshold(shapes[i], shapes_bin[i], 100, 255, 1);
        blur(shapes_bin[i], shapes_bin[i], Size(3, 3));
        Canny(shapes_bin[i], shape_edges[i], 0, 0, 3);
        findContours(shape_edges[i], shape_contours[i], shape_hierarchy[i], RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
       drawn_contours[i] = Mat::zeros(shape_edges[i].size(), CV_8UC3);
        for (size_t j = 0; j < shape_contours[i].size(); j++) {
            color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
            drawContours(drawn_contours[i], shape_contours[i], (int)j, color, 2, LINE_8, shape_hierarchy[i], 0);
            }
        }

    while (1)
        {
        Mat frame, frame_gray, frame_bin;
        cap >> frame;
        //frame = TestImage;

        cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
        threshold(frame_gray, frame_bin, 175, 255, 1);
        blur(frame_bin, frame_bin, Size(3, 3));
        Canny(frame_bin, detected_edges, 0, 0, 3);

        findContours(detected_edges, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        drawnEdges = Mat::zeros(detected_edges.size(), CV_8UC3);

        for (size_t i = 0; i < contours.size(); i++) {
           if (contourArea(contours.at(i)) > 2000 && contourArea(contours.at(i)) < 10000 ) {
                color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
                result[0] = matchShapes(contours[i], shape_contours[0][0], 1, 0);
                result[1] = matchShapes(contours[i], shape_contours[1][0], 1, 0);
                result[2] = matchShapes(contours[i], shape_contours[2][0], 1, 0);

                if (result[0] == std::min(result[0], (std::min(result[1], result[2])))) {
                    if (result[0] < 0.01) {
                        drawContours(drawnEdges, contours, (int)i, color, 2, LINE_8, hierarchy, 0);
                        putText(drawnEdges, "cloud", contours.at(i).at(0), 1, 2, color);
                        }
                    }
                else if (result[1] == std::min(result[1], (std::min(result[0], result[2])))) {
                    if (result[1] < 0.01) {
                        drawContours(drawnEdges, contours, (int)i, color, 2, LINE_8, hierarchy, 0);
                        putText(drawnEdges, "cross", contours.at(i).at(0), 1, 2, color);
                        }
                    }
                else if (result[2] == std::min(result[2], (std::min(result[1], result[0])))) {
                    if (result[2] < 0.01) {
                        drawContours(drawnEdges, contours, (int)i, color, 2, LINE_8, hierarchy, 0);
                        putText(drawnEdges, "straight", contours.at(i).at(0), 1, 2, color);
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