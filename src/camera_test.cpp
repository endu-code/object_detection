#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;


void MyLine(Mat img, Point start, Point end){
    int thickness = 4;
    int lineType = LINE_8;

    line(img, start, end, Scalar(0, 0, 0), thickness, lineType);
}



int main(int argc, char** argv)
{
    VideoCapture cap(2);
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap.open(2))
        return 0;
    
    // Mat cloud = imread("/home/dario/catkin_ws/src/endu_robotics/sample/Cloud Piece.JPG");
    // Mat cross = imread("/home/dario/catkin_ws/src/endu_robotics/sample/Cross Piece.JPG");
    // Mat straight = imread("/home/dario/catkin_ws/src/endu_robotics/sample/Straight Piece.JPG");

    // Load Test Image for shape detection
    Mat TestImage = imread("/home/dario/catkin_ws/src/endu_robotics/sample/Test.png");

    // Load sample images and store in Array
    Mat shapes[3], shapes_bin[3];
    shapes[0] = imread("/home/dario/catkin_ws/src/endu_robotics/sample/Cloud Piece.JPG");
    shapes[1] = imread("/home/dario/catkin_ws/src/endu_robotics/sample/Cross Piece.JPG");
    shapes[2] = imread("/home/dario/catkin_ws/src/endu_robotics/sample/Straight Piece.JPG");

    // Apply Binary threshold to sample shapes
    threshold(shapes[0], shapes_bin[0], 175, 255, 1);
    threshold(shapes[1], shapes_bin[1], 175, 255, 1);
    threshold(shapes[2], shapes_bin[2], 175, 255, 1);


    // Show thresholded samples
    namedWindow("cloud", WINDOW_AUTOSIZE);
    imshow("cloud", shapes_bin[0]);
    namedWindow("cross", WINDOW_AUTOSIZE);
    imshow("cross", shapes_bin[1]);
    namedWindow("straight", WINDOW_AUTOSIZE);
    imshow("straight", shapes_bin[2]);
    
    for(;;)
    {
        Mat frame, frame_bin;
          
        // cap >> frame;

        frame = TestImage;

        threshold(frame, frame_bin, 175, 255, 1);
        //   Point pt1, pt2;
        //   pt1.x = 100;
        //   pt1.y = 100;
        //   pt2.x = 600;
        //   pt2.y = 600;

        //   MyLine(frame, pt1, pt2);

          if( frame.empty() ) break; // end of video stream
          imshow("this is you, smile! :)", frame_bin);
          if( waitKey(10) == 27 ) break; // stop capturing by pressing ESC 
    }
    // the camera will be closed automatically upon exit
    // cap.close();
    return 0;
}