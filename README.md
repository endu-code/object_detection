@mainpage Object detection for Robotics


This Package is meant to serve as a template for object detection for a robotics application.
In this package a multithreaded ROS node is launched to capture frames from a 2D camera and process those images in order
to detect shapes and match them against a set of samples. From those samples the x and y coordinates in the camera frame 
will be send via ROS-Message for other ROS nodes to subscribe to.

The Goal is that those coordinates can be used for picking up workpieces in an industrial application.


@author Dario Aeschlimann (dario.aeschlimann@paoluzzo.ch)

![Paoluzzo Logo](src/object_detection/doc/img/Pao_Logo.jpg)
