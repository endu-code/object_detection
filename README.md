

This Package is meant to serve as a template for object detection for a robotics application.
In this package a multithreaded ROS node is launched to capture frames from a 2D camera and process those images in order
to detect shapes and match them against a set of samples. From those samples the x and y coordinates in the camera frame 
will be send via ROS-Message for other ROS nodes to subscribe to.

The Goal is that those coordinates can be used for picking up workpieces in an industrial application.
The package is written to use in our own production of high-precision workpieces.

See [www.paoluzzo.ch](https://www.paoluzzo.ch/) for more information about our company!

<p align="center">
  <img src="doc/img/Pao_Logo.jpg" />
</p>


### Installation

#### Ubuntu 20.04 / ROS Noetic

**Install ROS**

follow the installation guide [here](http://wiki.ros.org/noetic/Installation) for installing ROS

**Install OpenCV**

Follow [this Tutorial](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html) for installing OpenCV

**Install Object Detection Package**

Set up a catkin workspace following [this Tutorial](http://wiki.ros.org/catkin/Tutorials), then clone the repository into the src/ folder inside your catkin workspace.
Use catkin_make to compile.

```sh
cd ~/catkin_ws/src
git clone -b master git@github.com:endu-code/object_detection.git
cd ..
catkin_make
source devel/setup.bash
```
**Run the code**

roslaunch object_detection object_detection.launch will run the example code.

```sh
roslaunch object_detection object_detection.launch
```

**Adapt the code for your needs**
