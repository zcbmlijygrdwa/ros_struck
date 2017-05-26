# ros_struck
A ROS version of struck tracking



==================================   USAGE  ==================================

Download the package, unzip the file and rename the folder to "struck". Move the folder under catkin_ws/src/

Go to "catkin_ws/" and run:

catkin_make


After building the package:

source devel/setup.bash

rosrun struckRos struckRos src/struck/config.txt



==================================   ABOUT  ==================================

A ROS topic "/struckRos_bb" will be created to publish information of the bounding box. The information is an array of float64 number. Array size is 4;

Msgs Type: std_msgs::Float64MultiArray

Four points will be published:
x: the x position of the bounding box
y: the y position of the bounding box
w: the width of the bounding box
h: the height of the bounding box



