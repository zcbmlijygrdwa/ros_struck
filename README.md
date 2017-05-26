# ros_struck
A ROS version of struck tracking



==================================   USAGE  ==================================

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



