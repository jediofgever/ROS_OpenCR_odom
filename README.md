# ROS_OpenCR_odom
A interface for recieving odometry messaging from OpenCR and publishing them as ROS messages.


Welcome to the ROS_OpenCR_odom wiki!

A simple Package to pass odometry messages via ROS publisher.

## ## # Usage

install `rosserial` to be able to communicate with OpenCR 


`cd catkin_ws/src`

`git clone https://github.com/ros-drivers/rosserial`

`cd ..`

`catkin_make`

clone this repository to your` catkin_ws/src`


`cd catkin_ws/src`

`git clone https://github.com/jediofgever/ROS_OpenCR_odom.git`

`cd ..`

`catkin_make`

create executable of python file if it fails to start

sudo chmod +x catkin_ws/src/ROS_OpenCR_odom/teleop_twist_keyboard/teleop_twist_keyboard.py


## COMPILE 
upload odometry.ino file to your board 

start rosserial
`roscore`

`rosrun rosserial_python serial_node.py /dev/ttyACM0`

launch your command interface(keyboard in this case)
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

## VISUALIZE 

`rosrun rviz rviz`
add a tf to see coordinate transformation between `odom` and `base_link` frames


