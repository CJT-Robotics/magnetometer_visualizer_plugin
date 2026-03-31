cmake_minimum_required(VERSION 3.0.2)
project(magnetometer_visualizer_plugin)

find_package(catkin REQUIRED COMPONENTS 
    rospy 
    rqt_gui 
    rqt_gui_py 
    sensor_msgs
)

catkin_package()