# Information
  - This code runs the main ROS network on the PC ubuntu 20.04. Refer to system diagram for overview of the network
  - xpath and ypath files are temporary and dependent on the drone map provided. Please run MapChooseWP.py and export the xpath and ypath into global_planner.py's folder
  - Local_planner.py is currently under construction and ros topics provided by zed_ros_wrapper can be found https://www.stereolabs.com/docs/ros/zed-node/#spatial-mapping
# Zed_ros_wrapper package
  - zed_ros_wrapper package is stored in the same src folder as ros_essentials_cpp package (contain these navigation codes).
  - run zed wrapper by roslaunch zed_wrapper zed2i.launch. Go into another terminal and do rqt_graph to see all available topics or rostopic echo /tab
  - Current topic of interest from zed_wrapper is /zed2i/zed_node/pose which give PoseStamped msg by sensor fusion, slam, and loop enclosure
  - note that /zed2i/zed_node/pose provide only the position of where the zed camera starts, not real world frame.
  - /zed2i/zed_node/pose coordinates need to be rotated 90 degree about z axis because y points left and x points forward. Our real world is y-forward, x-right
  - every newly generated displacement trajectory vector from global_planner to local_planner needs to be rotate by theta, where theta is the bearing of the head of robot frame at initial startup BEFORE ANY MOVEMENT
  - also note that zed_ros_wrapper need zed_ros_interface package before compiling catkin_make.
  - MAKE SURE zed_ros_interface, zed_ros_wrapper, ros_essentials_cpp are all in the src folder of catkin_ws before catkin_make. All three folders need to be together to work.
# Support
  - Kha Dan is the maintainer of this folder. dan@gri.msstate.edu
