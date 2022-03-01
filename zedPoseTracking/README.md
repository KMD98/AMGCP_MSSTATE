# DEPRECATED. USE ROS WRAPPER PROVIDED BY STEREOLABS
# ZED SDK - Positional Tracking

This sample shows how to get the camera pose in a world reference.

## Getting Started
 - Get the latest [ZED SDK](https://www.stereolabs.com/developers/release/) and [pyZED Package](https://www.stereolabs.com/docs/app-development/python/install/)
 - Check the [Documentation](https://www.stereolabs.com/docs/)
 
## Run the program

      python "positional_tracking.py" for positional tracking with GL viewer
      python pose_trackingTest.py for positional tracking with euler angles and xyz translation vector in right hand z up config. No usage of GL viewer
      python zedpose.py to run as a ROS Noetic node that publishes zed 3d pose and 3d point cloud to a topic.

### Features
 - An OpenGL window displays the camera path in a 3D window but there is a lot of latency
 - path data, translation and rotation, are displayed

## Support
 - If you need assistance go to our Community site at https://community.stereolabs.com/
 - Most of the code is modified by Kha Dan. Please reach out to me at khadan.manh@gmail.com
