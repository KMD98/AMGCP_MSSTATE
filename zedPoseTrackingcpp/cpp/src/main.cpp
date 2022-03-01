/* Creator: Kha Dan
   Occupation: Research Engineer at GRI of Mississippi State University
   Code purpose: This code extract the pose of zed camera in euler and meters. It publishes to the ros topic visual_pose
*/
// Basic c++ library
#include <iostream>
#include <math.h>
// ZED includes
#include <sl/Camera.hpp>

// Import GLViewer here because it enables python print syntax
#include "GLViewer.hpp"

// Import ros and accessories
#include "ros/ros.h"
#include "ros_essentials_cpp/zed_pose.h"

// Using std namespace
using namespace std;
using namespace sl;

#define IMU_ONLY 0
const int MAX_CHAR = 128;


void parseArgs(int argc, char **argv, sl::InitParameters& param);

int main(int argc, char **argv) {

    // ROS declaration
    ros::init(argc, argv, "visual_slam");

	//create a node handle: it is reference assigned to a new node
	ros::NodeHandle n;
	//create a publisher with a topic "chatter" that will send a String message
	ros::Publisher pub = n.advertise<ros_essentials_cpp::zed_pose>("visual_pose", 1000);
	//Rate is a class the is used to define frequency for a loop. Here we send a message each two seconds.
	ros::Rate loop_rate(60); //60 message per second

    Camera zed;
    // Declare rotation and translation to store camera rotation
    sl::float3 rotation_vector;
    sl::float3 translation_vect;
    // Set configuration parameters for the ZED
    InitParameters init_parameters;
    init_parameters.coordinate_units = UNIT::METER;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;
    init_parameters.sdk_verbose = true;
    parseArgs(argc, argv, init_parameters);

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Camera Open", returned_state, "Exit program.");
        return EXIT_FAILURE;
    }

    auto camera_model = zed.getCameraInformation().camera_model;

    // Set parameters for Positional Tracking
    PositionalTrackingParameters positional_tracking_param;
    positional_tracking_param.enable_area_memory = true;
    // enable Positional Tracking
    returned_state = zed.enablePositionalTracking(positional_tracking_param);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Enabling positionnal tracking failed: ", returned_state);
        zed.close();
        return EXIT_FAILURE;
    }

    Pose camera_path;
    POSITIONAL_TRACKING_STATE tracking_state;
#if IMU_ONLY
    SensorsData sensors_data;
#endif
    while(ros::ok()) {
        if (zed.grab() == ERROR_CODE::SUCCESS) {
            ros_essentials_cpp::zed_pose pose3d;
            // Get the position of the camera in a fixed reference frame (the World Frame)
            tracking_state = zed.getPosition(camera_path, REFERENCE_FRAME::WORLD);

#if IMU_ONLY
            if (zed.getSensorsData(sensors_data, TIME_REFERENCE::IMAGE) == sl::ERROR_CODE::SUCCESS) {
                rotation_vector = sensors_data.imu.pose.getEulerAngles();
            }
#else
            if (tracking_state == POSITIONAL_TRACKING_STATE::OK) {
                // Get rotation and translation and displays it
                rotation_vector = camera_path.getEulerAngles();
                for(int i = 0;i<3;i++){
                    rotation_vector[i] = (rotation_vector[i]*180.0)/M_PI;
                }
                translation_vect = camera_path.getTranslation();
                pose3d.x = translation_vect[0];
                pose3d.y = translation_vect[1];
                pose3d.z = translation_vect[2];
                pose3d.pitch = rotation_vector[0];
                pose3d.roll = rotation_vector[1];
                pose3d.yaw = rotation_Vector[2];
                pub.publish(pose3d);
                /* Uncomment for debugging
                cout<<rotation_vector[0]<<", "<< rotation_vector[1] << ", " << rotation_vector[2] << endl;
                cout<<translation_vect[0]<<", "<< translation_vect[1] << ", " << translation_vect[2] << endl;*/
            }

#endif

        } else
            sleep_ms(1);
        loop_rate.sleep();
    }
    zed.disablePositionalTracking();
    zed.close();
    return EXIT_SUCCESS;
}

void parseArgs(int argc, char **argv, sl::InitParameters& param) {
    if (argc > 1 && string(argv[1]).find(".svo") != string::npos) {
        // SVO input mode
        param.input.setFromSVOFile(argv[1]);
        cout << "[Sample] Using SVO File input: " << argv[1] << endl;
    } else if (argc > 1 && string(argv[1]).find(".svo") == string::npos) {
        string arg = string(argv[1]);
        unsigned int a, b, c, d, port;
        if (sscanf(arg.c_str(), "%u.%u.%u.%u:%d", &a, &b, &c, &d, &port) == 5) {
            // Stream input mode - IP + port
            string ip_adress = to_string(a) + "." + to_string(b) + "." + to_string(c) + "." + to_string(d);
            param.input.setFromStream(sl::String(ip_adress.c_str()), port);
            cout << "[Sample] Using Stream input, IP : " << ip_adress << ", port : " << port << endl;
        } else if (sscanf(arg.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
            // Stream input mode - IP only
            param.input.setFromStream(sl::String(argv[1]));
            cout << "[Sample] Using Stream input, IP : " << argv[1] << endl;
        } else if (arg.find("HD2K") != string::npos) {
            param.camera_resolution = sl::RESOLUTION::HD2K;
            cout << "[Sample] Using Camera in resolution HD2K" << endl;
        } else if (arg.find("HD1080") != string::npos) {
            param.camera_resolution = sl::RESOLUTION::HD1080;
            cout << "[Sample] Using Camera in resolution HD1080" << endl;
        } else if (arg.find("HD720") != string::npos) {
            param.camera_resolution = sl::RESOLUTION::HD720;
            cout << "[Sample] Using Camera in resolution HD720" << endl;
        } else if (arg.find("VGA") != string::npos) {
            param.camera_resolution = sl::RESOLUTION::VGA;
            cout << "[Sample] Using Camera in resolution VGA" << endl;
        }
    } else {
        // Default
    }
}

