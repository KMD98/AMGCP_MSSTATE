###Positional Tracking with Zed2i camera.
# Creator: Kha Dan Research Engineer
# Company: Mississippi State University

import sys
import pyzed.sl as sl
import numpy as np
from math import degrees
from tf.transformations import euler_from_quaternion
if __name__ == "__main__":

    init_params = sl.InitParameters(camera_resolution=sl.RESOLUTION.VGA,
                                 coordinate_units=sl.UNIT.METER,
                                 coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP)
                                 
    #Declare camera class
    zed = sl.Camera()
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    #create class object.
    initial_position = sl.Transform()
    # Set the initial positon of the Camera Frame at 0.5m above the World Frame
    initial_translation = sl.Translation()
    initial_orientation = sl.Orientation()
    initial_translation.init_vector(0,0,0.5)
    initial_orientation.init_vector(0,0,0,1) #feed gps rtk heading before start and calculate the angle need to be turn in quaternion
    initial_position.set_translation(initial_translation) #set the object at 0.5m above ground
    initial_position.set_orientation(initial_orientation) #set the object at x degrees from y axis


    tracking_params = sl.PositionalTrackingParameters(initial_position) #begin tracking with the inital position
    zed.enable_positional_tracking(tracking_params)

    runtime = sl.RuntimeParameters()
    camera_pose = sl.Pose()

    camera_info = zed.get_camera_information()
    translation_object = sl.Translation()
    
    #Declare arrays to store pose data
    rotation_vector = np.zeros(3)
    translation_vector = np.zeros(3)
    #run positional tracking of zed
    while True:
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            tracking_state = zed.get_position(camera_pose, sl.REFERENCE_FRAME.WORLD)
            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                rotation_vector = camera_pose.get_rotation_vector() #grab euler orientation
                translation_address = camera_pose.get_translation(translation_object) #current position vector's address in form x,y,z right handed z up
                for i in range(0,3): #access the addres and grab components xyz
                    translation_vector[i] = round(translation_address.get()[i],3)
                    rotation_vector[i] = degrees(round(rotation_vector[i],3))
                print(rotation_vector)
                print(translation_vector)
                print('\n')

    zed.close()

