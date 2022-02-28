#Positional Tracking with Zed2i camera in ROS Noetic
# Creator: Kha Dan Research Engineer
# Company: Mississippi State University

import sys
import pyzed.sl as sl
import numpy as np
from math import degrees
from tf.transformations import euler_from_quaternion
from ros_essentials_cpp.msg import zed_pose

class poseTracking:
    def __init__(self):
        init_params = sl.InitParameters(camera_resolution=sl.RESOLUTION.VGA, coordinate_units=sl.UNIT.METER, coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP)
                                 
        #Declare camera class
        self.zed = sl.Camera()
        status = self.zed.open(init_params)
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
        self.zed.enable_positional_tracking(tracking_params)

        self.runtime = sl.RuntimeParameters()
        self.camera_pose = sl.Pose()

        self.camera_info = self.zed.get_camera_information()
        self.translation_object = sl.Translation()
        
        #Declare arrays to store pose data
        self.rotation_vector = np.zeros(3)
        self.translation_vector = np.zeros(3)

        #Declare publisher
        rospy.init_node('visual SLAM',anonymous = True)
        self.pub = rospy.Publisher('Visual-Inertial Pose', zed_pose, queue_size=10)
    
    def poseSpin():
        #run positional tracking of zed
        while not rospy.is_shutdown():
            if self.zed.grab(self.runtime) == sl.ERROR_CODE.SUCCESS:
                temp = zed_pose()
                tracking_state = self.zed.get_position(self.camera_pose, sl.REFERENCE_FRAME.WORLD)
                if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                    self.rotation_vector = self.camera_pose.get_rotation_vector() #grab euler orientation
                    translation_address = self.camera_pose.get_translation(self.translation_object) #current position vector's address in form x,y,z right handed z up
                    for i in range(0,3): #access the addres and grab components xyz
                        self.translation_vector[i] = round(translation_address.get()[i],3)
                        self.rotation_vector[i] = degrees(round(self.rotation_vector[i],3))
                    temp.x = self.translation_vector[0]
                    temp.y = self.translation_vector[1]
                    temp.z = self.translation_vector[2]
                    temp.pitch = self.rotation_vector[0]
                    temp.roll = self.rotation_vector[1]
                    temp.yaw = self.rotation_vector[2]
                    self.pub.publish(temp)
        self.zed.close()
        


if __name__ == "__main__":
    try:
        pose3D = poseTracking()
        pose3D.poseSpin()
        rospy.loginfo("Finished")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


        

