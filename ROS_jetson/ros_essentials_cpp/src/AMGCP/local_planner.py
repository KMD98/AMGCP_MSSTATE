#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from ros_essentials_cpp.msg import AMGCP_displacement
import numpy as np
from math import degrees
from tf.transformations import euler_from_quaternion  #using euler_from_quaternion(quaternion) function

class pathPlanner:
    def __init__(self):
        rospy.init_node('local_planner', anonymous=True)
        self.node_name = rospy.get_name()
        rospy.loginfo("Started node %s" % self.node_name)
        rospy.Subscriber("/zed2i/zed_node/pose", PoseStamped, self.zed_callback)
        rospy.Subscriber("amgcp_goalDisplacement", AMGCP_displacement, self.displacement_callback)
        #storage arrays for zed position and orientation
        self.zed_pose = np.zeros(6)
        #AGMCP variables
        self.rotation_matrix = np.zeros((2,2)) #a matrix that rotates displacement vector to camera frame
        self.displacement_vect = np.zeros(2) #stores the displacement vector during callback
        #Camera frame goal
        self.goal_vector = np.zeros(6) #add current pose of zed with transformed displacement vector to determine the goal point in camera frame.[x,y,z,roll,pitch,yaw]

    def zed_callback(self, message_zed):
        #grab zed quaternion orientation and convert to euler
        temp_euler = euler_from_quaternion([message_zed.pose.orientation.x,message_zed.pose.orientation.y,message_zed.pose.orientation.z,message_zed.pose.orientation.w])
        for i in range(3,6):
            self.zed_pose[i] = round(degrees(temp_euler[i - 3]),2)
        #grab the position and store it in zed pose
        self.zed_pose[0] = message_zed.pose.position.x
        self.zed_pose[1] = message_zed.pose.position.y
        self.zed_pose[2] = message_zed.pose.position.z
        #rospy.loginfo(self.zed_pose) #uncomment for debugging

    def displacement_callback(self, message_gps):
        #rospy.loginfo(message_gps)
        self.displacement_vect[0] = message_gps.straight_line
    
    def spin(self):
        rospy.spin()



if __name__ == '__main__':
    pilot = pathPlanner()
    pilot.spin()
