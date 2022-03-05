#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from ros_essentials_cpp.msg import AMGCP_displacement,motor_odometry
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
        self.pub = rospy.Publisher('motor_speeds',motor_odometry,queue_size=10)
        #storage arrays for zed position and orientation
        self.zed_pose = np.zeros(6)
        #AGMCP variables
        problem, --->self.rotation_matrix = np.zeros((2,2)) #a matrix that rotates displacement vector to camera frame
        self.displacement_vect = np.zeros(5) #stores the displacement vector during callback
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
        # Store the RTK data in a vector for any future use
        self.displacement_vect[0] = message_gps.x
        self.displacement_vect[1] = message_gps.y
        self.displacement_vect[2] = message_gps.straight_line
        self.displacement_vect[3] = message_gps.turn_angle
        ---->self.displacement_vect[4] = message_gps.current_bearing #must be initialize only when camera malfunctioned or in the beginning. Fix this
        # Declare the new goal vector. Declare without for loop because it's faster due to no if statements
        self.goal_vector[0:2] = np.matmul(self.rotation_matrix, self.displacement_vect[0:2]) + self.zed_pose[0:2] #transform displacement vector to camera frame and add to current pose
        self.goal_vector[5] = self.zed_pose[5] + self.displacement_vect[3]
        for i in range(2,5): #we do not care for z, roll or pitch value because our robot is UGV
            self.goal_vector[i] = self.zed_pose[i]
    
    def yaw_difference(self,curr_yaw, desired_yaw):
        return desired_yaw - curr_yaw
    
    def get_shortest_path(self,curr_x,curr_y,goal_x,goal_y):
        return 3
    
    def spin(self):
        while not rospy.is_shutdown:
            motor_speeds = motor_odometry()
            temp_yawdiff = self.yaw_difference(self.zed_pose[5], self.goal_vector[5])
            if temp_yawdiff < 4.0:
                #move in straight line
            else:
                #move by turning



if __name__ == '__main__':
    pilot = pathPlanner()
    pilot.spin()
