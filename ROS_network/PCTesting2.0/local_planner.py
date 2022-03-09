#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import PoseStamped
from ros_essentials_cpp.msg import AMGCP_displacement,motor_odometry
import numpy as np
from math import degrees, radians,cos, sin
from tf.transformations import euler_from_quaternion  #using euler_from_quaternion(quaternion) function

#pin definitions
rtk_SW = 11
operation_SW = 13
autonomous_SW = 15

class localPlanner:
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
        self.rotation_matrix = np.zeros((2,2)) #a matrix that rotates displacement vector to camera frame. #must be initialize in beginning operation or user request to change it
        self.displacement_vect = np.zeros(5) #stores the displacement vector during callback
        #Camera frame goal
        self.goal_vector = np.zeros(6) #add current pose of zed with transformed displacement vector to determine the goal point in camera frame.[x,y,z,roll,pitch,yaw]
        #rospy sleep rate
        self.rate = rospy.Rate(1) #1hz
        #declare switch logic array
        self.sw = np.zeros(3)

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
        self.displacement_vect[4] = message_gps.current_bearing 
        # Declare the new goal vector. Declare without for loop because it's faster due to no if statements
        self.goal_vector[0:2] = np.matmul(self.rotation_matrix, self.displacement_vect[0:2]) + self.zed_pose[0:2] #transform displacement vector to camera frame and add to current pose
        self.goal_vector[5] = self.zed_pose[5] + self.displacement_vect[3] #zed sees rotation from 0,180 then -180,0. Make sure you take this in consideration.
        for i in range(2,5): #we do not care for z, roll, and pitch value because our robot is UGV
            self.goal_vector[i] = self.zed_pose[i]
    
    def yaw_difference(self,curr_yaw, desired_yaw):
        return abs(desired_yaw - curr_yaw)
    
    def get_shortest_path(self,curr_x,curr_y,goal_x,goal_y):
        return 3
    
    def spin(self):
        #write 0,1,0,1 to motor driver
        initial_speed = motor_odometry()
        initial_speed.driver_side = byte(0)
        initial_speed.driver_dir = byte(1)
        initial_speed.passenger_side = byte(0)
        initial_speed.passenger_dir = byte(1)
        self.pub(initial_speed)

        # If RTK is not available because the operator has not indicated so, stuck in a loop until reliable and RTK heading and position is available.
        # Note that RTK should not be switched on until the robot is in the operating field and grabbed its first RTK heading and position reading. Else, RTK should be off at all , even in manual mode.
        while not GPIO.input(rtk_SW): #ensuring that local planner does not start until RTK is avaiable, so the AI can find the precise rotation matrix.
            rospy.loginfo("No RTK, please flip the RTK switch when RTK is available. Ensure that RTK is available before autonomous operation")
            self.rate.sleep()
        rospy.loginfo("The operator has indicated RTK is available, please switch ON autonomous mode and switch ON operation")

        #Initialize the rotation_matrix at inital starting position. The robot should not be moving at this initiation and RTK should be available and checked as indicated by operator.
        #Note that as long as the operator FLIPPED the RTK switch when RTK IS available, the rotation matrix will always be correct.
        if abs(self.displacement_vect[4]) <= 90.0):
            temp_theta = 90.0 - self.displacement_vect[4]
        elif abs(self.displacement_vect[4] > 90.0):
            temp_theta = self.displacement_vect[4] - 90.0 
        self.rotation_matrix[[0,1],[0,1]] = cos(radians(temp_theta))
        self.rotation_matrix[0][1] = -1 * sin(radians(temp_theta))
        self.rotation_matrix[1][0] = sin(radians(temp_theta))
        rospy.loginfo("The inital bearing being used for rotation matrix is: %s" %self.displacement_vect[4]) #use string because loginfo only take strings
        
        #Begin local navigation planner
        while not rospy.is_shutdown:
            self.sw[:] = np.array([GPIO.input(rtk_SW),GPIO.input(operation_SW),GPIO.input(autonomous_SW)])
            if self.sw[2]: #autonomous mode is selected. Keep in mind that the RTK is assumed available in the first iteration
                motor_speeds = motor_odometry()
                if (self.sw[0] and self.sw[1]):
                    temp_yawdiff = self.yaw_difference(self.zed_pose[5], self.goal_vector[5])
                    if temp_yawdiff < 4.0:
                        #move in straight line
                        motor_speeds.passenger_side = 37
                        motor_speeds.passenger_dir = 1
                        motor_speeds.driver_side = 37
                        motor_speeds.driver_dir = 1
                        self.pub(motor_speeds)
                    else:
                        #move by turning
                elif not self.sw[0]: #RTK is off, must terminate autonomous motion because navigation wil be unstable.
                    rospy.login("ERROR: Operator has indicated that RTK is not available. Ensure RTK is available before switching RTK ON")
                    #write 0 speed to motor to stop moving. This wont be entered again until the node is ran again.
                    motor_speeds.passenger_side = 0
                    motor_speeds.passenger_dir = 1
                    motor_speeds.driver_side = 0
                    motor_speeds.driver_dir = 1
                    self.pub(motor_speeds)
                elif self.sw[0] and not self.sw[1]: #RTK is on or got back on again, but operation mode is OFF. Tell operator that he/she can begin autonomous by turning ON operation switch
                    motor_speeds.passenger_side = 0
                    motor_speeds.passenger_dir = 1
                    motor_speeds.driver_side = 0
                    motor_speeds.driver_dir = 1
                    self.pub(motor_speeds)
                    rospy.loginfo("Turn operation switch ON to begin autonomous operation")

if __name__ == '__main__':
    try: 
        GPIO.setmode(GPIO.BOARD) #Set the configuration of GPIO, we are following pin numbers on board
        GPIO.setup([rtk_SW,operation_SW,autonomous_SW], GPIO.IN) #set the SWs as inputs
        pilot = localPlanner()
        pilot.spin()
        rospy.loginfo("Autonomous operation at the local planner has been terminated.")
        GPIO.cleanup() #cleanup all pins if spin gets terminated.
    except rospy.ROSInterruptException:
        pass
