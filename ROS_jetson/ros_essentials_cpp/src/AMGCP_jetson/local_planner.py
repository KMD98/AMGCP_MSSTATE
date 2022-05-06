#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from ros_essentials_cpp.msg import RTK_corrections,motor_rpm,operation_modes,corrected_pose
import numpy as np
from math import degrees, radians,cos, sin, sqrt
from tf.transformations import euler_from_quaternion  #using euler_from_quaternion(quaternion) function
from ros_essentials_cpp.srv import set_pose

class localPlanner:
    def __init__(self):
        rospy.init_node('local_planner', anonymous=True)
        self.node_name = rospy.get_name()
        rospy.loginfo("Started node %s" % self.node_name)
        rospy.Subscriber("/zed2i/zed_node/pose", PoseStamped, self.zed_callback)
        rospy.Subscriber("/RTK/pose_corrections", RTK_corrections, self.corrections_callback)
        rospy.Subscriber("/control/mode",operation_modes,self.ops_callback)
        self.pub = rospy.Publisher('/control/autonomous_speeds',motor_rpm,queue_size=10)
        self.goal_pub = rospy.Publisher('/local/goal',corrected_pose,queue_size=10)
        #storage arrays for zed position and orientation
        self.zed_pose = np.zeros(6)
        #AGMCP variables
        self.rotation_matrix = np.zeros((2,2)) #a matrix that rotates displacement vector to camera frame. #must be initialize in beginning operation or user request to change it
        self.displacement_vect = np.zeros(5) #stores the displacement vector during callback
        #Camera frame goal
        self.goal_vector = np.zeros(6) #add current pose of zed with transformed displacement vector to determine the goal point in camera frame.[x,y,z,roll,pitch,yaw]
        self.required_turn = 0
        #rospy sleep rate
        self.rate = rospy.Rate(20) #1hz
        #declare switch logic array
        self.sw = np.zeros(3)
        self.first_time = True

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

    def corrections_callback(self, message_gps):
        #rospy.loginfo(message_gps)
        # Store the RTK data in a vector for any future use
        self.displacement_vect[0] = message_gps.x
        self.displacement_vect[1] = message_gps.y
        self.displacement_vect[2] = message_gps.straight_line
        self.displacement_vect[3] = message_gps.turn_angle
        self.displacement_vect[4] = message_gps.current_bearing 
        # Declare the new goal vector. Declare without for loop because it's faster due to no if statements
        self.goal_vector[0:2] = np.matmul(self.rotation_matrix, self.displacement_vect[0:2]) + self.zed_pose[0:2] #transform displacement vector to camera frame and add to current pose
        #zed sees rotation from 0,180 CCW then 0,-180 CW. subtracting the turn angle volume would have the behavior negative is CCW and CW is positive.
        self.goal_vector[5] = self.zed_pose[5] - self.displacement_vect[3]
        #turn goal_vector to appropriate zed rotation frame where a complete rotation CW is 0 to -90, -90 to -180, -180 to 90, 90 to 0 where axis at -180 is the same as 180
        if abs(self.goal_vector[5]) > 180.0:
            if self.goal_vector[5] < 0.0:
                self.goal_vector[5] = 180.0 - (abs(self.goal_vector[5]) - 180.0)
            elif self.goal_vector[5] >= 0.0:
                self.goal_vector[5] = -180.0 + (self.goal_vector[5] - 180.0)

        for i in range(2,5): #we do not care for z, roll, and pitch value because our robot is UGV. We just store current zed value here
            self.goal_vector[i] = self.zed_pose[i]

    def ops_callback(self,message_ops):
        self.sw[:] = [message_ops.RTK_enable,message_ops.enable,message_ops.autonomous]

    def zedpose_reset(self):
        rospy.wait_for_service('/zed2i/zed_node/set_pose')
        try:
            frame_reset = rospy.ServiceProxy('/zed2i/zed_node/set_pose', set_pose)
            is_done = frame_reset(0,0,0,0,0,0) #reset, return 1 if success, else 0
            return is_done
        except rospy.ServiceException as e:
            rospy.loginfo("Service call to zed's set_pose service has failed: %s"%e)

    def yaw_difference(self,curr_yaw, desired_yaw):    
        #map function translates yaw in zed frame to 0-360 instead of 0-180 then-180-0 (CCW direction)
        def map_value(x, custom_range):
            return (x - custom_range[0]) * (custom_range[3] - custom_range[2]) / (custom_range[1] - custom_range[0]) + custom_range[2]
        
        #create the mapping parameters depending on direction of current yaw
        if curr_yaw >= 0.0:
            curr_param = np.array([0,180,360,180])
        elif curr_yaw < 0.0:
            curr_param = np.array([0,-180,0,180])
        if desired_yaw >= 0.0:
            desired_param = np.array([0,180,360,180])
        elif desired_yaw < 0.0:
            desired_param = np.array([0,-180,0,180])
        #get the angle difference with direction. Shortest path is not considered, and negative value means must turn CCW, and positive means CW
        temp = map_value(desired_yaw,desired_param) - map_value(curr_yaw,curr_param)
        #Now find the shortest turn direction to get to desired angle based off the angle difference with direction.
        if abs(temp) > 180: #Principle: If the robot turn by 180 and it is not there yet, the shortest distrance is in the opposite direction of rotation.
            if temp < 0:
                return temp + 360
            else:
                return temp - 360
        else:
            return temp
    
    def get_euclidean(self,curr_x,curr_y,goal_x,goal_y):
        return sqrt(pow(goal_x-curr_x,2) + pow(goal_y-curr_y,2))
    
    def move_motors(self,speed1,dir1,speed2,dir2):
        #send to the desired motor speed topic
        motor_speeds = motor_rpm()
        motor_speeds.passenger_side = speed2
        motor_speeds.passenger_dir = dir2
        motor_speeds.driver_side = speed1
        motor_speeds.driver_dir = dir1
        self.pub.publish(motor_speeds)
    
    def node_init(self):
        #write 0,1,0,1 to motor driver to ensure that the robot is not moving at the start.
        self.move_motors(0,1,0,1)
        # If RTK is not available because the operator has not indicated so, stuck in a loop until reliable and RTK heading and position is available.
        # Note that RTK should not be switched on until the robot is in the operating field and grabbed its first RTK heading and position reading. Else, RTK should be off at all , even in manual mode.
        if not self.sw[0]: #ensuring that local planner does not start until RTK is avaiable, so the AI can find the precise rotation matrix.
            pass
        elif self.sw[0]:
            #Reset pose tracking to ensure zed's frame is set at the current starting position of the robot.
            reset_is_success = self.zedpose_reset()
            count = 0
            if not reset_is_success:
                count+=1
                rospy.loginfo("ERROR CRITICAL!!! Resetting ZED frame to current position has failed!!! Trying to reset at attempt #%s" %count)
                reset_is_success = self.zedpose_reset()
            elif reset_is_success:
                rospy.loginfo("Position reset was successful. Beginning to calculate rotation matrix...")
                #Initialize the rotation_matrix at inital starting position. The robot should not be moving at this initiation and RTK should be
                # available and checked as indicated by operator.
                #Note that as long as the operator FLIPPED the RTK switch when RTK IS available, the rotation matrix will always be correct.
                temp_theta = self.displacement_vect[4] - 90.0
                self.rotation_matrix[[0,1],[0,1]] = cos(radians(temp_theta))
                self.rotation_matrix[0][1] = -1 * sin(radians(temp_theta))
                self.rotation_matrix[1][0] = sin(radians(temp_theta))
                rospy.loginfo("The inital bearing being used for rotation matrix is: %s" %self.displacement_vect[4]) #use string because loginfo only take strings
                #first time initialization is done, set the bool to false
                self.first_time = False            

    def send_goal(self):
        data = corrected_pose()
        data.x = self.goal_vector[0]
        data.y = self.goal_vector[1]
        data.z = self.goal_vector[2]
        data.roll = self.goal_vector[3]
        data.pitch = self.goal_vector[4]
        data.yaw = self.goal_vector[5]
        data.delyaw =self.required_turn
        self.goal_pub.publish(data)

    def spin(self):
        #Begin local navigation planner
        while not rospy.is_shutdown():
            self.send_goal()
            if self.first_time:
                self.node_init()
            if not self.first_time:
                if self.sw[0]: #start path tracker only if rtk is available
                    self.required_turn = self.yaw_difference(self.zed_pose[5], self.goal_vector[5]) #get yaw difference between goal and current pose
                    temp_displacement = self.get_euclidean(self.zed_pose[0],self.zed_pose[1],self.goal_vector[0],self.goal_vector[1]) #get straight line
                    #rospy.loginfo(self.required_turn) #UNCOMMENT FOR DEBUGGING PURPOSES, THIS PRINT THE ANGLE IT SUPPOSES TO TURN BY
                    move_allowed = (self.sw[1] and self.sw[2]) #Allow motor commands only when autonomous mode and enabled
                    if abs(self.required_turn) < 4.0 and move_allowed:                        
                        #move in straight line
                        if temp_displacement >= 0.2: #if bigger than 20cm then keep moving
                            self.move_motors(37,0,37,0)
                        elif temp_displacement < 0.2:
                            self.move_motors(0,1,0,1)
                    elif abs(self.required_turn) >= 4.0 and move_allowed:
                            #move by turning
                            if self.required_turn < 0.0: #Need to go CCW
                                self.move_motors(15,1,15,0) #0 spins driver side ccw and 1 spin passenger side cw. Making the vehicle turn CCW
                            elif self.required_turn >= 0.0:
                                self.move_motors(15,0,15,1) #spins/turn CW
                    elif not self.sw[1] and self.sw[2]: #Stop motors if enable is off and in autonomous mode. Automatic stop while not in autonomous mode is handled by manual receiver.
                        self.move_motors(0,0,0,0) 
                elif not self.sw[0]: #RTK is off, must terminate autonomous calculation because navigation wil be unstable.
                    if self.sw[2]: #If autonomous mode is still on, the local planner handle stopping instead of manual receiver.
                        self.move_motors(0,0,0,0) #Stop motor movement
            self.rate.sleep()

if __name__ == '__main__':
    try: 
        pilot = localPlanner()
        pilot.spin()
        pilot.move_motors(0,1,0,1) #ensuring local planner last command is to stop the motors from moving when the node is terminated
        rospy.loginfo("Autonomous operation at the local planner has been terminated.")
        rospy.loginfo("WARNING!!!WARNING!!! OPERATOR MUST FLIP RTK SWITCH OFF AFTER TERMINATION!!!")
    except rospy.ROSInterruptException:
        pass
