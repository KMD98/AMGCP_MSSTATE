#!/usr/bin/env python
# Code info: global planner for UGV
# Creator: Kha Dan Research Engineer
# All metrics are in meter.
import rospy
from math import sin,cos,sqrt,atan2,radians,degrees
from ros_essentials_cpp.msg import drone_RTKpose, AMGCP_RTKpose, RTK_corrections
import utm
import numpy as np
class NodeSubscriber:   
    def __init__(self):
        #initialize node
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'navigator' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('global_planner', anonymous=True)
        self.node_name = rospy.get_name()
        rospy.loginfo("Started node %s" % self.node_name)
        self.odometry_AMGCPdata = np.array([0,0,0,0])
        self.odometry_dronedata = np.array([0,0,0])
        self.new_message = False #ensure that navigation code do not execute on already processed data
        self.processing = False #ensure that callback doesnt change odometry value until navigator is done processing current odometry
        #Initialize latWP and lonWP. Make sure gcp_xpath.txt, gcp_ypath.txt, MapChooseWP, realtimeSHP,gcpfunctions,drone.json files are all in the same folder.
        #If new WP are set then all ROS nodes need to be terminated and reconnected before operation. ALso make the path to the file is correct
        self.latWP=[] #y axis. Gotta use lists because waypoints are dynamic. These are already in UTM
        self.lonWP=[] #x axis
        self.counter = 0
        self.drone_at_WP = False #indicator that drone has reached a WP
        self.camera_footprint = 30 #3000cm or 30m is assumed right now.
        self.R = 6378.0 #radius of the earth in km
        self.waypoint_angle = 0 #azimuth in degrees
        self.heading_error = 0 #angle between waypoint and robot heading in degrees
        with open('/home/jetson/catkin_ws/src/ros_essentials_cpp/src/AMGCP_jetson/gcp_xpath.txt','r') as fhandle:
            for x_coordinates in fhandle:
                self.lonWP.append(float(x_coordinates))
        with open('/home/jetson/catkin_ws/src/ros_essentials_cpp/src/AMGCP_jetson/gcp_ypath.txt','r') as fhandle:
            for y_coordinates in fhandle:
                self.latWP.append(float(y_coordinates))

        #this node subribes to the amgcp_RTKpose topic which the RTK publisher publishes to
        rospy.Subscriber("/RTK/amgcp_RTKpose", AMGCP_RTKpose, self.amgcp_callback)
        #this node also subscribes to drone_RTKpose topic
        rospy.Subscriber("/RTK/drone_RTKpose", drone_RTKpose,self.drone_callback)
        #this node publishes to amgcp_goalDisplacement topic
        self.ugv_pub = rospy.Publisher('/RTK/pose_corrections', RTK_corrections, queue_size=10)
        
    def amgcp_callback(self, amgcp_data):
        #we dont really care for height so we can skip that. Note that we want new data whenever the navigator is done processing
        if not self.processing:
            amgcp_utm = utm.from_latlon(amgcp_data.amgcp_lat,amgcp_data.amgcp_lon) # returns(east,north,zone #, zone letter)
            self.odometry_AMGCPdata[0] = amgcp_utm[0] #easting,x
            self.odometry_AMGCPdata[1] = amgcp_utm[1] #northing,y
            self.odometry_AMGCPdata[2] = amgcp_data.bearing
            self.odometry_AMGCPdata[3] = amgcp_data.speed2D
            self.new_message = True
            #UNCOMMENT FOR DEBUGGING ONLY
            #rospy.loginfo(self.odometry_AMGCPdata)  #uncomment to see data from sensor suite.
    
    def drone_callback(self,drone_data):
        if not self.processing:
            drone_utm = utm.from_latlon(drone_data.drone_lat,drone_data.drone_lon) #returns(east,north,zone#, zone letter)
            self.odometry_dronedata[0] = drone_utm[0] #easting, x
            self.odometry_dronedata[1] = drone_utm[1] #northing, y
            self.odometry_dronedata[2] = drone_data.drone_hmsl
            self.new_message = True
            #UCOMMENT FOR DEBUGGING ONLY
            #rospy.loginfo(self.odometry_dronedata)

    def pathTracker(self, MGCP_x, MGCP_y, WP_x, WP_y, drone_x, drone_y):
        #this function calculate displacement between goal and robot position
        #it also calculate the azimuth, which is the angle between goal position and true north
        self.processing = True #do not use new data until previous data has been processed, this is the time dependent section
        self.MGCP_displacement = np.array(self.getDisplacement(MGCP_x,MGCP_y,WP_x,WP_y)) #displacement vector between MGCP and waypoint in m. Goal is to reduce to <10cm
        self.drone_displacement = np.array(self.getDisplacement(drone_x,drone_y,WP_x,WP_y)) #displacement vector between drone and waypoint
        self.waypoint_angle = self.getWaypointAngle(MGCP_x,MGCP_y,WP_x,WP_y) #angle between north and waypoint or MGCP desired angle
        self.heading_error = self.waypoint_angle - self.odometry_AMGCPdata[2] #degree that MGCP must turn to get to desired waypoint_angle. - is counterclock, + is clockwise. Use this as indicator wheter to turn left or right.
        #self.is_imaged_by_drone()
        #Increment waypoint if reached.
        if self.MGCP_displacement[2] < 0.15:
            self.counter = self.counter + 1
        #Uncomment for debugging
        '''rospy.loginfo("displacement to goal MGCP: %f cm", self.MGCP_displacement)
        rospy.loginfo("displacement to goal Drone: %f cm", self.drone_displacement)
        rospy.loginfo("waypoints angle/azimuth: %f degrees", self.waypoint_angle)
        rospy.loginfo("Betty's heading: %f degrees",self.waypoint_angle)
        rospy.loginfo("heading error: %f degrees", self.heading_error)'''
     
    def getDisplacement(self,x1,y1,x2,y2):
        #We do not have to use haversine because we are not travlelling past zones or a large distance.
        return np.array([x2-x1,y2-y1,sqrt(pow(x2-x1,2) + pow(y2-y1,2))]) #displacement vector in m [x direction, y direction, shortest distance displacement]
    
    def getWaypointAngle(self,x1,y1,x2,y2):
        #We can do pythagorean theorem with UTM here or utilize lat and lon from both coordinates and 
        #use haversine but the function is currently accepting east and north in m. It must be turned to lat lon
        #Below is Haversine.
        '''dLat = radians(lat2 - lat1) 
        dLon = radians(lon2 - lon1)
        rLat1 = radians(lat1)
        rLat2 = radians(lat2)
        wpAngle = degrees(atan2((sin(dLon)*cos(rLat2)),(cos(rLat1)*sin(rLat2)-sin(rLat1)*cos(rLat2)*cos(dLon))))
        if wpAngle > 360:
            wpAngle = wpAngle -360
        elif wpAngle < 360:
            wpAngle = wpAngle + 360'''
        #pythagorean. This method is valid because the area of travel is very small relative to the great sphere
        sign_indicator = np.array([x2 - x1, y2 - y1]) 
        relative_angle = degrees(atan2(sign_indicator[1],sign_indicator[0])) #note that wpAngle should always be <= 360.0
        if sign_indicator[0] > 0:
            if sign_indicator[1] > 0:
                wpAngle = 90 - relative_angle #quad I
            elif sign_indicator[1] < 0:
                wpAngle = 90 + relative_angle #quad IV
        elif sign_indicator[0] < 0:
            if sign_indicator[1] > 0:
                wpAngle = 270 + relative_angle #quad II
            elif sign_indicator[1] < 0:
                wpAngle = 180 + (90 - relative_angle) #quad III
        return wpAngle
    
    def is_imaged_by_drone(self):
        if self.drone_displacement[2] < 200.0:
            self.drone_at_WP = True
        if self.drone_at_WP:
            if self.drone_displacement[2] >= self.camera_footprint: #means that the drone has imaged and is at least a camera footprint away
                self.counter = self.counter + 1
                self.drone_at_WP = False
    
    def spin(self):
        while not rospy.is_shutdown():
            if self.new_message:
                #Declare message type object
                corrections_UGV = RTK_corrections()
                self.new_message = False #dont iterate again until new data comes in
                #subroutines for navigating that is time dependent
                self.pathTracker(self.odometry_AMGCPdata[0],self.odometry_AMGCPdata[1],self.lonWP[self.counter],self.latWP[self.counter], self.odometry_dronedata[0],self.odometry_dronedata[1]) #processing with pathtracker.
                #Ready to publish displacement vector
                corrections_UGV.x = self.MGCP_displacement[0] #x displacement
                corrections_UGV.y = self.MGCP_displacement[1] #y displacement
                corrections_UGV.straight_line = self.MGCP_displacement[2] #straight line distance between wp and current pose
                corrections_UGV.turn_angle = self.heading_error #the angle to turn from current angle to meet the desired azimuth/bearing
                corrections_UGV.current_bearing = self.odometry_AMGCPdata[2] #the current bearing
                corrections_UGV.UTM_x = self.odometry_AMGCPdata[0]
                corrections_UGV.UTM_y = self.odometry_AMGCPdata[1]
                self.ugv_pub.publish(corrections_UGV)
                self.processing = False #processing is done, new data can be collected. Put at the end of this code section
            if self.counter >=len(self.latWP):
                self.counter = 0
            
if __name__ == '__main__':
    try:
        navigator = NodeSubscriber()
        navigator.spin()
        rospy.loginfo("Finished")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
