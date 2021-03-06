#!/usr/bin/env python3
import rospy
from math import sin,cos,sqrt,atan2,radians,degrees
from ros_essentials_cpp.msg import RTK

class NodeSubscriber:   
    def __init__(self):
        #initialize node
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'navigator' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('navigator', anonymous=True)
        self.node_name = rospy.get_name()
        rospy.loginfo("Started node %s" % self.node_name)
        self.odometry_data = []
        self.new_message = False #ensure that navigation code do not execute on already processed data
        self.processing = False #ensure that callback doesnt change odometry value until navigator is done processing current odometry
        #Initialize latWP and lonWP. Make sure gcp_xpath.txt, gcp_ypath.txt, MapChooseWP, realtimeSHP,gcpfunctions,drone.json files are all in the same folder.
        #If new WP are set then all ROS nodes need to be terminated and reconnected before operation. ALso make the path to the file is correct
        self.latWP=[] #y axis
        self.lonWP=[] #x axis
        self.counter = 0
        self.enabled = False #enabled will be set based on a gpio button connected to robot or radio....will determine later
        self.drone_at_WP = False #indicator that drone has reached a WP
        self.camera_footprint = 3000 #3000cm is assumed right now.
        self.drone_displacement = 0 #drone displacement to WP
        self.R = 6378.0 #radius of the earth in km
        self.waypoint_angle = 0 #azimuth in degrees
        self.heading_error = 0 #angle between waypoint and robot heading in degrees
        self.MGCP_displacement = 0 #MGCP distance to WP
        with open('/home/jetson/catkin_ws/src/ros_essentials_cpp/src/MGCP/gcp_xpath.txt','r') as fhandle:
            for x_coordinates in fhandle:
                self.lonWP.append(float(x_coordinates))
        with open('/home/jetson/catkin_ws/src/ros_essentials_cpp/src/MGCP/gcp_ypath.txt','r') as fhandle:
            for y_coordinates in fhandle:
                self.latWP.append(float(y_coordinates))
        #this node subribes to the RTK_odometry_topic which the odometry publisher publishes to
        rospy.Subscriber("RTK_odometry_topic", RTK, self.odometry_callback)
        
    def odometry_callback(self, RTK_data):
        #we dont really care for height so we can skip that. Note that we want new data whenever the navigator is done processing
        if not self.processing:
            self.odometry_data.clear()
            self.odometry_data.append(float(RTK_data.drone_lat))
            self.odometry_data.append(float(RTK_data.drone_lon))
            self.odometry_data.append(float(RTK_data.MGCP_lat))
            self.odometry_data.append(float(RTK_data.MGCP_lon))
            self.odometry_data.append(float(RTK_data.MGCP_heading))
            self.odometry_data.append(float(RTK_data.MGCP_groundSpeed))
            self.new_message = True
            #UNCOMMENT FOR DEBUGGING ONLY
            #rospy.loginfo(self.odometry_data)  #uncomment to see data from sensor suite.
    

    def pathTracker(self, MGCP_lat, MGCP_lon, WP_lat, WP_lon, lat_drone, lon_drone):
        #this function calculate displacement between goal and robot position
        #it also calculate the azimuth, which is the angle between goal position and true north
        self.MGCP_displacement = self.getDisplacement(MGCP_lat,MGCP_lon,WP_lat,WP_lon) #Distance between MGCP and waypoint in cm. Goal is to reduce to <10cm
        self.drone_displacement = self.getDisplacement(lat_drone,lon_drone,WP_lat,WP_lon)
        self.waypoint_angle = self.getWaypointAngle(MGCP_lat,MGCP_lon,WP_lat,WP_lon) #angle between north and waypoint or MGCP desired angle
        self.heading_error = self.waypoint_angle - self.odometry_data[4] #degree that MGCP must turn to get to desired waypoint_angle. - is counterclock, + is clockwise. Use this as indicator wheter to turn left or right.
        #Uncomment for debugging
        '''rospy.loginfo("displacement to goal MGCP: %f cm", self.MGCP_displacement)
        rospy.loginfo("displacement to goal Drone: %f cm", self.drone_displacement)
        rospy.loginfo("waypoints angle/azimuth: %f degrees", self.waypoint_angle)
        rospy.loginfo("Betty's heading: %f degrees",self.waypoint_angle)
        rospy.loginfo("heading error: %f degrees", self.heading_error)'''
     
    def getDisplacement(self,lat1,lon1,lat2,lon2):
        dLat = radians(lat2 - lat1)
        dLon = radians(lon2 - lon1)
        rLat1 = radians(lat1)
        rLat2 = radians(lat2)
        #haversine
        a = sin(dLat / 2) * sin(dLat / 2) + cos(rLat1) * cos(rLat2) * sin(dLon / 2) * sin(dLon / 2)
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        return self.R * c * 100000.0 #displacement in cm
    
    def getWaypointAngle(self,lat1,lon1,lat2,lon2):
        dLat = radians(lat2 - lat1)
        dLon = radians(lon2 - lon1)
        rLat1 = radians(lat1)
        rLat2 = radians(lat2)
        wpAngle = degrees(atan2((sin(dLon)*cos(rLat2)),(cos(rLat1)*sin(rLat2)-sin(rLat1)*cos(rLat2)*cos(dLon))))
        if wpAngle > 360:
            wpAngle = wpAngle -360
        elif wpAngle < 360:
            wpAngle = wpAngle + 360
        return wpAngle
    
    def is_imaged_by_drone(self):
        if self.drone_displacement < 50.0:
            self.drone_at_WP = True
        if self.drone_at_WP:
            if self.drone_displacement >= self.camera_footprint: #means that the drone has imaged and is at least a camera footprint away
                self.counter = self.counter + 1
                self.drone_at_WP = False
    
    def spin(self):
        while not rospy.is_shutdown():
            if self.new_message:
                self.processing = True #do not use new data until previous data has been processed, this is the time dependent section
                self.new_message = False #dont iterate again until new data comes in
                #subroutines for navigating that is time dependent.
                self.pathTracker(self.odometry_data[2],self.odometry_data[3],self.latWP[self.counter],self.lonWP[self.counter], self.odometry_data[0],self.odometry_data[1]) #processing with pathtracker.
                #Uncomment when testing hardware
                '''if self.enabled:
                    if abs(self.heading_error) < 1: #need a statement to handle the other case
                        if self.MGCP_displacement >= 50.0):
                            #send move command to low level
                            #insert move code here#
                        elif self.MGCP_displacement < 50.0:
                            #send brake command
                            ####insert brake code here####
                            self.is_imaged_by_drone() ##This function ensure drone_imaged is true then set next waypoint
                            if self.counter == (len(latWP) - 1):
                                break
                    elif abs(self.heading_error) >= 1:
                        #send command to turn the wheel to lower motor with self.heading_error as the value to turn and sign of it as direction cw or c'''
                self.processing = False #processing is done, new data can be collected. Put at the end of this code section
            
if __name__ == '__main__':
    try:
        navigator = NodeSubscriber()
        navigator.spin()
        rospy.loginfo("Finished")
    except rospy.ROSInterruptException:
        pass
        
