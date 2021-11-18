#!/usr/bin/env python3
import rospy
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
        rospy.loginfo("Started node %s" % self.nodename)
        self.odometry_data = []
        self.new_message = False #ensure that navigation code do not execute on already processed data
        self.processing = False #ensure that callback doesnt change odometry value until navigator is done processing current odometry
        #Initialize latWP and lonWP. Make sure gcp_xpath.txt, gcp_ypath.txt, MapChooseWP, realtimeSHP,gcpfunctions,drone.json files are all in the same folder.
        #If new WP are set then all ROS nodes need to be terminated and reconnected before operation.
        self.latWP=[]
        self.lonWP=[]
        with open('gcp_xpath.txt','r') as fhandle:
            for x_coordinates in fhandle:
                self.latWP.append(float(x_coordinates))
        with open('gcp_ypath.txt','r') as fhandle:
            for y_coordinates in fhandle:
                self.lonWP.append(float(y_coordinates))
        #this node subribes to the RTK_odometry_topic which the odometry publisher publishes to
        rospy.Subscriber("RTK_odometry_topic", RTK, odometry_callback)
        
    
    def odometry_callback(self, RTK_data):
        self.odometry_data.clear()
        self.new_message = True
        #we dont really care for height so we can skip that. Note that we want new data whenever the navigator is done processing
        if not self.processing:
            self.odometry_data.append(float(RTK_data.drone_lat))
            self.odometry_data.append(float(RTK_data.drone_lon))
            self.odometry_data.append(float(RTK_data.MGCP_lat))
            self.odometry_data.append(float(RTK_data.MGCP_lon))
            self.odometry_data.append(float(RTK_data.MGCP_heading))
            rospy.loginfo(self.odometry_data)
    
    def spin(self):
        while not rospy.is_shutdown():
            if new_message:
                self.new_message = False #dont iterate again until new data comes in
                self.processing = True #do not use new data until previous data has been processed, this is the time dependent section
                #subroutines for navigating that is time dependent.
                
                self.processing = False #processing is done, new data can be collected. Put at the end of this code section
            
if __name__ == '__main__':
    try:
        navigator = NodeSubscriber()
        navigator.spin()
    except rospy.ROSInterruptException:
        pass
        
