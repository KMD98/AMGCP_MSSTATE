#!/usr/bin/env python3
import rospy
from ros_essentials_cpp.msg import RTK

class NodeSubscriber:
    def __init__(self):
        #initialize node
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('navigator', anonymous=True)
        self.node_name = rospy.get_name()
        rospy.loginfo("Started node %s" % self.nodename)
        self.odometry_data = []
        #this node subribes to the RTK_odometry_topic which the odometry publisher publishes to
        rospy.Subscriber("RTK_odometry_topic", RTK, odometry_callback)
        
    
    def odometry_callback(self, RTK_data):
        self.odometry_data.clear()
        #we dont really care for height so we can skip that
        self.odometry_data.append(float(RTK_data.drone_lat))
        self.odometry_data.append(float(RTK_data.drone_lon))
        self.odometry_data.append(float(RTK_data.MGCP_lat))
        self.odometry_data.append(float(RTK_data.MGCP_lon))
        self.odometry_data.append(float(RTK_data.MGCP_heading))
    
    def spin(self):
        while not rospy.is_shutdown():
            rospy.loginfo(self.odometry_data)
            
if __name__ == '__main__':
    try:
        navigator = NodeSubscriber()
        navigator.spin()
    except rospy.ROSInterruptException:
        pass
        
