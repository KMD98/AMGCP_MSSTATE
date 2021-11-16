#!/usr/bin/env python3
import rospy
from ros_essentials_cpp.msg import RTK

def odometry_reading(RTK_data):
    '''rospy.loginfo("new RTK drone data received: (%.7f, %.7f, %.2f)", 
        RTK_data.drone_lat,RTK_data.drone_lon,RTK_data.drone_height)
    rospy.loginfo("new RTK MGCP data received (%.7f, %.7f, %.3f, %.2f)", 
        RTK_data.MGCP_lat,RTK_data.MGCP_lon,RTK_data.MGCP_height,RTK_data.MGCP_heading)'''
    rospy.loginfo(RTK_data)
def navigator():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('navigator', anonymous=True)

    rospy.Subscriber("RTK_odometry_topic", RTK, odometry_reading)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    navigator()
