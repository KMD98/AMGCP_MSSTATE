#!/usr/bin/env python
#This code serve as a unit test for AMGCP ros network
import rospy
import numpy as np
from ros_essentials_cpp.msg import operation_modes
def output_modes(sw):
    ops = operation_modes()
    ops.RTK_enable = sw[0]
    ops.enable = sw[1]
    ops.autonomous = sw[2]
    pub.publish(ops)

if __name__ == '__main__':
    rospy.init_node('unit_tester',anonymous=True)
    node_name = rospy.get_name()
    pub = rospy.Publisher('/control/mode', operation_modes,queue_size=10)
    rospy.loginfo('Started node %s'%node_name)
    rate = rospy.Rate(1)
    sw = np.array([1,1,1])
    while not rospy.is_shutdown():
        output_modes(sw)
        rate.sleep()
