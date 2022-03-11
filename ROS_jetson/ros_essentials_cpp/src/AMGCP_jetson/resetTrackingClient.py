#!/usr/bin/env python
import rospy
from ros_essentials_cpp.srv import reset_tracking

def testing():
    rospy.wait_for_service('/zed2i/zed_node/reset_tracking')
    try:
        tracking_reset = rospy.ServiceProxy('/zed2i/zed_node/reset_tracking', reset_tracking)
        done = tracking_reset() #reset
        return done
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s"%e)

if __name__ == '__main__':
    rospy.init_node("testingreset") #node must be named in order to use rospy.loginfo
    is_sucess = testing()
    rospy.loginfo("%s"%is_sucess)
    
