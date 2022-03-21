#!/usr/bin/env python
import rospy
from ros_essentials_cpp.srv import set_pose

def testing():
    rospy.wait_for_service('/zed2i/zed_node/set_pose')
    try:
        frame_reset = rospy.ServiceProxy('/zed2i/zed_node/set_pose', set_pose)
        done = frame_reset(0,0,0,0,0,0) #reset
        return done
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s"%e)

if __name__ == '__main__':
    rospy.init_node("testingreset") #node must be named in order to use rospy.loginfo
    is_sucess = testing()
    if is_sucess:
    	rospy.loginfo("we did it biiiitchhh")
    rospy.loginfo("%s"%is_sucess)
    
