#!/usr/bin/env python3
from smbus2 import SMBus
import numpy as np
import time
from ros_essentials_cpp.msg import IoTSensor
import rospy
 
addr = 0x6 # bus address
bus = SMBus(1) # indicates /dev/ic2-1

def readData():
	temp = bus.read_i2c_block_data(addr,0,8)
	return temp

def spin():
    rospy.init_node('rpmRead10Hz',anonymous = True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        motor_data = readData()
        rospy.loginfo(motor_data)
        rate.sleep()


if __name__ == '__main__':
	try:
		spin()
	except rospy.ROSInterruptException:
		pass
