#!/usr/bin/env python3
from smbus2 import SMBus
import numpy as np
import time
from ros_essentials_cpp.msg import IoTSensor
import rospy
 
addr = 0x9 # bus address
bus = SMBus(1) # indicates /dev/ic2-1

def readData():
	#bus.write_i2c_block_data(addr,0,data)
	temp = bus.read_i2c_block_data(addr,0,12)
	return temp

def testing():
    rospy.init_node('testData',anonymous = True)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        temporary = readData()
        rospy.loginfo(temporary)
        rate.sleep()


if __name__ == '__main__':
	try:
		testing()
	except rospy.ROSInterruptException:
		pass

