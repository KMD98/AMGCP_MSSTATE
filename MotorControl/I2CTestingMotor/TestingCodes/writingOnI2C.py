#!/usr/bin/env python3
from smbus2 import SMBus
import numpy as np
import time
import rospy


addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/ic2-1
data = np.array([35,0,20,1])

def writeData():
	#bus.write_i2c_block_data(addr,0,data)
	bus.write_i2c_block_data(addr,1,data)

def testing():
    rospy.init_node('testWritingData',anonymous = True)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        writeData()
        rate.sleep()


if __name__ == '__main__':
	try:
		testing()
	except rospy.ROSInterruptException:
		pass
