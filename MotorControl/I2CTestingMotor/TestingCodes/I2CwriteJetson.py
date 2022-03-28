from smbus import SMBus
import numpy as np
import time


addr = 0x6 # bus address
bus = SMBus(1) # indicates /dev/ic2-1
data = np.array([0,1,0,1])

def writeData():
	#bus.write_i2c_block_data(addr,0,data)
	print(data)
	bus.write_i2c_block_data(addr,1,data.tolist())

def testing():
    '''rospy.init_node('testWritingData',anonymous = True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        writeData()
        rate.sleep()'''
    writeData()


if __name__ == '__main__':
	testing()
