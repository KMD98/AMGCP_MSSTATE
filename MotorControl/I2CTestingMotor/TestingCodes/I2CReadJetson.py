from smbus import SMBus
import numpy as np
import time

 
addr = 0x6 # bus address
bus = SMBus(1) # indicates /dev/ic2-1

def readData():
	temp = bus.read_i2c_block_data(addr,0,8)
	return temp

def spin():
    motor_data = readData()
    print(motor_data)



if __name__ == '__main__':
    spin()

