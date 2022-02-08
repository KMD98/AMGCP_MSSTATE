from smbus2 import SMBus
import numpy as np
import time
 
addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/ic2-1
data = np.array([45,0,20,1]) #numpy will automatically use the minimum size datatype (byte) to store these values.
'''numb = 1
 
print ("Enter 1 for ON or 0 for OFF")
while numb == 1:
 
	ledstate = input(">>>>> ")
 
	if ledstate == "1":
		bus.write_i2c_block_data(addr,0,data) #the 0 will show up on the other end as the delimiter/start of data received
	else:
		numb = 0'''
while True:
	bus.write_i2c_block_data(addr,0,data)
	time.sleep(0.1)
