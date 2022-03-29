#from smbus import SMBus
import numpy as np
import time
import keyboard

addr = 0x6 # bus address
bus = SMBus(1) # indicates /dev/ic2-1
previous_data = [0,1,0,1]
data = [0,1,0,1]

def writeData(data):
    global previous_data
    if previous_data != data:
        bus.write_i2c_block_data(addr,1,data)
    previous_data = data[:]

def testing():
    while True:
        key = keyboard.read_key()
        if key == 'w':
            writeData([37,1,37,1])
        elif key == 's':
            writeData([37,0,37,0])
        elif key == 'a':
            writeData([10,0,10,1])
        elif key == 'd':
            writeData([10,1,10,0])
        else:
            writeData([0,1,0,1])


if __name__ == '__main__':
	testing()
