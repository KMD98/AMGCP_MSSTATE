from smbus import SMBus
import time
addr_droneCoor = 0x08
addr_MGCPCoor = 0x09
addr_heading = 0x07
bus = SMBus(1)
    
#I2C subroutines for master and send back data from each slave
def readingI2Cbus(addr):
    #grab coordinates
    data = bus.read_i2c_block_data(addr,0,19);
    print(data)
    print(type(data[0]))
    #return ''.join(data)

#def writeRTM(data): #send to real time monitoring sender uncomment when ready
    #for i in range (0,len(data)):
        #bus.write_byte(0x06,data[i]) #address of real time monitor sender

if __name__ == '__main__':
    while True:
        readingI2Cbus(addr_droneCoor)
        '''#testing lat value being passed
        latleft = (reading[0]<<8) + reading[1]
        print(latleft)
        latright = (reading[2]<<8) + reading[3]
        print(latright)'''
        time.sleep(1.0)
