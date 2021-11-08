from smbus import SMBus
import time
addr_droneCoor = 0x08
addr_MGCPCoor = 0x09
addr_heading = 0x07
bus = SMBus(1)
    
#I2C subroutines for master and send back data from each slave
def readingI2Cbus(addr):
    #grab coordinates
    temp = bus.read_i2c_block_data(addr,0,19)
    bytetoInt(temp)
    #return ''.join(temp)

def bytetoInt(temp):
    data = []
    i = 0
    while i < 17:
        data[i] = temp[i]<<8 + temp[i+1]
        i = i + 2
    print(data)
    #return char array
    
#def writeRTM(data): #send to real time monitoring sender uncomment when ready
    #for i in range (0,len(data)):
        #bus.write_byte(0x06,data[i]) #address of real time monitor sender

if __name__ == '__main__':
    while True:
        readingI2Cbus(addr_droneCoor)
        time.sleep(1.0)
