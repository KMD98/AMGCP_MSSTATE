from smbus import SMBus
import time
addr_droneCoor = 0x08
addr_MGCPCoor = 0x09
addr_heading = 0x07
bus = SMBus(1)
    
#I2C subroutines for master and send back data from each slave
def readingI2Cbus(addr):
    #grab drone coordinates
    data = []
    #receives the length in two bytes and then convert that into an int
    high = bus.read_byte(addr)
    low = bus.read_byte(addr)
    length = (high<<8) + low
    for i in range (0,length): #need to be dynamic
        data.append(chr(bus.read_byte(addr)))
    data.pop() #remove last element delimiter
    data.remove('!') #remove all limiter in the beginning
    return ''.join(data)

while True:
    drone_coor = readingI2Cbus(addr_droneCoor)
    mgcp_coor =readingI2Cbus(addr_MGCPCoor)
    heading =readingI2Cbus(addr_heading)
    print("Drone coordinates:", drone_coor)
    print("MGCP coordinates:", mgcp_coor)
    print('MGCP heading',heading)
    time.sleep(1.0)