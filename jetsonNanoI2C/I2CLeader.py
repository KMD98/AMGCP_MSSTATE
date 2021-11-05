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

def coordinateParse(coordinates):
    temp_list = coordinates.split(',')
    return float(temp_list[0]), float(temp_list[1]),float(temp_list[2]), float(temp_list[3]) #lat, lon, height above ellipsoid, hmsl

#def writeRTM(data): #send to real time monitoring sender uncomment when ready
    #for i in range (0,len(data)):
        #bus.write_byte(0x06,data[i]) #address of real time monitor sender

if __name__ == '__main__':
    while True:
        reading = bus.read_i2c_block_data(addr_droneCoor, 0, 16)
        print(reading)
        #testing lat value being passed
        latleft = (reading[0]<<8) + reading[1]
        print(latleft)
        latright = (reading[2]<<8) + reading[3]
        print(latright)
        time.sleep(1.0)
