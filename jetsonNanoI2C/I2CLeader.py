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
        drone_coor = readingI2Cbus(addr_droneCoor) #a string of coordinates
        mgcp_coor =readingI2Cbus(addr_MGCPCoor) #a  string of coordinates        
        heading =float(readingI2Cbus(addr_heading)) #a string of heading value but turned to float
        drone_lat,drone_lon,drone_height,drone_hmsl = coordinateParse(drone_coor) #parsed coor and turn to float
        mgcp_lat,mgcp_lon,mgcp_height,mgcp_hmsl = coordinateParse(mgcp_coor)#parsed coor and turned into float
        print("Drone coor:", drone_lat,drone_lon,drone_height,drone_hmsl)
        print("MGCP coor:", mgcp_lat,mgcp_lon,mgcp_height,mgcp_hmsl)
        print('MGCP heading',heading)
        #write to real time sender
        #writeRTM("!" + drone_coor + "!" + mgcp_coor + "!" + str(heading) + "!" + hot_temp + "!" + cold_temp + "#")
        time.sleep(1.0)
