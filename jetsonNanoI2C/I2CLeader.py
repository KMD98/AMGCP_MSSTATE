from smbus import SMBus
import time
addr_droneCoor = 0x08
addr_MGCPCoor = 0x09
addr_heading = 0x07
bus = SMBus(1)
    
#I2C subroutines for master and send back data from each slave
def readingI2CbusDrone(addr):
    #grab coordinates
    temp = bus.read_i2c_block_data(addr,0,19)
    return bytetoFloatDrone(temp)

def readingI2CBusMGCP(addr):
    temp = bus.read_i2c_block_data(addr,0,15)
    return bytetoFloatMGCP(temp)    

def readingI2CBusHeading(addr):
    temp = bus.read_i2c_block_data(addr,0,4)
    return bytetoFloatHeading(temp)

def bytetoFloatDrone(temp):
    data = []
    coor = []
    i = 0
    c = 0
    #note that data in the third entry or data[2] may lose a zero if character count is less than 4
    # make sure to append that 0 before conversion to float array
    while i <=14:
        data.append(str((temp[i]<<8) + temp[i+1]))
        i = i + 2
    #Scan the i2c data to see if any of the lat,lon, or height is negative and change value to appropriate sign
    if temp[16] == 1:
        data[0] = str(-1*int(data[0]))
    if temp[17] == 1:
        data[3] = str(-1*int(data[3]))
    if temp[18] == 1:
        data[6] = str(-1*int(data[6]))
    #make sure the 4 remaining decimal for lat and lon always has 4 characters, if not that means a leading 0 was dropped. Insert the 0
    #keep adding 0 to the front until len of 4 has been met. Not needing for heigh because heigh decimal can be represented with a single int
    while len(data[2]) < 4:
        data[2] = "0" + data[2]
    while len(data[5]) < 4:
        data[5] = "0" + data[5]
    #Now convert to float list
    count = 0
    while count <=6:
        if count < 6:
            coor.append(float(data[count] + "." + data[count+1] + data[count+2]))
            count+=3
        elif count == 6:
            coor.append(float(data[count] + "." + data[count+1]))
            break
    data.clear()
    return coor

def bytetoFloatMGCP(temp):
    data = []
    i = 0
    while i <=8:
        data.append((temp[i]<<24) + (temp[i+1]<<16) +(temp[i+2]<<8) + temp[i+3])
        i+=4
    #Divide lat and lon data by 10000000.0f and height by 1000.0f to get decimal place
    if temp[12] == 1:
        data[0] = -1.0*(float(data[0])/float(10000000.0))
    elif temp[12] == 0:
        data[0] = (float(data[0])/float(10000000.0))
    if temp[13] == 1:
        data[1] = -1.0*(float(data[1])/float(10000000.0))
    elif temp[13] == 0:
        data[1] = (float(data[1])/float(10000000.0))
    if temp[14] == 1:
        data[2] = -1.0*(float(data[2])/float(1000.0))        
    elif temp[14] == 0:
        data[2] = float(data[2])/float(1000.0)
    return data

def bytetoFloatHeading(temp):
    data = float((temp[0]<<24) + (temp[1]<<16) + (temp[2]<<8) + temp[3])/float(100.0)
    return data
#def writeRTM(data): #send to real time monitoring sender uncomment when ready
    #for i in range (0,len(data)):
        #bus.write_byte(0x06,data[i]) #address of real time monitor sender

if __name__ == '__main__':
    while True:
        drone_coor = readingI2CbusDrone(addr_droneCoor)
        #delay between reads to be definite that SDA Has pulled low
        time.sleep(0.005)
        mgcp_coor = readingI2CBusMGCP(addr_MGCPCoor)
        time.sleep(0.005)
        mgcp_heading = readingI2CBusHeading(addr_heading)
        print("Drone Coordinates: ", drone_coor)
        print("MGCP Coordinates: ", mgcp_coor)
        print("MGCP Heading: ", mgcp_heading)
        time.sleep(1.0)
