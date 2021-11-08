from smbus import SMBus
import time
addr_droneCoor = 0x08
addr_MGCPCoor = 0x09
addr_heading = 0x07
bus = SMBus(1)
    
#I2C subroutines for master and send back data from each slave
def readingI2CbusCoor(addr):
    #grab coordinates
    temp = bus.read_i2c_block_data(addr,0,19)
    bytetoIntCoor(temp)
    #return ''.join(temp)

def bytetoIntCoor(temp):
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
    count = 0
    while count <=6:
        if count < 6:
            coor.append(float(data[count] + "." + data[count+1] + data[count+2]))
            count+=3
        elif count == 6:
            coor.append(float(data[count] + "." + data[count+1]))
            break
    data.clear()
    print(coor)
    #return char array
    
#def writeRTM(data): #send to real time monitoring sender uncomment when ready
    #for i in range (0,len(data)):
        #bus.write_byte(0x06,data[i]) #address of real time monitor sender

if __name__ == '__main__':
    while True:
        readingI2CbusCoor(addr_droneCoor)
        time.sleep(1.0)
