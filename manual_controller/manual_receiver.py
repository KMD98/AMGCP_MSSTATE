#!/usr/bin/python

import serial, string

output = ''
data =[]
ser = serial.Serial('/dev/ttyUSB0', 19200, 8, 'N', 1, timeout=1)
while True:
  #print("----")
  while output != b'!': #we use byte conversion here because data received from xbee is in byte format, and python 3 does not automatically converts char because a char is not a byte unlike c++
    output = ser.read(1)
  while output != b'#':
    data.append(output)
    output = ser.read(1)
  data.append(output)
  print(data)
  data =[]
  output = " "
