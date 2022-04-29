#!/usr/bin/python

import serial, string

output = " "
data =[]
ser = serial.Serial('/dev/ttyUSB0', 19200, 8, 'N', 1, timeout=1)
while True:
  print "----"
  while output != '!':
    output = ser.read(1)
  while output!='#':
    data.append(output)
    output = ser.read(1)
  data.append(output)
  print data
  data =[]
  output = " "