#Before parsing, generate json file from shp files with time stamp at https://mapshaper.org/ the website is free
#Creator: Kha Dan Research Engineer
# Software Purpose: Generate drone trajectory and let user pick waypoints for UGV to travel
# publishes chosen waypoint to two .txt files (x path, ypath)
import gcpfunctions
from gcpfunctions import findSmallestandBiggest, createGCPlongpath,createGCPlatpath, findMaxdistance
import json
import matplotlib.pyplot as plt
import sys
import utm
import numpy as np
import math
#Change file name here as desired
file_name = input("File name:")
with open(file_name) as json_file:
    data = json.load(json_file)

#store the size of data points into file and offset it to 0
data_size = len(data['features'])

#declare lists to store coordinates
long_coor = []
lat_coor = []

time_stamp = []
#longitude is x and latitude is y. Latitude measures angular distance from the equator to a point north or south of the equator.
# While longitude is an angular measure of east/west from the Prime Meridian. ... Latitude values increase or decrease along the vertical axis, the Y axis.
# Longitude changes value along the horizontal access, the X axis. Format of geojson is coordinates: longitude, latitude
for i in range(data_size):
    long_coor.append(-1*data['features'][i]['geometry']['coordinates'][0])
    lat_coor.append(data['features'][i]['geometry']['coordinates'][1])
    time_stamp.append(data['features'][i]['properties']['Timestamp'][0:19])
x_coorUTM = np.zeros(len(long_coor))
y_coorUTM = np.zeros(len(lat_coor))
corresponding_zone = np.zeros(len(long_coor))
#convert to UTM for waypoint choosing
for i in range(0,len(long_coor)):
    temp_ls = utm.from_latlon(lat_coor[i],long_coor[i])
    corresponding_zone[i] = temp_ls[2]
    x_coorUTM[i] = temp_ls[0]
    y_coorUTM[i] = temp_ls[1]
#find the largest and smallest coordinates. The realtimeplotSHP.py told us that it is moving right to left or left to right
(smallestx,largestx)=findSmallestandBiggest(long_coor)
(smallesty,largesty)=findSmallestandBiggest(lat_coor)
# find maximum x distance traveled (longitude)
maximum_x_dist = findMaxdistance(smallestx,largestx)

#calculate the velocity of gcp for each point interval. the gcpfunctions.py has the velocity function. The
#gcpfunctions.calcvelocity will return a velocity list for most intervals in the entire path
#(interval with 0s time are dropped because we cannot divide by 0)
drone_velocity = gcpfunctions.calcVelocity(lat_coor,long_coor,time_stamp)
running_sum = 0
#convert from km/s to m/s
for i in range(len(drone_velocity)):
    drone_velocity[i] = drone_velocity[i] * 1000
    running_sum = running_sum + drone_velocity[i]
avg_velocity = running_sum / len(drone_velocity)
print("Average drone velocity is:",avg_velocity, "m/s")
print("Drone path is in UTM zone:", corresponding_zone[0])
#plot the graph for user to see
plt.ylabel("y(m)")
plt.xlabel("x(m)")
plt.plot(x_coorUTM,y_coorUTM)
plt.show(block=False)
#get the mouse input using ginput function
number_of_wp = int(input("View the map and decide how many waypoints?"))
#get the yaxis position of the gcp
#gcp_yValue = float(input("Where on the y axis (latitude) will the GCP be? ")) uncomment this to get static y input
#close the figure. replotting normally close the figure but we do this to prevent any surprises
plt.close()
#Now replot drone path and let user choose WP
plt.ylabel("y(m)")
plt.xlabel("x(m)")
title_name = "Choose MGCP " + str(number_of_wp) + " waypoints"
plt.title(title_name)
plt.plot(x_coorUTM,y_coorUTM)
#store the chosen coordinates into a zipped list of tuples. We are going to unzip it later as just tuples (x,y)
print("Choose your",number_of_wp,"MGCP waypoints by left clicking on the Figure. Right click to erase the previous chosen point.")
gcp_path=plt.ginput(number_of_wp)
plt.show(block=False)
#close to prevent any surprises
plt.close()
#unzip the zipped list and store them into two tuple (x,y). We dont really care about the y path. We will pick y path
#further down the line
gcp_xpath, gcp_ypath = zip(*gcp_path)
print(gcp_xpath)
print(gcp_ypath)

#turn the path tuple into a list because our functions uses list as arguments
gcp_xpath = list(gcp_xpath)
gcp_ypath = list(gcp_ypath)
#write gcp_xpath into a file
with open('gcp_xpath.txt', 'w') as fhandle:
    for gcp_x_coordinates in gcp_xpath:
        fhandle.write('%s\n' % gcp_x_coordinates)
with open('gcp_ypath.txt', 'w') as fhandle:
    for gcp_y_coordinates in gcp_ypath:
        fhandle.write('%s\n' % gcp_y_coordinates)
#find the desired path for gcp
#Reintialize xpathgcp for now. Later on, initialize gcp_xpathWP through interactive graph via mappingJSON.py.
# realtimeplotSHP would just take that exported value for xpathgcp and use it. The final interactive function would
# have an export function so realtimeplotSHP.py can use that to read it.
#separation_size_wp = len(gcp_xpath) uncomment if using static y input
#gcp_ypath = createGCPlatpath(gcp_yValue,separation_size_wp - 1) uncomment this to have a static y input
#calculate distance between each waypoint. They are all equally spaced
distanceWP = math.sqrt(pow((gcp_ypath[1] - gcp_ypath[0]),2) + pow((gcp_xpath[1] - gcp_xpath[0]),2))
print("The distance between waypoints is:",distanceWP,"m")
#Plot gcp and drone flight path with LBO
circleGCP = []
for i in range(len(gcp_xpath)):
    #create the circle spline for each gcp lat and long coordinates and store it in a list
    circleGCP.append(plt.Circle((gcp_xpath[i], gcp_ypath[i]), 0.00005, facecolor='none', edgecolor = 'r'))
#You can save figures if you want to
fig1, ax2 = plt.subplots() # note we must use plt.subplots, not plt.subplot because add_patch belongs to the subplots class
ax2.set_ylabel("y(m)")
ax2.set_xlabel("x(m)")
ax2.set_title("Drone and MGCP Collaboration Final Map")
#draw all the circle splines
for i in range(len(circleGCP)):
    ax2.add_patch(circleGCP[i])
#plot the drone path
ax2.plot(x_coorUTM,y_coorUTM)
#plot the gcp path in the map
ax2.scatter(gcp_xpath,gcp_ypath)
#making sure all axis are equal in pixels so we dont have an ellipsoid looking circle and show
ax2.axis('equal')
plt.show()
print("The MGCP chosen x values have been exported to gcp_xpath.txt. It is in the same folder this .py file is in")
activate_simulation = input("Would you like to start the simulation for Drone and GCP collaboration (Y/N)? ")
if(activate_simulation.capitalize() == "Y"):
    #run the simulation
    stream = open("realtimeplotSHP.py")
    read_file = stream.read()
    exec(read_file)
elif(activate_simulation.capitalize() == "N"):
    print("The chosen x values are in gcp_xpath.txt. Run realtimeplotSHP.py to see the simulation for drone and gcp collaboration")
sys.exit()

