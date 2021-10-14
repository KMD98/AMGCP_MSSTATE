#DEPRECATED
#Before parsing, generate json file from shp files with time stamp at https://mapshaper.org/ the website is free
import gcpfunctions
from gcpfunctions import findSmallestandBiggest, createGCPlongpath,createGCPlatpath, findMaxdistance
import json
import matplotlib.pyplot as plt
#Change file name here as desired
file_name = input("File name:")
with open(file_name) as json_file:
    data = json.load(json_file)

#store the size of data points into file and offset it to 0
data_size = len(data['features'])

#declare lists to store coordinates
x_coor = []
y_coor = []
time_stamp = []
#longitude is x and latitude is y. Latitude measures angular distance from the equator to a point north or south of the equator.
# While longitude is an angular measure of east/west from the Prime Meridian. ... Latitude values increase or decrease along the vertical axis, the Y axis.
# Longitude changes value along the horizontal access, the X axis. Format of geojson is coordinates: longitude, latitude
for i in range(data_size):
    x_coor.append(data['features'][i]['geometry']['coordinates'][0])
    y_coor.append(data['features'][i]['geometry']['coordinates'][1])
    time_stamp.append(data['features'][i]['properties']['Timestamp'][0:19])

#find the largest and smallest coordinates. The realtimeplotSHP.py told us that it is moving right to left or left to right
(smallestx,largestx)=findSmallestandBiggest(x_coor)
(smallesty,largesty)=findSmallestandBiggest(y_coor)
# find maximum x distance traveled (longitude)
maximum_x_dist = findMaxdistance(smallestx,largestx)
#find the desired path for gcp
#Reintialize xpathgcp for now. Later on, initialize gcp_xpathWP through interactive graph via mappingJSON.py.
# realtimeplotSHP would just take that exported value for xpathgcp and use it. The final interactive function would
# have an export function so realtimeplotSHP.py can use that to read it.
gcp_xpath = [88.772414, 88.772847,88.773402,88.774018,88.774428]
separation_size_wp = len(gcp_xpath)
gcp_ypath = createGCPlatpath(33.47025,separation_size_wp - 1)
#calculate the velocity of gcp for each point interval. the gcpfunctions.py has the velocity function. The
#gcpfunctions.calcvelocity will return a velocity list for most intervals in the entire path
#(interval with 0s time are dropped because we cannot divide by 0)
drone_velocity = gcpfunctions.calcVelocity(y_coor,x_coor,time_stamp)
running_sum = 0
#convert from km/s to m/s
for i in range(len(drone_velocity)):
    drone_velocity[i] = drone_velocity[i] * 1000
    running_sum = running_sum + drone_velocity[i]
avg_velocity = running_sum / len(drone_velocity)
print("Average drone velocity is:",avg_velocity, "m/s")
#calculate distance between each waypoint. They are all equally spaced
distanceWP = gcpfunctions.getDistanceFromLatLonInKm(gcp_ypath[0],gcp_xpath[0],gcp_ypath[1],gcp_xpath[1]) * 1000
print("The distance between waypoints is:",distanceWP,"m")


#Plot gcp and drone flight path with LBO
circleGCP = []
for i in range(len(gcp_xpath)):
    #create the circle spline for each gcp lat and long coordinates and store it in a list
    circleGCP.append(plt.Circle((gcp_xpath[i], gcp_ypath[i]), 0.00005, facecolor='none', edgecolor = 'r'))

#You can save figures if you want to
fig, ax = plt.subplots() # note we must use plt.subplots, not plt.subplot because add_patch is part of the subplot class
ax.set_ylabel("Latitude")
ax.set_xlabel("Longitude")
#draw all the circle splines
for i in range(len(circleGCP)):
    ax.add_patch(circleGCP[i])
#plot the drone path
ax.plot(x_coor,y_coor)
#plot the gcp path in the map
#ax.scatter(gcp_xpath,gcp_ypath)
#making sure all axis are equal in pixels so we dont have an ellipsoid looking circle
plt.show()
