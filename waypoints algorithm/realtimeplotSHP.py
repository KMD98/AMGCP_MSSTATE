import json
import matplotlib.pyplot as plt
import gcpfunctions
import sys
#Change file name here as desired

file_name = input("File name:")
with open(file_name) as json_file:
    data = json.load(json_file)
#store the size of data points into file and offset it to 0
data_size = len(data['features'])

#declare lists to store coordinates. long is x_coor and lat is y_coor. shp and kml has long listed first.
x_coor = []
y_coor = []
time_stamp = []
for i in range(data_size):
    x_coor.append(data['features'][i]['geometry']['coordinates'][0])
    y_coor.append(data['features'][i]['geometry']['coordinates'][1])
    time_stamp.append(data['features'][i]['properties']['Timestamp'][0:19])

#Reintialize xpathgcp for now. Later on, initialize gcp_xpathWP through interactive graph via mappingJSON.py.
# realtimeplotSHP would just take that exported value for xpathgcp and use it.
# So the real code would have some type of import function and the mappingJSON would have export function
gcp_xpathWP =[]
with open('gcp_xpath.txt','r') as fhandle:
    for x_coordinates in fhandle:
        gcp_xpathWP.append(float(x_coordinates))
# Declare the gcp path. Pass in the longitude values along x-axis that we want gcp to travel on. Pass the y-axis value (latitude). Pass in how many points we want.
#We can make wp_number dynamic (user input) later on when we incorporated camera parameter.
wp_number = len(gcp_xpathWP)
gcp_ypathWP = gcpfunctions.createGCPlatpath(33.47025,wp_number - 1)
#calculate distance between each waypoint. They are all equally spaced
distanceWP = gcpfunctions.getDistanceFromLatLonInKm(gcp_ypathWP[0],gcp_xpathWP[0],gcp_ypathWP[1],gcp_xpathWP[1]) * 1000
print("The distance between waypoints is:",distanceWP,"m")
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
print("The drone average velocity is:",avg_velocity,"m/s")
#Declare circle splines
circleGCP = []
for i in range(len(gcp_xpathWP)):
    #create the circle spline for each gcp lat and long coordinates and store it in a list
    circleGCP.append(plt.Circle((gcp_xpathWP[i], gcp_ypathWP[i]), 0.00006, facecolor='none', edgecolor = 'r'))
########################################################################################################################
#The following code dynamically plot the flight's path. The mgcp can use a modified section of this code to realtime plot
#its path progress. IF deployed on gcp, have it plot everytime new data is received instead of accumulating data like this
#algo.
x=[]
y = []
fig, ax = plt.subplots() # note we must use plt.subplots, not plt.subplot
#initialize gcp lists
gcp_x = []
gcp_y = []
#initialize list counter for gcp path
c = 0
gcp_x.append(gcp_xpathWP[0])
gcp_y.append(gcp_ypathWP[0])
#Declare camera in track in meter
camera_intrack = 38.1
#plot the waypoints
ax.scatter(gcp_xpathWP,gcp_ypathWP)
#plot the LBO with respect to waypoints
for i in range(len(circleGCP)):
    ax.add_patch(circleGCP[i])
#initialize prev_D2 to the first drone position
prev_D2 = float(y_coor[0])
#find the middle y value of the drone path
smallest_drone_y, largest_drone_y=gcpfunctions.findSmallestandBiggest(y_coor)
middle_drone_y = abs(largest_drone_y - smallest_drone_y)/2 + smallest_drone_y
#plot the flight path as a simulation
for i in range(len(x_coor)):
  #keep adding values to lists per iteration
  x.append((float(x_coor[i])))
  y.append((float(y_coor[i])))
  #we only care about the drone's y path when we are trying to tell when the GCP should move with respect to the drone's progress
  D2_y = y[i]
  #Check if drone has passed the GCP
  #We will use gcp_xpathWP for the longitude values of the drone when calculating displacement from gcp because we do not care for drone's x path, we only care for y.
  #setting the drone's x path to equal the gcp's x path when comparing makes calculations easier.
  displacementMeters = gcpfunctions.getDistanceFromLatLonInKm(gcp_ypathWP[c],gcp_xpathWP[c],D2_y, gcp_xpathWP[c]) * 1000
  # Only set the next goal when the drone is as far as the maximum intrack, the drone is heading away from the gcp,
  # there are still waypoints left, the drone has x value has pass the waypoint x value
  if (displacementMeters > camera_intrack and (D2_y - prev_D2 > 0.0001) and (c < len(gcp_xpathWP)) and (x[i] > gcp_xpathWP[c])):
   #we append C+1 (the next x parameter to it because the drone has passed the initial point x point: x[i] > gcp_xpathWP.
   # We want to set the goal to the next point. We display this in the simulation by appending the next goal as c+1
    gcp_x.append(gcp_xpathWP[c+1])
    gcp_y.append(gcp_ypathWP[c+1])
    ax.plot(gcp_x,gcp_y,'b')
    #dont increment c if it's already at the last index because displacementMeter would throw an error outbound
    # index error due to c = c + 1 when already at max c
    if (c < len(gcp_xpathWP) - 1):
        #increment c to get the next goal (waypoint) before loop iterates
        c = c + 1
  # uncomment the line below to plot scatter points
  # ax.scatter(x_coor[i],y_coor[i])
  ax.plot(x, y, 'r')
  ax.axis('equal')
  # prevent blocking
  plt.show(block=False)
  # adjust pause speed if want the trace to be faster (it is in seconds)
  plt.pause(0.1)
  # store the drone previous position
  prev_D2 = D2_y
#end the script
sys.exit()