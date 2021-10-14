from math import sin, cos, sqrt, atan2, radians
from datetime import datetime
#declare twoD array with the following function list_to_2darr
#zip function combine corresponding x and y with respect to their indices and form tuples
#list function combine tuples into a 1d array with the structure [(x1,y1),(x2,y2)...(xn,yn)]
#np.array convert the list into a matrix by organizing the tuples in the 1d list into a 2d matrix by stacking them as columns
#np.array has the structure [x1 y1];[x2 y2];....;[xn yn]

def list_to_2darr(x_coor,y_coor):
  return np.array(list(zip(x_coor, y_coor)))

#find smallest and largest value in a list and return them as a tuple. All comparison needs to be
# in absolute because coordinate has direction indicator +/-
def findSmallestandBiggest(value):
  smallest_latff = value[0]
  largest_latff = value[0]
  for i in range(len(value)):
    if smallest_latff > value[i]:
      smallest_latff = value[i]
    elif largest_latff < value[i]:
      largest_latff = value[i]
  return smallest_latff,largest_latff

def findMaxdistance(value1, value2):
  return abs(value1 - value2)

#create the lateral path x-axis for gcp
def createGCPlongpath(spacing_size,maximum_long_dist,smallest_long,largest_long):
  # Estimate algo: we want a lot of data points between smallest lat and longest lat
  gcp_longpath = []
  increment_size = maximum_long_dist/spacing_size
  i = 0
  # Determine if direction is negative or positive/ determine the actual start point
  if(smallest_long < 0):
    #if the smallest determined latitude is smaller than 0 then the largest latitude is actually where the mgcp
    #should begin because it has the smaller magnitude.
    running_sum = largest_long
    increment_size = -1 * increment_size
  elif(smallest_long > 0):
    #if the smallest is bigger than 0 then we are dealing with positive direction. Smallest is indeed smallest
    running_sum = smallest_long
  while (i <= spacing_size):
    # initialize the latitude list with the first point in the loop by appending before incrementing running sum
    gcp_longpath.append(running_sum)
    running_sum = running_sum + increment_size
    i = i + 1
  return gcp_longpath

#create the gcp long path array to be plotted with gcp lat path array
def createGCPlatpath(optimized_latitude,spacing_size):
  gcp_latpath = []
  i = 0
  while (i <= spacing_size):
    gcp_latpath.append(optimized_latitude)
    i = i + 1
  return gcp_latpath

def calcDistTravelledft(displacement):
  return displacement * Decimal(60.0) * Decimal(1.15) * Decimal(5280.0)

def calcDistTravelledmile(displacement):
  return displacement * Decimal(60.0) * Decimal(1.15)

#calculate the average velocity of the drone based on analyzing the time stamps of the trajectory km/s
def calcVelocity(latValue,longValue, timeValue):
  distanceKM = []
  time_difference = []
  drone_velocity = []
  for i in range(len(latValue)):
    if i != (len(latValue) - 1):
      distanceKM.append(getDistanceFromLatLonInKm(latValue[i],longValue[i],latValue[i+1],longValue[i+1]))
      time_difference.append(getTimeDifference(timeValue[i],timeValue[i+1]))
      #if time_difference is 0 then we dont want to calculate velocity, else divide by 0 error
      if time_difference[i] != 0.0:
        drone_velocity.append(distanceKM[i]/time_difference[i])
  return drone_velocity


def getDistanceFromLatLonInKm(lat1, lon1, lat2, lon2):
    R = 6371  # Radius of the earth in km
    dLat = radians(lat2 - lat1)
    dLon = radians(lon2 - lon1)
    rLat1 = radians(lat1)
    rLat2 = radians(lat2)
    #haversine
    a = sin(dLat / 2) * sin(dLat / 2) + cos(rLat1) * cos(rLat2) * sin(dLon / 2) * sin(dLon / 2)
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    d = R * c  # Distance in km
    return d

def getTimeDifference(timeValue1, timeValue2):
  #declare time format
  fmt = '%Y:%m:%d %H:%M:%S'
  #strip 'em
  tstamp1 = datetime.strptime(timeValue1, fmt)
  tstamp2 = datetime.strptime(timeValue2, fmt)
  #absolute bars work here too but who cares
  if tstamp1 > tstamp2:
    td = tstamp1 - tstamp2
  else:
    td = tstamp2 - tstamp1
  return td.total_seconds()

#return gcp_xpath and gcp_ypath
def gcpPath(x_coor,chosen_y_coor, separation_size):
  # find the largest and smallest coordinates. The realtimeplotSHP.py told us that it is moving right to left.
  (smallestx, largestx) = findSmallestandBiggest(x_coor)
  # find the desired path for gcp
  maximum_x_dist = findMaxdistance(smallestx, largestx)
  gcp_xpath = createGCPlongpath(separation_size, maximum_x_dist, smallestx, largestx)
  gcp_ypath = createGCPlatpath(chosen_y_coor, separation_size)
  return gcp_xpath,gcp_ypath