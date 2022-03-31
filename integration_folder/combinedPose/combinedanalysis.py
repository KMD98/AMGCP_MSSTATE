import numpy as np
import matplotlib.pyplot as plt
import sys
import math
import utm
def poseplot():
    with open('combineOdom.txt', 'r') as fhandle:
       lines = fhandle.readlines()
    extra_goals = np.array([[320390.462,3705920.877],[320390.0223,3705917.8799],[320387.630,3705916.175],[320385.5566,3705917.3331]])
    for i in range(0,len(extra_goals[:,0])):   
        convert = utm.to_latlon(extra_goals[i][0],extra_goals[i][1],16,'S')
        extra_goals[i][0] = convert[1]
        extra_goals[i][1] = convert[0]
    goals = np.array([[-88.9330228,33.4777622],extra_goals[0],extra_goals[1],[-88.9330322,33.4776764],extra_goals[2],[-88.9330773,33.4776858],extra_goals[3],[-88.9330667,33.4777549],[-88.9330228,33.4777622]])
    absolute_path = np.zeros((len(goals[:,0]),len(goals[0,:])))
    for i in range(0,len(goals[:,0])):
        utm_temp = utm.from_latlon(goals[i][1],goals[i][0])
        absolute_path[i][0] = utm_temp[0]
        absolute_path[i][1] = utm_temp[1]
    combined_pose = np.zeros((len(lines),2))
    i = 0
    for pose in lines:
        lat,lon= pose.split(" ")
        temp = utm.from_latlon(float(lat),float(lon))
        combined_pose[i][0] = temp[0]
        combined_pose[i][1] = temp[1]
        i+=1
    total_absdistance = 0
    for i in range(1,len(absolute_path[:,0])):
        total_absdistance = total_absdistance + math.sqrt(math.pow(absolute_path[i][0] - absolute_path[i-1][0],2) + math.pow(absolute_path[i][1] - absolute_path[i][1],2))
    print("Total absolute traveled distance is: %s m" %total_absdistance)
    total_combineddist =0
    for i in range(1,len(combined_pose[:,0])):
        total_combineddist = total_combineddist + math.sqrt(math.pow(combined_pose[i][0] - combined_pose[i-1][0],2) + math.pow(combined_pose[i][1] - combined_pose[i][1],2))
    print("Total tracked distance is: %s m" %total_combineddist)
    #Convert all odom to local frame
    starting_point = [absolute_path[0,0], absolute_path[0,1]]
    for i in range(len(absolute_path[:,0])):
        absolute_path[i][0] = absolute_path[i][0] - starting_point[0]
        absolute_path[i][1] = absolute_path[i][1] - starting_point[1]
    for i in range(len(combined_pose[:,0])):
        combined_pose[i][0]-= starting_point[0]
        combined_pose[i][1]-= starting_point[1]
    #get loop enclosure error
    loop_error = math.sqrt(pow(absolute_path[0][0] - combined_pose[-1,0],2) + math.pow(absolute_path[0][1]-combined_pose[-1,1],2))
    #plot
    fig1,ax2 = plt.subplots()
    ax2.set_ylabel("y(m)")
    ax2.set_xlabel("x(m)")
    ax2.set_title("Combined GPS and Visual Inertial Positional Tracking")
    for i in range (0,len(absolute_path[:,1])):
        if i == 0:
            ax2.add_patch(plt.Circle((absolute_path[i][0],absolute_path[i][1]),0.2, facecolor='none',edgecolor = 'g'))
    ax2.plot(combined_pose[:,0],combined_pose[:,1], label='combined odom')
    ax2.legend(loc = 'upper right')
    ax2.plot(absolute_path[:,0],absolute_path[:,1], label='absolute path') 
    ax2.legend(loc = 'upper right')
    plt.grid()
    ax2.axis('equal')
    txt="Error in total distance traveled and loop enclosure: " + str(round((abs(total_absdistance - total_combineddist)),2)) + 'm ' + str(round(loop_error,2)) +'m.'
    plt.figtext(0.5, 0.01, txt, wrap=True, horizontalalignment='center', fontsize=8)           
    plt.show()

if __name__ == "__main__":
    poseplot()
