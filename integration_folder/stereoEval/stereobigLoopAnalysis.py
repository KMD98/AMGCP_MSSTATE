import numpy as np
import matplotlib.pyplot as plt
import sys
import math

def poseplot():
    with open('bigloop.txt', 'r') as fhandle:
       lines = fhandle.readlines()
    absolute_path = np.array([[0,0],[7.5,-3],[12.67,0],[24.0,-1.5],[43,-2],[42.3,-6.8],[13,-4.5],[7.5,-3],[-0.4,-3],[0,0]])
    stereo_pose = np.zeros((len(lines),2))
    i = 0
    for pose in lines:
        x,y,z = pose.split(" ")
        stereo_pose[i][0] = x
        stereo_pose[i][1] = y
        i+=1
    total_absdistance = 0
    for i in range(1,len(absolute_path[:,0])):
        total_absdistance = total_absdistance + math.sqrt(math.pow(absolute_path[i][0] - absolute_path[i-1][0],2) + math.pow(absolute_path[i][1] - absolute_path[i][1],2))
    print("%s m" %total_absdistance)
    total_stereodist =0
    for i in range(1,len(stereo_pose[:,0])):
        total_stereodist = total_stereodist + math.sqrt(math.pow(stereo_pose[i][0] - stereo_pose[i-1][0],2) + math.pow(stereo_pose[i][1] - stereo_pose[i][1],2))
    print("%s m" %total_stereodist)
    fig1,ax2 = plt.subplots()
    ax2.set_ylabel("y(m)")
    ax2.set_xlabel("x(m)")
    ax2.set_title("Large Loop: Absolute Path and Visual-Inertial Odometry")
    for i in range (0,10):
        ax2.add_patch(plt.Circle((absolute_path[i][0],absolute_path[i][1]),0.4, facecolor='none',edgecolor = 'r'))
    ax2.plot(stereo_pose[:1595,0],stereo_pose[:1595,1], label='visual-inertial odom')
    ax2.legend(loc = 'upper right')
    ax2.plot(absolute_path[:,0],absolute_path[:,1], label='absolute path') 
    ax2.legend(loc = 'upper right')
    plt.grid()
    ax2.axis('equal')
    txt="Average error in total distance traveled: %s m"%(abs(total_absdistance - total_stereodist))
    plt.figtext(0.5, 0.01, txt, wrap=True, horizontalalignment='center', fontsize=8)           
    plt.show()

if __name__ == "__main__":
    poseplot()
