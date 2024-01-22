
#import bpy
import sys
import site


# IMPORT PIP LIBS
import importlib
import math
import os
import random
import numpy as np
import cv2
import scipy
from src.helper import create_map

from src.RRT_star import Node, bidirectional_RRT
from src.generate_trajectory import Traj
import time

from djitellopy import Tello
import  csv







def main():
    #user input
    start=np.array([0.0,0.0,0.5]) #0,3,2 is inside an obstacle and doesn't work for map3!
    goal = np.array([-0.59, 6.81, 0.5])
    map_number=1

    map_string = "map"+str(map_number)+".txt"
    

    

    # Create map
    obstacle_blocks,boundary = create_map(map_string)
    print(obstacle_blocks)
    print(boundary)
    
    
    start_node = Node(start[0],start[1],start[2])
    goal_node = Node(goal[0],goal[1],goal[2])
    start_node.cost = 0.0
    rrt = bidirectional_RRT(start_node,goal_node,0.2,0.6,10000,600,boundary,obstacle_blocks)
    tree, final_path = rrt.run()
    
    for point in final_path:
        print(point)
    
    waypoints = np.asanyarray(final_path)

    traj=Traj(waypoints)

    tello=Tello()
    tello.connect()

    start_time = time.time()
    previous_time = start_time
    previous_pos = start
    tello.takeoff()




    csv_file_path = "pos_log.csv"

    with open(csv_file_path, mode='w', newline='') as csv_file:
    # Create a CSV writer object
        csv_writer = csv.writer(csv_file)
        log_data=[]
        try:
            while True:
                current_time = time.time()
                dt = current_time - previous_time
                state=tello.get_current_state()
                log_data.append([current_time-start_time,state['x'],state['y'],state['z'],state['pitch'],state['roll'],state['yaw']])
                if dt>=1:
                    pos,vel,acc,jerk,yaw=traj.get_des_state(current_time-start_time)
                    pos=np.array(pos)
                    del_pos = (pos-previous_pos)*100
                    speed = np.linalg.norm(del_pos)/dt
                    tello.go_xyz_speed(int(del_pos[0]),int(del_pos[1]),int(del_pos[2]),int(speed))
                    previous_time = current_time
                    previous_pos=np.array(pos)
                if np.linalg.norm(pos-goal)<0.1:
                    tello.land()
                    break
        except KeyboardInterrupt:
            print("exiting..")
            tello.land()
        csv_writer.writerows(log_data)



        


if __name__ == '__main__':
    main()
    

   