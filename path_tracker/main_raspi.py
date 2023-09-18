from raspi_utils import Controller2D
import numpy as np
import csv
from time import *
import gps
import sys
#import matplotlib.pyplot as plt
        
    
with open("local_coordinates.txt") as waypoints_file:
    waypoints = list(csv.reader(waypoints_file, delimiter=",", quoting=csv.QUOTE_NONNUMERIC))

waypoints_array = np.asarray(waypoints)
coordinates = waypoints_array[:,:2]

controller = Controller2D(coordinates)
coordinates = controller.deg2cart(coordinates)

# vehicle data
velocity = 7 # m/s
yaw_vehicle = 0

# initializing and passing vehicle data to the controller
controller._current_x, controller._current_y = controller.deg2cart(gps.get_coordinates())
controller._current_yaw = yaw_vehicle
controller._current_speed = velocity
x,y = [],[]
x.append(controller._current_x.item())
y.append(controller._current_y.item())
#print(controller._current_yaw*180/np.pi)

start=200
stop = 238

for i in range(start, stop):
    #print("Iteration: ",i)
    crosstrack_distance, min_distance_index = controller.get_min_distance_index()
    #print("Crosstrack: ", crosstrack_distance)
    yaw_track = controller.get_yaw_track(min_distance_index)
    #print("Yaw_track: ", yaw_track*180/np.pi)
    #print("Yaw_vehicle: ", controller._current_yaw*180/np.pi)

    new_steering_angle, yaw_difference = controller.get_steering_angle(crosstrack_distance, 
                                                                       yaw_track,
                                                                       min_distance_index)
    #print("Yaw difference in angle: ", yaw_difference*180/np.pi)

    controller._set_steer = new_steering_angle
    controller._previous_x = controller._current_x
    controller._previous_y = controller._current_y
    controller.update_coordinates()
    controller._current_yaw = np.arctan2(controller._current_y-controller._previous_y,
                                         controller._current_x-controller._previous_x)
    
    #new_current_x += velocity*np.cos(controller._set_steer+controller._current_yaw)
    #new_current_y += velocity*np.sin(controller._set_steer+controller._current_yaw)

    # updating controller with the new vehicle coordinates
    #controller._current_x = new_current_x
    #controller._current_y = new_current_y
    #controller._current_yaw = controller._current_yaw + controller._set_steer
    #vehicle_coordinates = np.array([[controller._current_x, controller._current_y]])
    #x.append(controller._current_x.item())
    #y.append(controller._current_y.item())
    
    controller.display_angle(str(yaw_difference*180/np.pi))
    print("\n")

print(yaw_difference*180/np.pi)
controller.show_direction(yaw_difference)
xy = np.concatenate((np.asarray(x).reshape(-1,1), np.asarray(y).reshape(-1,1)), axis=1)

#plt.plot(coordinates[:,0], coordinates[:,1], 'b', marker='.', markersize=5, label="data_points")
##plt.plot(vehicle_coordinates[:,0], vehicle_coordinates[:,1], 'r', marker='.', markersize=5, label="data_points")
#plt.plot(xy[:stop,0], xy[:stop,1], 'r', marker='.', markersize=5, label="vehicle_trajectory")
##plt.plot(coordinates[min_distance_index,0], coordinates[min_distance_index,1], 'g', marker='.', markersize=5, label="data_points")
#plt.legend()
#plt.show()
