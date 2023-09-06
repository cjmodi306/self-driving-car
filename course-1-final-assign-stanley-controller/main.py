import numpy as np
import csv
import matplotlib.pyplot as plt
import cv2
from pyproj import Transformer
import sys

class Controller2D(object):
    def __init__(self, waypoints):
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = self.deg2cart(waypoints)/100
        self.max_steering        = 1.22
        self.min_steering        = -1.22

    def deg2cart(self, coordinates, interpolate=True):
        gps2cart = Transformer.from_crs(4979, 4978,always_xy=True)
        cartesian_coordinates = coordinates.copy()

        for i in range(coordinates.shape[0]):
            cartesian_coordinates[i] = gps2cart.transform(coordinates[i,0], coordinates[i,1], 0)[:2]
        
        self._waypoints = cartesian_coordinates

        return cartesian_coordinates*1000
    
    def get_min_distance_index(self):
        vehicle_coordinates = np.array([self._current_x, self._current_y])
        crosstrack_distance = [np.linalg.norm(vehicle_coordinates-self._waypoints[i]) for i in range(self._waypoints.shape[0])]
        min_distance_index = crosstrack_distance.index(min(crosstrack_distance))
        return np.min(crosstrack_distance), min_distance_index
    
    def get_yaw_track(self, min_distance_index):
        yaw_track = np.arctan2(self._waypoints[min_distance_index+1, 1]-self._waypoints[min_distance_index, 1],
                            self._waypoints[min_distance_index+1, 0]- self._waypoints[min_distance_index, 0]) 
        return yaw_track

    def get_steering_angle(self, crosstrack_distance, yaw_track, min_distance_index):
        # Eliminating yaw difference
        print("Yaw_track: ", yaw_track*180/np.pi)
        print("Yaw_vehicle: ", controller._current_yaw*180/np.pi)

        yaw_difference = yaw_track-self._current_yaw
        print("Yaw difference in radians: ", yaw_difference)

        if yaw_difference > np.pi:
            yaw_difference -= 2*np.pi
        elif yaw_difference < -np.pi:
            yaw_difference += 2*np.pi
        
        print("Corrected Yaw difference in degrees: ", yaw_difference*180/np.pi)
        steering_angle = yaw_difference
        # Eliminating crosstrack difference
        crosstrack_yaw = np.arctan2(self._current_y-self._waypoints[min_distance_index][1], 
                                    self._current_x-self._waypoints[min_distance_index][0])
        yaw2ct = yaw_track - crosstrack_yaw
        if yaw2ct < -np.pi:
            yaw2ct += 2*np.pi
        elif yaw2ct > np.pi:
            yaw2ct -= 2*np.pi
        if yaw2ct < 0:
            crosstrack_distance = -crosstrack_distance
        else:
            crosstrack_distance = crosstrack_distance

        steering_angle += np.arctan2(crosstrack_distance,self._current_speed)
        
        # Limiting the steering angle values
        if steering_angle > self.max_steering:
            steering_angle = self.max_steering
        elif steering_angle < self.min_steering:
            steering_angle = self.min_steering
        
        return steering_angle, yaw_difference

    def show_vector(self, heading_diff):
        vector_angle = (np.pi/2)+heading_diff
        base_vector = np.array([[1, 0]])
        x_new = base_vector[0,0]*np.cos(vector_angle) - base_vector[0,1]*np.sin(vector_angle)
        y_new = base_vector[0,0]*np.sin(vector_angle) + base_vector[0,1]*np.cos(vector_angle)
        transformed_vector = np.array([[x_new, y_new]])
        print(transformed_vector)
        tail = [0, 0]
        fig, ax = plt.subplots(1)
        ax.quiver(*tail,
                    transformed_vector[:, 0],
                    transformed_vector[:, 1],
                    scale=1,
                    scale_units='xy',
                    angles = 'xy',
                    color=['r'])
        ax.set_xlim((-1, transformed_vector[:,0].max()+1))
        ax.set_ylim((-1, transformed_vector[:,1].max()+1))
        plt.show()
    
    def show_direction(self, heading_diff):
        image = cv2.imread("arrow.jpg")
        image_center = tuple(np.array(image.shape[1::-1]) / 2)
        angle = 90 + ((heading_diff*180)/np.pi)
        rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
        result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
        cv2.imshow("direction", result)
        cv2.waitKey(100)
    
with open("local_coordinates.txt") as waypoints_file:
    waypoints = list(csv.reader(waypoints_file, delimiter=",", quoting=csv.QUOTE_NONNUMERIC))

waypoints_array = np.asarray(waypoints)
coordinates = waypoints_array[:,:2]

controller = Controller2D(coordinates)
coordinates = controller._waypoints
# vehicle data
velocity = 30 # m/s
vehicle_coordinates = np.array([[controller._waypoints[0,0], controller._waypoints[0,1]]])
yaw_vehicle = -np.pi
print(vehicle_coordinates)

# initializing and passing vehicle data to the controller
controller._current_x = vehicle_coordinates[:,0].item()
controller._current_y = vehicle_coordinates[:,1].item()
print(controller._current_x, controller._current_y)
controller._current_yaw = yaw_vehicle
controller._current_speed = velocity
new_current_x = vehicle_coordinates[:,0].item()
new_current_y = vehicle_coordinates[:,1].item()


x,y = [],[]
x.append(controller._current_x)
y.append(controller._current_y)
#print(controller._current_yaw*180/np.pi)

start=0
stop = 538

for i in range(start, stop):
    print("Iteration: ",i)
    velocity = 35
    crosstrack_distance, min_distance_index = controller.get_min_distance_index()
    if crosstrack_distance>10:
        velocity *= 0.5
    print("Crosstrack: ", crosstrack_distance)
    yaw_track = controller.get_yaw_track(min_distance_index)
    #print("Yaw_track: ", yaw_track*180/np.pi)
    #print("Yaw_vehicle: ", controller._current_yaw*180/np.pi)

    new_steering_angle, yaw_difference = controller.get_steering_angle(crosstrack_distance, 
                                                                       yaw_track,
                                                                       min_distance_index)
    print("Yaw difference: ", yaw_difference*180/np.pi)

    controller._set_steer = new_steering_angle
    print(new_steering_angle*180/np.pi)
    print(new_current_x, new_current_y)

    new_current_x += velocity*np.cos(controller._set_steer+controller._current_yaw)
    new_current_y += velocity*np.sin(controller._set_steer+controller._current_yaw)

    print(new_current_x, new_current_y)
    print(coordinates[i,0], coordinates[i,1])

    # updating controller with the new vehicle coordinates
    controller._current_x = new_current_x
    controller._current_y = new_current_y
    controller._current_yaw = controller._current_yaw + controller._set_steer
    vehicle_coordinates = np.array([[controller._current_x, controller._current_y]])
    x.append(controller._current_x)
    y.append(controller._current_y)
    print("\n")
    #sys.exit()

xy = np.concatenate((np.asarray(x).reshape(-1,1), np.asarray(y).reshape(-1,1)), axis=1)


plt.plot(coordinates[:,0], coordinates[:,1], 'b', marker='.', markersize=5, label="data_points")
plt.plot(xy[:stop,0], xy[:stop,1], 'r', marker='.', markersize=5, label="vehicle_trajectory")
plt.legend()
#plt.xlim([41415, 41425])
#plt.ylim([5090, 5060])
plt.show()