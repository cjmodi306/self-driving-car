import numpy as np
import csv
#import matplotlib.pyplot as plt
from lcd_display import i2c_lcd
from time import sleep
import utm
import gps

class Controller2D(object):
    def __init__(self, waypoints):
        self._current_x          = 0
        self._current_y          = 0
        self._previous_x         = 0
        self._previous_y         = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self.max_steering        = 1.22
        self.min_steering        = -1.22
        
    def update_coordinates(self):
        self._current_x, self._current_y = self.deg2cart(gps.get_coordinates())

    """
    def deg2cart(self, coordinates, interpolate=True):
        gps2cart = Transformer.from_crs(4979, 4978,always_xy=True)
        cartesian_coordinates = coordinates.copy()

        for i in range(coordinates.shape[0]):
            cartesian_coordinates[i] = gps2cart.transform(coordinates[i,0], coordinates[i,1], 0)[:2]
        
        self._waypoints = cartesian_coordinates

        return cartesian_coordinates*1000
    """
    def deg2cart(self, coordinates):
        cartesian_coordinates = np.array((coordinates)).copy()
        cartesian_coordinates= cartesian_coordinates.reshape(-1,2)
        if cartesian_coordinates.shape[0] > 1:
            for i in range(coordinates.shape[0]):
                cartesian_coordinates[i,0], cartesian_coordinates[i,1] = utm.from_latlon(coordinates[i,0], coordinates[i,1])[:2]
            self._waypoints = cartesian_coordinates
            return cartesian_coordinates*1000
        else:
            #print(utm.from_latlon(coordinates[0], coordinates[1])[:2])#*1000
            res = (utm.from_latlon(coordinates[0], coordinates[1])[:2])
            return res[0],res[1]
        
    def get_min_distance_index(self):
        vehicle_coordinates = np.array([[self._current_x, self._current_y]])
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
        print("Yaw_vehicle: ", self._current_yaw*180/np.pi)

        yaw_difference = yaw_track-self._current_yaw
        print("Yaw difference: ", yaw_difference)

        if yaw_difference > np.pi:
            yaw_difference -= 2*np.pi
        elif yaw_difference < -np.pi:
            yaw_difference += 2*np.pi
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
    
    def display_angle(self,text):
        mylcd = i2c_lcd.lcd()
        mylcd.lcd_display_string(text)
        sleep(1)
        mylcd.lcd_clear()
    
    def show_direction(self, heading_diff):
        angle = 90 + ((heading_diff*180)/np.pi)
        
