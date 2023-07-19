#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np
import matplotlib.pyplot as plt
import os, sys
import csv
import pandas as pd

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi
        self.max_steering        = 1.22
        self.min_steering        = -1.22

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('t_previous', 0.0)
        self.vars.create_var('error_previous', 0.0)
        self.vars.create_var('integral_error_previous', 0.0)
        self.vars.create_var('throttle_previous', 0.0)


        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.

            kp = 1.0
            ki = 0.2
            kd = 0.01

            throttle_output = 0
            brake_output    = 0

            # pid control
            st = t - self.vars.t_previous

            # error term
            e_v = v_desired - v

            # I
            inte_v = self.vars.integral_error_previous + e_v * st

            # D
            derivate = (e_v - self.vars.error_previous) / st

            acc = kp * e_v + ki * inte_v + kd * derivate

            if acc > 0:
                throttle_output = (np.tanh(acc) + 1)/2
                # throttle_output = max(0.0, min(1.0, throttle_output))
                if throttle_output - self.vars.throttle_previous > 0.1:
                    throttle_output = self.vars.throttle_previous + 0.1
            else:
                throttle_output = 0


            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change the steer output with the lateral controller. 
            steer_output = 0

            # Use stanley controller for lateral control
            k_e = 0.3
            slope = (waypoints[-1][1]-waypoints[0][1])/ (waypoints[-1][0]-waypoints[0][0])
            a = -slope
            b = 1.0
            c = (slope*waypoints[0][0]) - waypoints[0][1]

            # heading error
            yaw_path = np.arctan2(waypoints[-1][1]-waypoints[0][1], waypoints[-1][0]-waypoints[0][0])
            print(yaw_path)
            # yaw_path = np.arctan2(slope, 1.0)  # This was turning the vehicle only to the right (some error)
            yaw_diff_heading = yaw_path - yaw 
            if yaw_diff_heading > np.pi:
                yaw_diff_heading -= 2 * np.pi
            if yaw_diff_heading < - np.pi:
                yaw_diff_heading += 2 * np.pi

            # crosstrack erroe
            current_xy = np.array([x, y])
            crosstrack_error = np.min(np.sum((current_xy - np.array(waypoints)[:, :2])**2, axis=1))
            yaw_cross_track = np.arctan2(y-waypoints[0][1], x-waypoints[0][0])
            yaw_path2ct = yaw_path - yaw_cross_track
            if yaw_path2ct > np.pi:
                yaw_path2ct -= 2 * np.pi
            if yaw_path2ct < - np.pi:
                yaw_path2ct += 2 * np.pi
            if yaw_path2ct > 0:
                crosstrack_error = abs(crosstrack_error)
            else:
                crosstrack_error = - abs(crosstrack_error)
            yaw_diff_crosstrack = np.arctan(k_e * crosstrack_error / (v))

            # final expected steering
            steer_expect = yaw_diff_crosstrack + yaw_diff_heading
            if steer_expect > np.pi:
                steer_expect -= 2 * np.pi
            if steer_expect < - np.pi:
                steer_expect += 2 * np.pi
            steer_expect = min(1.22, steer_expect)
            steer_expect = max(-1.22, steer_expect)

            #update
            steer_output = steer_expect

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.throttle_previous = throttle_output
        self.vars.t_previous = t
        self.vars.error_previous = e_v
        self.vars.integral_error_previous = inte_v

    def show_path(self, x, y, color):
        for i in range(len(x)):
            plt.plot(x[i], y[i], color, marker=".", markersize=1)
    
    def get_min_distance_index(self, vehicle_coordinates, coordinates):
        crosstrack_distance = [np.sum((vehicle_coordinates-coordinates[i])**2) for i in range(coordinates.shape[0])]
        min_distance_index = crosstrack_distance.index(min(crosstrack_distance))
        return np.min(crosstrack_distance), min_distance_index
    
    def get_steering_angle(self, crosstrack_distance, yaw_track, yaw_vehicle, velocity, crosstrack_yaw):
        yaw_difference = yaw_track-yaw_vehicle
        steering_angle = yaw_difference
        sign = yaw_vehicle/abs(yaw_vehicle)
        #if yaw_vehicle > crosstrack_yaw:
        steering_angle = steering_angle + np.arctan(0.02*sign*crosstrack_distance/velocity)
        #else:
        #    steering_angle = yaw_difference #+ np.arctan(crosstrack_distance/velocity)
        print("yaw_diff: ", yaw_difference*180/np.pi)
        print("mid term:", (np.arctan2(crosstrack_distance,velocity))*180/np.pi)
        if steering_angle > self.max_steering:
            steering_angle = self.max_steering
        elif steering_angle < self.min_steering:
            steering_angle = self.min_steering
        print("steer: ", steering_angle)
        return steering_angle, yaw_difference

with open("course-1-final-assign-stanley-controller/racetrack_waypoints.txt") as waypoints_file:
    waypoints = list(csv.reader(waypoints_file, delimiter=",", quoting=csv.QUOTE_NONNUMERIC))

waypoints_array = np.asarray(waypoints)
coordinates = waypoints_array[:,:2]

velocity = 2 # m/s
previous_x, previous_y = 0, 0
vehicle_coordinates = np.array([[-150, 82]])
#yaw_vehicle = np.arctan2(vehicle_coordinates[:,1]-previous_y, vehicle_coordinates[:,0]-previous_x)
yaw_vehicle = -1.57
controller = Controller2D(waypoints)
controller._current_x = vehicle_coordinates[:,0]
controller._current_y = vehicle_coordinates[:,1]
controller._current_yaw = yaw_vehicle
new_current_x = vehicle_coordinates[:,0].item()
new_current_y = vehicle_coordinates[:,1].item()

x,y = [],[]
x.append(controller._current_x.item())
y.append(controller._current_y.item())
#print(controller._current_yaw*180/np.pi)

start=0
stop = 500

for i in range(start, stop):
    crosstrack_distance, min_distance_index = controller.get_min_distance_index(vehicle_coordinates, coordinates)
    print("Iteration: ",i)
    print("cs: ", crosstrack_distance)
    print("current yaw: ", controller._current_yaw*180/np.pi)
    
    yaw_track = np.arctan2(coordinates[min_distance_index+15, 1]-coordinates[min_distance_index, 1],
                           coordinates[min_distance_index+15, 0]- coordinates[min_distance_index, 0])
    
    print("yaw_track: ",yaw_track*180/np.pi)

    
    crosstrack_yaw = np.arctan2(coordinates[min_distance_index,1]-controller._current_y, coordinates[min_distance_index,0]-controller._current_x)
    
    new_steering_angle, yaw_difference = controller.get_steering_angle(crosstrack_distance, yaw_track, controller._current_yaw, velocity, crosstrack_yaw)

    previous_x = controller._current_x
    previous_y = controller._current_y

    controller._set_steer = new_steering_angle
    print("current steer: ", controller._set_steer*180/np.pi)
    print(controller._set_steer+controller._current_yaw)
    new_current_x += velocity*np.cos(controller._set_steer+controller._current_yaw)
    new_current_y += velocity*np.sin(controller._set_steer+controller._current_yaw)
    controller._current_x = new_current_x
    controller._current_y = new_current_y

    slope = np.arctan2(new_current_y-previous_y, new_current_x-previous_x)

    controller._current_yaw = controller._current_yaw + controller._set_steer
    vehicle_coordinates = np.array([[controller._current_x, controller._current_y]])
    x.append(controller._current_x.item())
    y.append(controller._current_y.item())
    print("\n")
  
    #print(yaw_track*180/np.pi, controller._current_yaw*180/np.pi)
xy = np.concatenate((np.asarray(x).reshape(-1,1), np.asarray(y).reshape(-1,1)), axis=1)
plt.plot(xy[:stop,0], xy[:stop,1], 'r', marker='.', markersize=10, label="vehicle_trajectory")
plt.plot(coordinates[:stop,0], coordinates[:stop,1], 'b', marker='.', markersize=10, label="data_points")
plt.legend()
plt.show()