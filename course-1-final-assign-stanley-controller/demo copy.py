import numpy as np
import csv
import matplotlib.pyplot as plt
class Controller2D(object):
    def __init__(self, waypoints):
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self.max_steering        = 1.22
        self.min_steering        = -1.22

        
    def show_path(self, x, y, color):
        for i in range(len(x)):
            plt.plot(x[i], y[i], color, marker=".", markersize=1)
    
    def get_min_distance_index(self, vehicle_coordinates, coordinates):
        crosstrack_distance = [np.linalg.norm(vehicle_coordinates-coordinates[i]) for i in range(coordinates.shape[0])]
        min_distance_index = crosstrack_distance.index(min(crosstrack_distance))
        return np.min(crosstrack_distance)*111139, min_distance_index
       
    def get_steering_angle(self, crosstrack_distance, yaw_track, yaw_vehicle, velocity, vehicle_coordinates, track_coordinates):
        # Eliminating yaw difference
        yaw_difference = yaw_track-yaw_vehicle
        steering_angle = yaw_difference
        
        # Eliminating crosstrack difference
        crosstrack_yaw = np.arctan2(vehicle_coordinates[:,1]-track_coordinates[1], vehicle_coordinates[:,0]-track_coordinates[0])
        yaw2ct = yaw_track - crosstrack_yaw
        if yaw2ct < -np.pi:
            yaw2ct += 2*np.pi
        elif yaw2ct > np.pi:
            yaw2ct -= 2*np.pi
        if yaw2ct < 0:
            crosstrack_distance = -crosstrack_distance
        else:
            crosstrack_distance = crosstrack_distance

        steering_angle += np.arctan2(crosstrack_distance,velocity)
        
        # Limiting the steering angle values
        if steering_angle > self.max_steering:
            steering_angle = self.max_steering
        elif steering_angle < self.min_steering:
            steering_angle = self.min_steering
        
        return steering_angle, yaw_difference
    
with open("course-1-final-assign-stanley-controller/waypoints.txt") as waypoints_file:
    waypoints = list(csv.reader(waypoints_file, delimiter=",", quoting=csv.QUOTE_NONNUMERIC))

waypoints_array = np.asarray(waypoints)
coordinates = waypoints_array[:,:2]
coordinates[:,1] = coordinates[:,1]


# vehicle data
velocity = 0.00003 # m/s
vehicle_coordinates = np.array([[49.2429011, 6.9728040]])
yaw_vehicle = -np.pi/2

# initializing and passing vehicle data to the controller
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

start=200
stop = 220
for i in range(start, stop):
    crosstrack_distance, min_distance_index = controller.get_min_distance_index(vehicle_coordinates, coordinates)
    print("Iteration: ",i, crosstrack_distance)
    yaw_track = np.arctan2(coordinates[min_distance_index+1, 1]-coordinates[min_distance_index, 1],
                            coordinates[min_distance_index+1, 0]- coordinates[min_distance_index, 0]) 
    
    new_steering_angle, yaw_difference = controller.get_steering_angle(crosstrack_distance, 
                                                                       yaw_track, 
                                                                       controller._current_yaw, 
                                                                       velocity,
                                                                       vehicle_coordinates,
                                                                       coordinates[min_distance_index])

    controller._set_steer = new_steering_angle

    new_current_x += velocity*np.cos(controller._set_steer+controller._current_yaw)
    new_current_y += velocity*np.sin(controller._set_steer+controller._current_yaw)

    # updating controller with the new vehicle coordinates
    controller._current_x = new_current_x
    controller._current_y = new_current_y
    controller._current_yaw = controller._current_yaw + controller._set_steer
    vehicle_coordinates = np.array([[controller._current_x, controller._current_y]])
    print(vehicle_coordinates)
    x.append(controller._current_x.item())
    y.append(controller._current_y.item())
    print("\n")

xy = np.concatenate((np.asarray(x).reshape(-1,1), np.asarray(y).reshape(-1,1)), axis=1)

plt.plot(coordinates[:,0], coordinates[:,1], 'b', marker='.', markersize=5, label="data_points")
#plt.plot(vehicle_coordinates[:,0], vehicle_coordinates[:,1], 'r', marker='.', markersize=5, label="data_points")
plt.plot(xy[:stop,0], xy[:stop,1], 'r', marker='.', markersize=5, label="vehicle_trajectory")
plt.plot(coordinates[min_distance_index,0], coordinates[min_distance_index,1], 'g', marker='.', markersize=5, label="data_points")
plt.legend()
plt.show()