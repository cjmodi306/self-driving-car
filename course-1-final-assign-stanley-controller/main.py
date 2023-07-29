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

    def deg2cart(self, coordinates, interpolate=True):
        first_row = coordinates[0,:].copy()

        for i in range(coordinates.shape[0]):
            coordinates[i] = (coordinates[i].copy()-first_row)*111111
        
        if interpolate:
            x_new = np.linspace(coordinates[0,0], coordinates[-1,0],1000)
            if x_new[1] < x_new[0]:
                x_new = -x_new
                coordinates = -coordinates
            y_new = np.interp(x_new, coordinates[:,0], coordinates[:,1])
            #print(y_new)
            coordinates = np.hstack([-x_new.reshape(-1,1), -y_new.reshape(-1,1)])
        
        self._waypoints = coordinates

        return coordinates
    
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
        yaw_difference = yaw_track-self._current_yaw
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
    
with open("course-1-final-assign-stanley-controller/waypoints.txt") as waypoints_file:
    waypoints = list(csv.reader(waypoints_file, delimiter=",", quoting=csv.QUOTE_NONNUMERIC))

waypoints_array = np.asarray(waypoints)
coordinates = waypoints_array[:,:2]

controller = Controller2D(coordinates)
coordinates = controller.deg2cart(coordinates)
#print(coordinates)
# vehicle data
velocity = 10 # m/s
vehicle_coordinates = np.array([[-13, -12]])
yaw_vehicle = np.pi

# initializing and passing vehicle data to the controller
controller._current_x = vehicle_coordinates[:,0]
controller._current_y = vehicle_coordinates[:,1]
controller._current_yaw = yaw_vehicle
controller._current_speed = velocity
new_current_x = vehicle_coordinates[:,0].item()
new_current_y = vehicle_coordinates[:,1].item()

x,y = [],[]
x.append(controller._current_x.item())
y.append(controller._current_y.item())
#print(controller._current_yaw*180/np.pi)

start=200
stop = 205

for i in range(start, stop):
    crosstrack_distance, min_distance_index = controller.get_min_distance_index()
    yaw_track = controller.get_yaw_track(min_distance_index)
    new_steering_angle, yaw_difference = controller.get_steering_angle(crosstrack_distance, 
                                                                       yaw_track,
                                                                       min_distance_index)

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
#plt.plot(coordinates[min_distance_index,0], coordinates[min_distance_index,1], 'g', marker='.', markersize=5, label="data_points")
plt.legend()
plt.show()