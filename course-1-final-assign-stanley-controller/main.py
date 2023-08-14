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
        print("Yaw_track: ", yaw_track*180/np.pi)
        print("Yaw_vehicle: ", controller._current_yaw*180/np.pi)

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
    
with open("course-1-final-assign-stanley-controller/ruckweg_coordinates.txt") as waypoints_file:
    waypoints = list(csv.reader(waypoints_file, delimiter=",", quoting=csv.QUOTE_NONNUMERIC))

waypoints_array = np.asarray(waypoints)
coordinates = waypoints_array[:,:2]

controller = Controller2D(coordinates)
coordinates = controller.deg2cart(coordinates)
#print(coordinates)
# vehicle data
velocity = 7 # m/s
vehicle_coordinates = np.array([[-39, -72]])
yaw_vehicle = -np.pi

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
stop = 450

for i in range(start, stop):
    print("Iteration: ",i)
    crosstrack_distance, min_distance_index = controller.get_min_distance_index()
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

    new_current_x += velocity*np.cos(controller._set_steer+controller._current_yaw)
    new_current_y += velocity*np.sin(controller._set_steer+controller._current_yaw)

    # updating controller with the new vehicle coordinates
    controller._current_x = new_current_x
    controller._current_y = new_current_y
    controller._current_yaw = controller._current_yaw + controller._set_steer
    vehicle_coordinates = np.array([[controller._current_x, controller._current_y]])
    x.append(controller._current_x.item())
    y.append(controller._current_y.item())
    print("\n")

print(yaw_difference*180/np.pi)
controller.show_vector(yaw_difference)
xy = np.concatenate((np.asarray(x).reshape(-1,1), np.asarray(y).reshape(-1,1)), axis=1)

plt.plot(coordinates[:,0], coordinates[:,1], 'b', marker='.', markersize=5, label="data_points")
#plt.plot(vehicle_coordinates[:,0], vehicle_coordinates[:,1], 'r', marker='.', markersize=5, label="data_points")
plt.plot(xy[:stop,0], xy[:stop,1], 'r', marker='.', markersize=5, label="vehicle_trajectory")
#plt.plot(coordinates[min_distance_index,0], coordinates[min_distance_index,1], 'g', marker='.', markersize=5, label="data_points")
plt.legend()
plt.show()