import pickle
import numpy as np
import matplotlib.pyplot as plt
import sys

DATA_PATH = 'data/data.pickle'

def get_data(DATA_PATH):
    with open(DATA_PATH, 'rb') as f:
        data = pickle.load(f)
        return data
    
data = get_data(DATA_PATH=DATA_PATH)
timestamp = data['t']  # timestamps [s]
x_init  = data['x_init'] # initial x position [m]
y_init  = data['y_init'] # initial y position [m]
heading_init = data['th_init'] # initial theta position [rad]
# input signal
velocity  = data['v']  # translational velocity input [m/s]
rot_velocity = data['om']  # rotational velocity input [rad/s]
# bearing and range measurements, LIDAR constants
bearing = data['b']  # bearing to each landmarks center in the frame attached to the laser [rad]
range_measurement = data['r']  # range measurements [m]
landmarks = data['l']  # x,y positions of landmarks [m]
d = data['d']  # distance between robot center and laser rangefinder [m]
# noise variance
v_var = 1  # translation velocity variance  
om_var = 5  # rotational velocity variance 
r_var = 0.01  # range measurements variance
b_var = 10  # bearing measurement variance
# Covariance matrix
Q_km = np.diag([v_var, om_var]) # input noise covariance 
cov_y = np.diag([r_var, b_var])  # measurement noise covariance 

x = np.ones((len(timestamp), 3))
x[0] = x_init, y_init, heading_init

# Motion model
for i in range(1, len(timestamp)):
    delta_t = timestamp[i] - timestamp[i-1]
    theta_prev = x[i-1][2]
    x_prev = np.array([x[i-1]]).T
 
    inputs = np.array([[velocity[i], rot_velocity[i]]]).T
    F = np.array([[np.cos(theta_prev), 0], [np.sin(theta_prev), 0], [0, 1]])

    x_predicted = x_prev + delta_t* (F.dot(inputs))
    
    dF = np.array([[1,0,-delta_t*velocity[i]*np.sin(theta_prev)], 
                    [0,1,delta_t*velocity[i]*np.cos(theta_prev)],
                    [0,0,delta_t]], dtype='float')
    print(x_predicted.shape)
    sys.exit()
    #P_predicted = F P F.T + L Q L.T
