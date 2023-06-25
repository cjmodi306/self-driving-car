import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Bicycle:
    def __init__(self):
        self.long_pos = 0
        self.lateral_pos = 0
        self.steering_angle = 0
        
        self.heading_angle = 0
        self.slip_angle = 0
        self.time_stamp = 10e-3
        self.wheelbase = 2
        self.rear_distance = 1.2
        self.steering_angle_rate_max = 1.22
    
    def reset(self):
        self.long_pos = 0
        self.lateral_pos = 0
        self.steering_angle = 0
        self.heading_angle = 0
        self.slip_angle = 0

    def step(self, velocity, steering_angle_rate):
        
        if steering_angle_rate > 0:
            steering_angle_rate = min(steering_angle_rate, self.steering_angle_rate_max)

        else:
            steering_angle_rate = max(steering_angle_rate, -self.steering_angle_rate_max)

        self.long_velocity = velocity*np.cos(self.heading_angle + self.slip_angle)
        self.lateral_velocity = velocity*np.sin(self.heading_angle + self.slip_angle)
        self.angular_velocity = (velocity/self.wheelbase)*np.cos(self.slip_angle)*np.tan(self.steering_angle)

        self.steering_angle_rate = steering_angle_rate
        self.slip_angle = np.arctan(self.rear_distance * np.tan(self.steering_angle) / self.wheelbase)

        self.long_pos += self.long_velocity * self.time_stamp
        self.lateral_pos += self.lateral_velocity * self.time_stamp
        self.heading_angle += self.angular_velocity * self.time_stamp
        self.steering_angle += self.steering_angle_rate * self.time_stamp

model = Bicycle()

end_time = 30 # seconds
turning_radius = 8 # meters
turning_angle = np.arctan(model.wheelbase/turning_radius)
model.steering_angle = turning_angle


t_data = np.arange(0,end_time,model.time_stamp)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)

for i in range(x_data.shape[0]):
    x_data[i] = model.long_pos
    y_data[i] = model.lateral_pos
    velocity = 2*np.pi*turning_radius/end_time
    model.step(velocity, 0)
#plt.axis('equal')
plt.plot(x_data, y_data)
plt.show()