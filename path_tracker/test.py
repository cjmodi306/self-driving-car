import numpy as np
import matplotlib.pyplot as plt

v_c = np.array([[1, 0]])
t_c = np.array([[-1, -0.11]])

print(v_c[:,1]-t_c[:,1], v_c[:,0]-t_c[:,0])
print(np.arctan2(t_c[:,1], t_c[:,0])*180/np.pi)

#plt.plot(v_c[:,0],v_c[:,1], 'r-',marker=".",markersize=10)
plt.plot(t_c[:,0],t_c[:,1], 'b-',marker=".",markersize=10)
plt.xlim([-3,3])
plt.ylim([-3,3])
plt.grid()
plt.show()
