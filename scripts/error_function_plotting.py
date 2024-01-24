from matplotlib import pyplot as plt
import numpy as np


def function(e_x, e_y):
    # omega = np.array([[50, 0],
    #              [0, 50]])


    return 200*e_x**2 + 400*e_y**2

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

e_x_min, e_x_max = -100, 100
e_y_min, e_y_max = -100, 100

num_points = 1000
x = np.linspace(e_x_min, e_x_max, num_points)
y = np.linspace(e_y_min, e_y_max, num_points)
X, Y = np.meshgrid(x, y)
z = function(X, Y)


ax.plot_surface(X, Y, z, cmap='viridis', alpha=.5)
ax.set_xlabel('E_x')
ax.set_ylabel('E_y')
ax.set_zlabel('function')
plt.show()
