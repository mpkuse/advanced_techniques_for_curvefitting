from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import numpy as np
import code


BASE_DIR = '../data3d/'
BUILD_DIR = '../build/'

p_X = np.loadtxt( BUILD_DIR + '/p_X.txt', delimiter=',' )
c_X = np.loadtxt( BUILD_DIR + '/c_X.txt', delimiter=',' )
c_u = np.loadtxt( BUILD_DIR + '/c_u.txt', delimiter=',' )


# plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.grid(True)
ax.set_title( 'hello')
ax.scatter( p_X[0,:],p_X[1,:],p_X[2,:], c='r', marker='o' )
ax.scatter( c_X[0,:],c_X[1,:],c_X[2,:], c='b', marker='*' )
ax.scatter( c_u[0,:],c_u[1,:],c_u[2,:], c='g', marker='*' )

plt.draw()
plt.show()
