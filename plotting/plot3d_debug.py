from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import numpy as np
import code


BASE_DIR = '../data3d/'
w_X = np.transpose( np.loadtxt( BASE_DIR + '/mateigen_86_51___w_X_iprev_triangulated.txt' ) )[0:3,:]
w_X2 = np.transpose( np.loadtxt( BASE_DIR + '/mateigen_86_51___w_X_icurr_triangulated.txt' ) )[0:3,:]


# plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.grid(True)
ax.set_title( 'hello')
ax.scatter( w_X[0,:],w_X[1,:],w_X[2,:] , c='r', marker='o' )
ax.scatter( w_X2[0,:],w_X2[1,:],w_X2[2,:] , c='b', marker='*' )

plt.draw()
plt.show()
