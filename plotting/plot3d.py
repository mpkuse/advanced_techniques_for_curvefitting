from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import numpy as np
import code

# cas = np.loadtxt( '../data3d/mannequin_mini.dat')
#
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.grid(True)
# ax.scatter( cas[:,0],cas[:,1],cas[:,2] , c='r', marker='*' )
# plt.show()

# np.savetxt( '../data3d/mannequin_micro.dat', cas[::10,:], fmt='%4.4lf' )
BASE_DIR = '../build/'

def animate(i):
    print i, len(T_cap)
    xmax = 200
    if i < len(T_cap):
        Xd_i = np.matmul( T_cap[i][0:3,0:3], w_Xd ) #+ T_cap[i][0:3,3]
        print Xd_i.shape
        ax.cla()
        ax.set_title( 'Iteration %d' %(i) )
        ax.scatter( w_X[0,:],w_X[1,:],w_X[2,:] , c='r', marker='o' )
        ax.set( xlim=(-xmax,xmax), ylim=(-xmax,xmax), zlim=(-xmax,xmax) )
        ax.scatter( Xd_i[0,:], Xd_i[1,:], Xd_i[2,:], c='g', marker='^' )
        ax.set( xlim=(-xmax,xmax), ylim=(-xmax,xmax), zlim=(-xmax,xmax) )



# Load point clouds
w_X =  np.loadtxt( BASE_DIR+'/w_X.txt', delimiter=',' ) #3xN
w_Xd = np.loadtxt( BASE_DIR+'/w_Xd.txt', delimiter=',' )


# Load all the intermediate poses
T_cap = []
for i in range(100):
    # read T_cap_i
    try:
        T = np.loadtxt( BASE_DIR+'/T_cap_%d.txt' %(i), delimiter=',' )
        print 'read %d' %i
        T_cap.append( T )
    except:
        print 'failed to read %d' %i
        break;


# Plot
fig = plt.figure()

ax = fig.add_subplot(111, projection='3d')
ax.grid(True)
ax.set_title( 'hello')
ax.scatter( w_X[0,:],w_X[1,:],w_X[2,:] , c='r', marker='o' )
ax.scatter( w_Xd[0,:],w_Xd[1,:],w_Xd[2,:] , c='g', marker='^' )


# Animate
aim = FuncAnimation( fig, animate, frames=np.arange(0,len(T_cap)) )

plt.draw()
plt.show()

# code.interact( local=locals() )
