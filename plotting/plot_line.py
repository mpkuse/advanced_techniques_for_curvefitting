# Plots the data points
# plots the line in each iteration

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def animate(i):
    fname =  '../build/itr%d.txt' %(i)
    print 'load file: ', fname
    try:
        params = np.loadtxt( fname, delimiter=',' )
        print i, params
        line.set_ydata( params[0]*x + params[1] )
    except:
        print 'failed'
        return



fig, ax = plt.subplots( figsize=(5,3) )
ax.set( xlim=(-20,20), ylim=(-20,20) )


M = np.loadtxt( '../build/M.txt', delimiter=',' )
print M.shape
ax.plot( M[:,0], M[:,1], 'r.')


x = np.linspace( -20, 20, 5 )
y = 4.4*x + 2
line = ax.plot( x, y )[0]

anim = FuncAnimation( fig, animate )

plt.draw()
plt.show()

# plt.show()
