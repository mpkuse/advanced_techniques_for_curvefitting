# Plots the data points
# plots the line in each iteration

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def animate(i):
    fname =  '../build/itr%d.txt' %(i)
    ax.set_title( 'Iteration %d' %(i))
    # print 'load file: ', fname
    try:
        params = np.loadtxt( fname, delimiter=',' )
        print i, params
        curve.set_ydata( params[0]*x*x + params[1]*x + params[2] )
    except:
        print i, 'failed'
        return



fig, ax = plt.subplots( figsize=(20,20) )
ax.set( xlim=(-100,100), ylim=(-100,100) )
# ax.title( 0.5, 0.85, 'hello')
ax.set_title( 'hello')

M = np.loadtxt( '../build/M.txt', delimiter=',' )
print M.shape
ax.plot( M[:,0], M[:,1], 'r.')


x = np.linspace( -20, 20, 50 )
y = 4.4*x + 2
curve = ax.plot( x, y )[0]

anim = FuncAnimation( fig, animate, frames=np.arange(0,30), repeat=True, interval=500 )

plt.draw()
plt.show()

anim.save( 'file.gif', writer='imagemagick', fps=7.5 )
# anim.save( 'file.mp4', fps=7.5, extra_args=['-vcodec', 'libx264'])

# plt.show()
