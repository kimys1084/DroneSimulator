from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import cnames
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import sys

# TODO add functionality for plotting state and desired_state

F=0
M=0
TIME=0


fig = plt.figure(num=0, figsize = (12,8))
fig.suptitle("QUADROTOR SIMULATION")
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Drone")
'''
ax01 = fig.add_subplot(222)
ax01.set_title('Force')
ax01.plot(TIME,F,'-', c='cyan')

ax02 = fig.add_subplot(223, projection='3d')
ax02.set_title('Moment')
ax02.plot([],[],[], '-', c='cyan')[0]
'''
#create new figure
#fig1 = plt.figure(num=1, figsize=(12,8))
#fig1.suptitle("test")

wayPoints= []

def drawpoints(dataPoints):
	wayPoints = dataPoints



def plot_quad_3d(args=()):

	ax.plot([], [], [], '-', c='cyan')[0]
	ax.plot([], [], [], '-', c='red')[0]
	ax.plot([], [], [], '-', c='blue', marker='o', markevery=2)[0]
	
	for i in range(0, len(wayPoints)):
		ax.plot([wayPoints[i][0]],[wayPoints[i][1]], [wayPoints[i][2]], '-', c= 'red', marker='o', markevery=2)
	

	set_limit((-4,4), (-4,4), (0,7))
	an = animation.FuncAnimation(fig, _callback, fargs = args, init_func=None,frames=400, interval=10, blit=False)

	#an1 = animation.FuncAnimation(fig1,_callback, fargs = args, init_func=None, frames=400, interval=10, blit=False)
#makes an animation by repeatedly calling a _callback function, passing in(optional) arguaments in farg
#args sched, kEvent_Render	
	plt.show()
	if len(sys.argv) > 1 and sys.argv[1] == 'save':
		an.save('sim.gif', dpi=80, writer='imagemagick', fps=60)
	else:
		plt.show()

def set_limit(x, y, z):
	ax.set_xlim(x)
	ax.set_ylim(y)
	ax.set_zlim(z)

def set_frame(frame):
    # convert 3x6 world_frame matrix into three line_data objects which is 3x2 (row:point index, column:x,y,z)
	lines_data = [frame[:,[0,2]], frame[:,[1,3]], frame[:,[4,5]]]
	lines = ax.get_lines()
	for line, line_data in zip(lines, lines_data):
		x, y, z = line_data
		line.set_data(x, y)
		line.set_3d_properties(z)
	#set_limit((x-0.5,x+0.5),(y-0.5, y+0.5),(z-0.5, z+5))

	#update_state()

def _callback(i, sched, id):
    # forward the event from GUI thread to scheduler threadA
    # do the actual rendering in _render method
    # start scheduler after we get the first frame so that we can see the initial state
	sched.start()
	sched.postEvent(id)

def update_data(force, moment, time):
	F = force
	M = moment
	TIME = time
	#print ("F : ", F,"TIME : ",TIME)

def update_state():
	line = ax01.get_lines()
	line.set_data(TIME,F)
	ax01.axes.set_xlim(TIME, TIME+100)
	
	#Mplot.set_data(TIME,M)
	#Mplot.axes.set_xlim(TIME, TIME+100)

