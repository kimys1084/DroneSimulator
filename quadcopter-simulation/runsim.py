import render as r
import quadPlot as plt
import controller
import trajGen
import scheduler
from model.quadcopter import Quadcopter
import numpy as np
import time as thread_time

control_frequency = 200 # Hz for attitude control loop
dt = 1.0 / control_frequency
time = [0.0]
wayPoints = [[0,0,0], [0.0,20,20], [0,40,30], [20,30,40], [50,50,50]]
totalTime = 5.0
plt.drawpoints(wayPoints)


def render(quad):
	frame = quad.world_frame()
	#get quadrotor body pos in world frame
	plt.set_frame(frame)

def attitudeControl(quad, time):
	
	#desired_state = trajGen.genLine(time[0])
	desired_state = trajGen.snap_Trajectory(wayPoints, totalTime, time[0])
	F, M = controller.run(quad, desired_state)
	quad.update(dt, F, M)
	time[0] += dt
	plt.update_data(F, M, time[0])

def main():
	pos = (0,0,0)
	attitude = (0,0,np.pi/2)
	quadcopter = Quadcopter(pos, attitude)

	trajectory = trajGen.snap_Trajectory(wayPoints, totalTime, dt)

	r.init(trajectory)
	#sched = scheduler.Scheduler()
	#sched.add_task(attitudeControl, dt, (quadcopter,time))
	#kEvent_Render = sched.add_event(render, (quadcopter,))
	#plt.plot_quad_3d((sched, kEvent_Render))
	r.draw_world()
	



if __name__ == "__main__":
    main()

