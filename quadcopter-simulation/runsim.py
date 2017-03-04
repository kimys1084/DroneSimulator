import render as r
import quadPlot as plt
import controller
import trajGen
import scheduler
import model.params as params
from model.quadcopter import Quadcopter
import numpy as np
import time as thread_time

dt = params.dt
time = [0.0]
#wayPoints = [[0.0,0.0,0.0], [0.0,10.0,10.0], [0.0,10.0,15.0], [10.0,15.0,20.0], [25.0,25.0,25.0]]
#wayPoints = [[0.0,0.0,0.0], [0.0,5.0,10.0], [5.0,5.0,10.0], [10.0,5.0,15.0], [10.0,10.0,25.0],[10.0,10.0,40]]
#wayPoints = [[0,0,0], [0,10,20.0]]
#wayPoints = [[0,0,0], [0,0,1.0]]
#wayPoints = [[0.0,0.0,0.0], [10.0,10.0,10.0], [10.0,10.0,5.0], [10.0,5.0,10.0], [10.0,10.0,7.0],[10.0,10.0,20]]
wayPoints = [[0,0,0],[0,0,5]]
totalTime = 5.0

def attitudeControl(quad, time):
	
	#desired_state = trajGen.genLine(time[0])
	#desired_state = trajGen.snap_Trajectory(wayPoints, totalTime, time[0])
	desied_state = trajGen.waypoint()
	F, M = controller.run(quad, desired_state)
	quad.update(dt, F, M)
	time[0] += dt

def main():
	pos = (0,0,0)
	attitude = (0,0,0)
	
	#generate trajectory
	trajectory = trajGen.snap_Trajectory(wayPoints, totalTime, dt)
	r.init(pos, attitude, dt, trajectory)
	#sched = scheduler.Scheduler()
	#sched.add_task(attitudeControl, dt, (quadcopter,time))
	#kEvent_Render = sched.add_event(render, (quadcopter,))
	#plt.plot_quad_3d((sched, kEvent_Render))
	r.draw_world()
	



if __name__ == "__main__":
    main()

