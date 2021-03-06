import render as r
import controller3 as controller
import trajGen
import model.params as params
from model.quadcopter import Quadcopter
from stick_simulation.trajectoryReader import trajectory
import numpy as np
import time as thread_time

dt = params.dt
time = [0.0]
#wayPoints = [[0.0,0.0,0.0], [0.0,10.0,10.0], [0.0,10.0,15.0], [10.0,15.0,20.0], [25.0,25.0,25.0]]
#wayPoints = [[0.0,0.0,0.0], [0.0,5.0,10.0], [5.0,5.0,10.0], [10.0,5.0,15.0], [10.0,10.0,25.0],[10.0,10.0,40]]
#wayPoints = [[0.0,0.0,0.0], [1.0,1.0,3.0], [1.0,3.0,5.0], [3.0,3.0,5.0], [2.0,2.0,4.0],[4.0,4.0,6.0]]
#wayPoints = [[0,0,0],[1,1,3],[1,0,2],[1,0,3],[1,1,5]]; #totaltime 7
#wayPoints = [[0.0,0.0,0.0], [0.0,1.0,1.0], [0.0,1.0,1.5], [1.0,1.0,1.0], [2.0,2.0,2.0],
#			[3.0,3.0,3.0],[0.0,3.0,3.0],[2.0,1.0,1.0]] #Totaltime 7
wayPoints = [[0,0,0],[1,0.1,0],[0.2,0.2,0]]
#wayPoints = [[0.0,0.0,0.0], [0.0,1.0,1.0], [0.0,1.0,1.5], [1.0,1.0,1.0], [2.0,2.0,2.0],
#			[3.0,3.0,3.0],[0.0,3.0,3.0],[2.0,1.0,1.0],[2.0,1.0,1.5],[1.5,1.0,1.0],[1.3,1.0,1.5],[1.5,2.0,1.0],[2.0,2.0,2.0]] #Totaltime 10.0
totalTime = 2.0
'''
def attitudeControl(quad, time):
	
	#desired_state = trajGen.snap_Trajectory(wayPoints, totalTime, time[0])
	#desied_state = trajGen.waypoint()
	F, M = controller.run(quad, desired_state)
	quad.update(dt, F, M)
	time[0] += dt
'''
def main():
	pos = (0,0,0)
	attitude = (0,0,0)
	
	#generate trajectory
	#---------------------------------------------------------------		
	traj = trajectory("stick_simulation/data/trajectory4.txt")
	desiredTrajectory = traj.getTrajectory()
	waypoints = traj.getControlPoints()
	desiredTrajectory = trajGen.snap_Trajectory(waypoints, 32.0, dt)
	#---------------------------------------------------------------

	#desiredTrajectory = trajGen.snap_Trajectory(wayPoints, totalTime, dt)
	r.init(pos, attitude, dt, desiredTrajectory)
	r.draw_world()
	



if __name__ == "__main__":
    main()

