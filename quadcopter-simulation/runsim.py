
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
wayPoints = [[0,0,0], [0.0,0.2,1], [0,0.4,2], [0,0.7,3], [1,0,5]]
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
	sched = scheduler.Scheduler()
	sched.add_task(attitudeControl, dt, (quadcopter,time))
	kEvent_Render = sched.add_event(render, (quadcopter,))
	plt.plot_quad_3d((sched, kEvent_Render))
	try:
		while True:
			thread_time.sleep(5)
	except KeyboardInterrupt:
		print ("attempting to close threads.")
		sched.stop()
		print ("terminated.")

if __name__ == "__main__":
    main()

