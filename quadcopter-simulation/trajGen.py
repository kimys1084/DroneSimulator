import numpy as np
from collections import namedtuple
from math import sin, cos
# This generate a straight line along z axis for hover control
def snap_Trajectory(wayPoints, end, dt):

	time =0.0
	T =0.0
	n = len(wayPoints) -1

	t_index =0

	interval = end / n
	traj_time = []
	timez = 0
	for i in range(0, n):
		traj_time.append([timez, timez + interval])
		timez += interval

	# case i = 0
	A = np.c_[np.zeros((1, 0)), 1, np.zeros((1,7)), np.zeros((1, 8*(n-1)))]
	b = [[wayPoints[0][0], wayPoints[0][1], wayPoints[0][2]]]
	
	A = np.r_[A, np.c_[np.zeros((1, 0)), np.ones((1,8)), np.zeros((1, 8*(n-1)))]]
	temp = [[wayPoints[1][0], wayPoints[1][1], wayPoints[1][2]]]
	b = np.r_[b, temp]

	for i in range(1, n):
		A = np.r_[A, np.c_[np.zeros((1, 8*i)), 1, np.zeros((1,7)), np.zeros((1, 8*(n-i-1)))]]
		
		temp = [[wayPoints[i][0], wayPoints[i][1], wayPoints[i][2]]]
		b = np.r_[b, temp]
	
		A = np.r_[A, np.c_[np.zeros((1, 8*i)), np.ones((1,8)), np.zeros((1, 8*(n-i-1)))]]

		temp = [[wayPoints[i+1][0], wayPoints[i+1][1], wayPoints[i+1][2]]]
		b = np.r_[b, temp]

	A = np.r_[A,np.c_[0,1,np.zeros((1,6)), np.zeros((1,(n-1)*8))]]
	A = np.r_[A,np.c_[0,0,1,np.zeros((1,5)), np.zeros((1,(n-1)*8))]]
	A = np.r_[A,np.c_[0,0,0,1,np.zeros((1,4)), np.zeros((1,(n-1)*8))]]
	
	b = np.r_[b,np.zeros((1,3))]
	b = np.r_[b,np.zeros((1,3))]
	b = np.r_[b,np.zeros((1,3))]

	A = np.r_[A,np.c_[np.zeros((1,(n-1)*8)),0,1,2,3,4,5,6,7]]
	A = np.r_[A,np.c_[np.zeros((1,(n-1)*8)),0,0,2,6,12,20,30,42]]
	A = np.r_[A,np.c_[np.zeros((1,(n-1)*8)),0,0,0,6,24,60,120,210]]

	b = np.r_[b,np.zeros((1,3))]
	b = np.r_[b,np.zeros((1,3))]
	b = np.r_[b,np.zeros((1,3))]


	for i in range(1,n):
		A = np.r_[A, np.c_[np.zeros((1,(i-1)*8)), 0, 1, 2, 3, 4, 5, 6, 7, 0, -1, np.zeros((1,6)), np.zeros((1,(n-i-1)*8))]]
		A = np.r_[A,np.c_[np.zeros((1,(i-1)*8)), 0, 0, 2, 6, 12, 20, 30, 42, 0, 0, -2, np.zeros((1,5)),np.zeros((1,(n-i-1)*8))]]
		A = np.r_[A,np.c_[np.zeros((1,(i-1)*8)), 0, 0, 0, 6, 24, 60, 120, 210, 0, 0, 0, -6, np.zeros((1,4)),np.zeros((1,(n-i-1)*8))]]
		A = np.r_[A,np.c_[np.zeros((1,(i-1)*8)), 0, 0, 0, 0, 24, 120, 360, 840, 0, 0, 0, 0, -24, np.zeros((1,3)),np.zeros((1,(n-i-1)*8))]]
		A = np.r_[A,np.c_[np.zeros((1,(i-1)*8)), 0, 0, 0, 0, 0, 120, 720, 2520, 0, 0, 0, 0, 0, -120,  np.zeros((1,2)),np.zeros((1,(n-i-1)*8))]]
		A = np.r_[A,np.c_[np.zeros((1,(i-1)*8)), 0, 0, 0, 0, 0, 0, 720, 5040, 0, 0, 0, 0, 0, 0, -720, np.zeros((1,1)),np.zeros((1,(n-i-1)*8))]]

		b = np.r_[b,np.zeros((1,3))]
		b = np.r_[b,np.zeros((1,3))]
		b = np.r_[b,np.zeros((1,3))]
		b = np.r_[b,np.zeros((1,3))]
		b = np.r_[b,np.zeros((1,3))]
		b = np.r_[b,np.zeros((1,3))]
		print i


	alpha = np.linalg.inv(A).dot(b)

	
	trajectory_list = []

	while time <= end:	
		if ( time == end):
			pos = wayPoints[-1]
			vel = np.zeros(3)
			acc = np.zeros(3)
		else:
			for i in range(0, len(traj_time)):
				if traj_time[i][0] <= time and time < traj_time[i][1]:
					t_index = i
					break
		
			if( time == 0.0):
				pos = wayPoints[0]
				vel = np.zeros(3)
				acc = np.zeros(3)
			else:
				if t_index >= 0:
					T  = time - traj_time[t_index][0]
			
	
			scale = T / interval
			coefficients = alpha[8*(t_index) : 8*(t_index+1), : ]
		
			pos = np.array([1, scale, scale**2, scale**3, scale**4, scale**5, scale**6, scale**7]).dot(coefficients)
			vel = np.array([0, 1, 2*scale, 3*scale**2, 4*scale**3, 5*scale**4, 6*scale**5, 7*scale**6]).dot(coefficients)
			acc = np.array([0, 0, 2, 6*scale, 12*scale**2, 20*scale**3, 30*scale**4, 42*scale**5]).dot(coefficients)

		yaw = 0#sin(time/2)
		yawdot = 0 # cos(time/2)
		time +=dt
		#pos = np.array([0,0,0])
		#vel = np.array([0,0,0])
		#acc = np.array([0,0,0])
		attitude = np.array([0,0,0])
		omega = np.array([0,0,0])
		
		DesiredState = namedtuple('DesiredState', 'pos vel acc attitude omega yaw yawdot')
		trajectory_list.append(DesiredState(pos, vel, acc, attitude, omega, yaw, yawdot))

	return trajectory_list


