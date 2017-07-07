import numpy as np
from collections import namedtuple
from math import sin, cos, asin, atan2, atan, acos, sqrt

class trajectory:
		
	def __init__(self, filename):
		self.points = None
		self.filename = filename
		self.trajectory_list = []
		self.timestep = 0.005
		self.initPos = None
		self.initAttitude = None
		self.frameNumber = 0

	def getTrajectory(self):
		self.points = np.array(self.readFromFile())
		self.points = np.reshape(self.points,(3,4,-1))
		self.frameNumber = self.points.shape[2] 
		# [frameNumber][control points index][x,y,z] 
		self.points = np.reshape(self.points, (self.frameNumber,4,3))

		#self.points, self.frameNumber = interpolation(self.points,self.frameNumber,10)



		for idx in range(self.frameNumber):
			pos = self.calTranslation(idx)
			vel = self.calVelocity(idx)
			acc = self.calAcceleration(idx)
			attitude = self.calRotation(idx)

			#print attitude
			omega = self.calOmega(idx)
			yaw = attitude[2]
			yawdot = omega[2]
			DesiredState = namedtuple('DesiredState', 'pos vel acc attitude omega yaw yawdot')
			self.trajectory_list.append(DesiredState(pos, vel, acc, attitude, omega, yaw, yawdot))

		return self.trajectory_list

	def readFromFile(self):
		points_temp = []
		f = open(self.filename, "r")
		while True:
			line = f.readline()
			if not line: break
			line = map(float,line.split())
			#convert centimeter to meter	
			points_temp.append(line[0] / 100.0)
			points_temp.append(line[1] / 100.0)
			points_temp.append(line[2] / 100.0)

		f.close()

		return points_temp

	# return center of gravity's position
	def calTranslation(self, idx):
		line1 = np.array(self.points[idx][1]) + np.array(self.points[idx][0])
		line2 = np.array(self.points[idx][3]) + np.array(self.points[idx][2])
		# get center of gravity position
		pos = (line1 + line2) /2.0
		if idx == 0:
			self.initPos = pos
		#set initial position zero	
		return pos - self.initPos

	def calVelocity(self, idx):
		if idx == 0:
			return np.zeros(3)
		else:
			vel = (self.calTranslation(idx) - self.calTranslation(idx-1)) / self.timestep
			return vel


	def calAcceleration(self, idx):
		if idx == 0:
			return np.zeros(3)
		else:
			acc = (self.calVelocity(idx) - self.calVelocity(idx-1)) / self.timestep
			return acc

	def calRotation(self, idx):
		cog = self.calTranslation(idx)
		line1 = np.array(self.points[idx][1]) - cog
		line2 = np.array(self.points[idx][3]) - cog


		norm_axis1 = normalize(line1)
		norm_axis2 = normalize(line2)
		norm_axis3 = normalize(np.cross(norm_axis1, norm_axis2))
		
		# because line1 & line2 are not othogonal
		norm_axis2 = normalize(np.cross(norm_axis3, norm_axis1))

		rotMatrix =make_R(norm_axis1, norm_axis2, norm_axis3)
		
		attitude = np.array(RotToRPY(rotMatrix))

		#set inital rotation zeros
		if idx == 0:
			self.initAttitude = np.array(RotToRPY(rotMatrix))

		#print attitude
		#print ""
		return attitude - self.initAttitude

	def calOmega(self, idx):
		if idx == 0:
			return np.zeros(3)
		else:
			omega = (self.calRotation(idx) - self.calRotation(idx-1)) / self.timestep
			return omega
	def printTraj(self):
		print self.points.shape

	def showTime(self):
		return self.frameNumber * self.timestep
	
	def getControlPoints(self):
		timeSlice = 40
		control_points_list = []
		control_points_number = self.frameNumber / timeSlice
		print control_points_number
		for idx in range(control_points_number):
			line1 = np.array(self.points[idx*timeSlice][1]) + np.array(self.points[idx*timeSlice][0])
			line2 = np.array(self.points[idx*timeSlice][3]) + np.array(self.points[idx*timeSlice][2])
			# get center of gravity position
			pos = (line1 + line2) /2.0
			if idx == 0:
				self.initPos = pos	
			control_points_list.append(pos - self.initPos)
		return control_points_list
	
	def getControlPoints2(self):
		distanceSlice = 0.04
		control_points_list = []

		for idx in range(self.frameNumber):
			line1 = np.array(self.points[idx][1]) + np.array(self.points[idx][0])
			line2 = np.array(self.points[idx][3]) + np.array(self.points[idx][2])
			# get center of gravity position
			pos = (line1 + line2) /2.0
			if idx == 0:
				self.initPos = pos
				control_points_list.append(pos-self.initPos)	
			elif distance(control_points_list[-1], pos-self.initPos) >= distanceSlice:
				control_points_list.append(pos-self.initPos)

		print "number of control points : ", len(control_points_list)
		return control_points_list


def normalize(v):
	norm = np.linalg.norm(v)
	if norm ==0:
		return v
	return v/norm


def RotToRPY(R): #original
    phi = asin(R[2,1])
    theta = atan2(-R[2,0]/cos(phi),R[2,2]/cos(phi))
    psi = atan2(-R[0,1]/cos(phi),R[1,1]/cos(phi))
    return phi*180/3.14, theta*180/3.14, psi*180/3.14



def make_R(a1,a2,a3):
	
	R = np.array([[a1[0], a2[0], a3[0]],
				  [a1[1], a2[1], a3[1]],
				  [a1[2], a2[2], a3[2]]])
	return R


#def interpolation(points, frameNumber, step):

def mag(x):
	return sqrt(sum(i**2 for i in x))

def distance(a,b):
	return sqrt((a[0]-b[0])**2 +(a[1]-b[1])**2 +(a[2]-b[2])**2)