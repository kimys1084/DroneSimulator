import numpy as np
import scipy.integrate as integrate
from utils.quaternion import Quaternion 
from utils.utils import RPYToRot, RotToQuat, RotToRPY
import model.params as params

class Quadcopter:
	""" Quadcopter class
		state  - 1 dimensional vector but used as 13 x 1. [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
		where [qw, qx, qy, qz] is quternion and [p, q, r] are angular velocity [roll_dot, pitch_dot, yaw_dot]
	F      - 1 x 1, thrust output from controller
	M      - 3 x 1, moments output from controller
	params - system parameters struct, arm_length, g, mass, etc.
	"""
	def __init__(self, pos, attitude):
		""" pos = [x,y,z] attitude = [rool,pitch,yaw]
		"""
		self.state = np.zeros(13)
		self.R_nom = np.zeros(3)
		self.omega_nom = np.zeros(3)
		self.acc = np.zeros(3)
		self.acc[2] = 1
		# TODO
		# acc delete
		self.x_vector = np.array([[0,0,0]]).T
		self.y_vector = np.array([[0,0,0]]).T
		self.z_vector = np.array([[0,0,0]]).T
		roll, pitch, yaw = attitude
		rot    = RPYToRot(roll, pitch, yaw)
		quat   = RotToQuat(rot)
		self.state[0] = pos[0] 
		self.state[1] = pos[1]
		self.state[2] = pos[2]
		self.state[6] = quat[0]
		self.state[7] = quat[1]
		self.state[8] = quat[2]
		self.state[9] = quat[3]

	def world_frame(self):
		""" position returns a 3x6 matrix
			return drone body position in world frame  
            where row is [x, y, z] column is m1 m2 m3 m4 origin h
			m1 = motor1
		"""
		origin = self.state[0:3]
		quat = Quaternion(self.state[6:10])
		rot = quat.as_rotation_matrix()
		wHb = np.r_[np.c_[rot,origin], np.array([[0, 0, 0, 1]])]
		quadBodyFrame = params.body_frame.T
		quadWorldFrame = wHb.dot(quadBodyFrame)
		world_frame = quadWorldFrame[0:3]
		return world_frame

	def update_R_nom(self, R):
		R_nom = R

	def get_R(self):
		return self.R_nom

	def get_omega_nom(self):
		return self.omega_nom

	def update_omega_nom(self, omega):
		omega_nom = omega

	def position(self):
		return self.state[0:3]

	def velocity(self):
		return self.state[3:6]

	def attitude(self):
		rot = Quaternion(self.state[6:10]).as_rotation_matrix()
		return RotToRPY(rot)
    
	def omega(self):
		return self.state[10:13]

	# for controller 3 debugg not in use

	def store_xyz(self, x,y,z):
		self.x_vector = x
		self.y_vector = y
		self.z_vector = z

	def get_x(self):
		
		return [self.x_vector[0][0], self.x_vector[1][0], self.x_vector[2][0]]	
	def get_y(self):
		
		return [self.y_vector[0][0], self.y_vector[1][0], self.y_vector[2][0]]	
	def get_z(self):
		
		return [self.z_vector[0][0], self.z_vector[1][0], self.z_vector[2][0]]	

	#-----------------------------------

	def acceleration(self):
		return self.acc

	def calculate_acc(self, state, t, F):
		x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r = state
		quat = np.array([qw,qx,qy,qz])

		bRw = Quaternion(quat).as_rotation_matrix() # world to body rotation matrix
		wRb = bRw.T # orthogonal matrix inverse = transpose
        # acceleration - Newton's second law of motion
		accel = 1.0 / params.mass * (wRb.dot(np.array([[0, 0, F]]).T) - np.array([[0, 0, params.mass * params.g]]).T)
		
		return accel[0][0], accel[1][0], accel[2][0]

	def state_dot(self, state, t, F, M):
		x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r = state
		quat = np.array([qw,qx,qy,qz])

		bRw = Quaternion(quat).as_rotation_matrix() # world to body rotation matrix
		wRb = bRw.T # orthogonal matrix inverse = transpose
        # acceleration - Newton's second law of motion
		accel = 1.0 / params.mass * (wRb.dot(np.array([[0, 0, F]]).T) - np.array([[0, 0, params.mass * params.g]]).T)
        # angular velocity - using quternion
        # http://www.euclideanspace.com/physics/kinematics/angularvelocity/
		K_quat = 2.0; # this enforces the magnitude 1 constraint for the quaternion 
		quaterror = 1.0 - (qw**2 + qx**2 + qy**2 + qz**2)
		qdot = (-1.0/2) * np.array([[0, -p, -q, -r],
									[p,  0, -r,  q], 
									[q,  r,  0, -p],
									[r, -q,  p,  0]]).dot(quat) + K_quat * quaterror * quat;
        
        # angular acceleration - Euler's equation of motion
        # https://en.wikipedia.org/wiki/Euler%27s_equations_(rigid_body_dynamics)
		omega = np.array([p,q,r])
		pqrdot = params.invI.dot( M.flatten() - np.cross(omega, params.I.dot(omega)) )
		state_dot = np.zeros(13)
		state_dot[0]  = xdot
		state_dot[1]  = ydot
		state_dot[2]  = zdot
		state_dot[3]  = accel[0]
		state_dot[4]  = accel[1]
		state_dot[5]  = accel[2]
		state_dot[6]  = qdot[0]
		state_dot[7]  = qdot[1]
		state_dot[8]  = qdot[2]
		state_dot[9]  = qdot[3]
		state_dot[10] = pqrdot[0]
		state_dot[11] = pqrdot[1]
		state_dot[12] = pqrdot[2]

		return state_dot    

	def update(self, dt, F, M):
        # limit thrust and Moment
		L = params.arm_length
		r = params.r
		prop_thrusts = params.invA.dot(np.r_[np.array([[F]]), M])
		#set limit thrust
		prop_thrusts_clamped = np.maximum(np.minimum(prop_thrusts, params.maxF/4), params.minF/4)
		F = np.sum(prop_thrusts_clamped)
		M = params.A[1:].dot(prop_thrusts_clamped)
		self.state = integrate.odeint(self.state_dot, self.state, [0,dt], args = (F, M))[1]
		self.acc = self.calculate_acc(self.state,dt, F)
