import numpy as np
import model.params as params
from math import sin,cos
from utils.utils import RPYToRot, RotToRPY
#input : roll, pitch, yaw, z
# return [F, M] F is total force thrust, M is 3x1 moment matrix

#constants

debug = False
#debug = True

v_p = 100
k_p = np.array([[v_p,0,0],
				[0, v_p,0],
				[0,0,v_p]])
v_v = 2
k_v = np.array([[v_v,0,0],
				[0,v_v,0],
				[0,0,v_v]])
v_r= 50
k_r = np.array([[v_r,0,0],
				[0, v_r,0],
				[0,0,v_r]])

v_w =2
k_w = np.array([[v_w,0,0],
				[0,v_w,0],
				[0,0,v_w]])


def run(quad, des_state):

	gravity = np.array([[0,0, params.mass*params.g]]).T


	#TODO
	#remove this area and calculate up vector with ohter method
	body_data = []
	frame = quad.world_frame()

	for col in range(0,4):
		body_data.append(frame[:,col])
	center = (body_data[0] + body_data[2])/2

	line1 = body_data[1] - center
	line2 = body_data[2] - center
	upVector = normalize(np.cross(line1, line2))
	z_b = np.array([upVector]).T
	#--------------------------------


	#---- get current state
	#-------get pos
	x,y,z = quad.position()
	pos = np.array([[0,0,z]]).T
	
	vx,vy,vz = quad.velocity()
	vel = np.array([[0,0,vz]]).T


	#-------get attitude
 	w_R_b = quad.rotation()
	p,q,r = quad.omega()
	w_b = np.array([[p,q,r]]).T
	

	#---- get des state
	#-------get pos
	x,y,z = des_state.pos
	des_pos = np.array([[0,0,z]]).T
	
	vx,vy,vz = des_state.vel
	des_vel = np.array([[0,0,vz]]).T
	
	ax,ay,az = des_state.acc
	des_acc = np.array([[0,0,az]]).T
	
	#-------get attitude
	des_roll,des_pitch,des_yaw = des_state.attitude
	w_des = np.array([des_state.omega]).T
	R_des = RPYToRot(des_roll, des_pitch, des_yaw)

	#calculate error
	e_p = pos - des_pos
	e_v = vel - des_vel
	
	e_R_matrix = (np.dot(R_des.T, w_R_b) - np.dot(w_R_b.T, R_des))
	e_R = 0.5* vee_map(e_R_matrix)
	
	e_w = w_b - w_des
	M = -np.dot(k_r,e_R) - np.dot(k_w,e_w)
	
	z_err = -np.dot(k_p, e_p) - np.dot(k_v, e_v) +gravity + params.mass*des_acc # -k_p*e_p -k_v*e_v +mgZw + ma_des

	F =np.dot(z_err.T,z_b)[0][0]
	#F = (params.mass * gravity)[2][0]
	#print F
	return F, M
	






#------------------------------------------------------------------------------------

def normalize(v):
	norm = np.linalg.norm(v)
	if norm == 0:
		return v
	return v / norm

def v_cross(a,b):
	t_a = np.array([a[0][0], a[1][0], a[2][0]])
	t_b = np.array([b[0][0], b[1][0], b[2][0]])
	
	t_v = np.cross(t_a, t_b)
	
	v = np.array([t_v]).T
	return v
	
def make_R(a1,a2,a3):
	
	R = np.array([[a1[0][0], a2[0][0], a3[0][0]],
				  [a1[1][0], a2[1][0], a3[1][0]],
				  [a1[2][0], a2[2][0], a3[2][0]]])
	return R

def vee_map(R):	
	return np.array([[-R[1][2], R[0][2], -R[0][1]]]).T	 

	
