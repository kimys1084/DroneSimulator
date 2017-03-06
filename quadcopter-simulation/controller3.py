import numpy as np
import model.params as params
from math import sin,cos

#Minimum snap trajectory generation and control for quadrotors

# return [F, M] F is total force thrust, M is 3x1 moment matrix

#constants

debug = True

v_p = 100
k_p = np.array([[v_p,0,0],
				[0, v_p,0],
				[0,0,v_p]])
v_v = 5
k_v = np.array([[v_v,0,0],
				[0,v_v,0],
				[0,0,v_v]])
v_r= 0.1
k_r = np.array([[v_r,0,0],
				[0, v_r,0],
				[0,0,v_r]])

v_w = 5
k_w = np.array([[v_w,0,0],
				[0,v_w,0],
				[0,0,v_w]])

def run(quad, des_state):
	x,y,z = quad.position()
	pos = np.array([[x,y,z]]).T
	vx,vy,vz = quad.velocity()
	vel = np.array([[vx,vy,vz]]).T

	phi, theta, psi = quad.attitude()
	p,q,r = quad.omega()
	w_b = np.array([[p,q,r]]).T

	x,y,z = des_state.pos
	des_pos = np.array([[x,y,z]]).T
	vx,vy,vz = des_state.vel
	des_vel = np.array([[vx,vy,vz]]).T

	ax,ay,az = des_state.acc
	des_acc = np.array([[ax,ay,az]]).T
	b_acc_x, b_acc_y, b_acc_z = quad.acceleration()
	x_c = np.array([[cos(psi), sin(psi), 0]]).T

	body_data = []
	frame = quad.world_frame()

	for col in range(0,4):
		body_data.append(frame[:,col])
	center = (body_data[0] + body_data[2])/2

	line1 = body_data[1] - center
	line2 = body_data[2] - center
	up_vector = normalize(np.cross(line1, line2))
	z_b = np.array([up_vector]).T
	'''
	y_b = v_cross(z_b, x_c)
	y_b = normalize(y_b)
	x_b = v_cross(y_b,z_b)
	'''
	x_b = normalize(-np.array([line2]).T)
	y_b = normalize(np.array([line1]).T)
	w_R_b = make_R(x_b, y_b, z_b)
	

	des_psi = des_state.yaw
	des_psi_dot = des_state.yawdot


	e_p = pos - des_pos
	e_v = vel - des_vel
	gravity = np.array([[0,0, params.mass*params.g]]).T
	
	F_des = -np.dot(k_p, e_p) - np.dot(k_v, e_v) +gravity + params.mass*des_acc # -k_p*e_p -k_v*e_v +mgZw + ma_des
	
	z_b_des = normalize(F_des)
	#TODO
	#delete this

	x_c_des = np.array([[cos(des_psi), sin(des_psi), 0]]).T
	
	y_b_des = v_cross(z_b_des, x_c_des)
	y_b_des = normalize(y_b_des)
	x_b_des = v_cross(z_b_des, y_b_des)

	R_des = make_R(x_b_des, y_b_des, z_b_des)

	quad.store_xyz(x_b_des, y_b_des, z_b_des)	
	#quad.store_xyz(x_b, y_b, z_b)	
	#print ""
	e_R_matrix = (np.dot(R_des.T, w_R_b) - np.dot(w_R_b.T, R_des))
	e_R = 0.5* vee_map(e_R_matrix)
	
	
	w_des = np.array([[0,0,0]]).T
	e_w = w_b - w_des
	
	M = -np.dot(k_r,e_R) #- np.dot(k_w,e_w)
	#M = np.array([[0,0,0]]).T
	F = np.dot(F_des.T, z_b)[0][0]

	if debug == True:
		print "R_des", R_des
		print ""
		print "w_R_b", w_R_b
		print ""
		print e_R_matrix
		#print "F : " ,F
		#print "M :", M
		print ""
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

	
