from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from plot import plot_err
from model.quadcopter import Quadcopter
from Json import ListToJsonFile
from collections import namedtuple
#import attitude_controller as controller
import position_controller as controller
import printManual as manual
import numpy as np
import math

NUMTILES = 100
MAPSIZE =10
width = 1600
height = 1200
play = True
index = 0
delay = 0
light0 = True
lightPos = [-10.0, 10.0, 10.0, 0.0] 

lightKa = [0.5, 0.0, 0.0, 1.0]
lightKd = [0.5, 0.5, 0.0, 1.0]
lightKs = [0.5, 1.0, 1.0 , 1.0]
matKs = [1.0, 1.0, 1.0, 1.0]
matShininess = 50.0




def drawFloor():
	for col in range(-MAPSIZE, MAPSIZE, NUMTILES):
		for row in range(-MAPSIZE, MAPSIZE, NUMTILES):
			glBegin(GL_POLYGON)
			glNormal3d(0,0,1)
			glVertex3f(col, row, 0)
			glVertex3f(col+NUMTILES, row, 0)
			glVertex3f(col+NUMTILES, row+NUMTILES, 0)
			glVertex3f(col, row+NUMTILES, 0)
			glEnd()
def drawMap():

	glColor3f(0,0,0)
	glBegin(GL_LINES)
	interval = MAPSIZE / NUMTILES
	#draw xyz plain
	for i in range(0, NUMTILES+1):
		
		glVertex3f(0, float(i)*interval, 0)
		glVertex3f(MAPSIZE, float(i)*interval, 0)
	
		glVertex3f(0, float(i)*interval, 0)
		glVertex3f(0, float(i)*interval, MAPSIZE)

		glVertex3f(float(i)*interval, 0, 0)
		glVertex3f(float(i)*interval, MAPSIZE, 0)

		glVertex3f(float(i)*interval, 0, 0)
		glVertex3f(float(i)*interval, 0, MAPSIZE)

		glVertex3f(0, 0, float(i)*interval)
		glVertex3f(MAPSIZE, 0, float(i)*interval)
	
		glVertex3f(0, 0, float(i)*interval)
		glVertex3f(0, MAPSIZE, float(i)*interval)
	glEnd()

	glPointSize(10)
	glBegin(GL_POINTS)

	glColor3f(255,0,0)
	glVertex3f(0,0,0)
	glColor3f(255,255,255)

	glEnd()


def draw_trajectory():
	
	glColor3f(255,0,0)
	glBegin(GL_LINES)
	for i in range(0, len(trajectory)):
		glVertex3fv(trajectory[i].pos)

	glEnd()
	glColor3f(0,0,0)


def init(initPos, initAttitude, deltaTime, input_trajectory):


	global trajectory
	global quadCopter
	global ex_time
	global dt
	global actual
	actual = []
	ex_time =0
	dt = deltaTime
	trajectory =  input_trajectory
	quadCopter = Quadcopter(initPos, initAttitude)

#update quadcopter state
def update_state(index):

	if index < len(trajectory):
		desired_state = trajectory[index]
	else:
		#hovering
		index = len(trajectory)-1		
		desired_state =  trajectory[index]
		print "hovering.."
	F, M = controller.run(quadCopter, desired_state)
	quadCopter.update(dt,F,M)
	

def stateEvaluation(index):
	desired_state = trajectory[index]
	des_pos = desired_state.pos
	des_yaw = desired_state.yaw
	cur_pos =quadCopter.position()
	nouse1, nouse2, cur_yaw = quadCopter.attitude()

	#distance btw des_pos a& cur_pos
	error = math.sqrt((des_pos[0]- cur_pos[0])**2 + (des_pos[1]- cur_pos[1])**2 + (des_pos[2]- cur_pos[2])**2)
	threshold = 0.01
	#print error
	#print "vel : ", desired_state.vel
	#print "acc : ", desired_state.acc
	if error <= threshold:
		return True
	else: return False

#draw quadcopter
def draw_quadCopter(index):
	global play
	global actual
	if play == True:	
		update_state(index)

	if index >= len(trajectory)-1:
		index = len(trajectory)-1
		
	cur_pos = trajectory[index].pos
	frame = quadCopter.world_frame()

	body_data = []
	for col in range(0,4):
		body_data.append(frame[:,col])


	acc = quadCopter.acceleration()
	acc = normalize(acc)
	
	center = (body_data[0] + body_data[2]) / 2

	
	
	line1 = (body_data[1] - center) *2
	line2 = (body_data[2] - center) *2
	up_vector = normalize(np.cross(line1, line2))*2

	glBegin(GL_POINTS)
	glColor3f(255,125,0)
	glVertex3fv(cur_pos)
	glEnd()
	glColor3f(255,255,255)
	#------------draw quadcopter
	
	x,y,z = quadCopter.position()
	rotx,roty,rotz = quadCopter.attitude()
	pos = np.array([x,y,z])
	attitude = np.array([rotx,roty,rotz])
	
	if len(actual) <= len(trajectory):
		ActualState = namedtuple('ActualState', 'pos, attitude')
		actual.append(ActualState(pos, attitude))
	
	rx, ry, rz = quadCopter.attitude()
	
	glPushMatrix()

	glTranslatef(x,y,z)
	glRotatef(ry*180.0/3.14, 0,1,0)
	glRotatef(rx*180.0/3.14, 1,0,0)
	glRotatef(rz*180.0/3.14, 0,0,1)

	glPushMatrix()
	
	glScalef(0.45, 0.02, 0.02)
	glColor3f(0.5,0,0.5)
	glutSolidCube(1.0)
	glColor3f(0,0,0)
	glutWireCube(1.0)
	glPopMatrix()
	
	glPushMatrix()
	glRotatef(90,0,0,1)
	glScalef(0.45, 0.02, 0.02)
	glColor3f(1,0,1)
	glutSolidCube(1.0)
	glColor3f(0,0,0)
	glutWireCube(1.0)
	glPopMatrix()


	glPopMatrix()

	
	
	# glBegin(GL_LINES)

	# glColor3f(0,255,0)
	# glVertex3fv(center)
	# glVertex3fv(center + acc)

	# glEnd()	
	#position vector
	# glPushMatrix()
	# glBegin(GL_LINES)
	# glColor3f(255,0,255)
	# glVertex3f(x,y,z)
	# glVertex3f(quadCopter.get_x()[0] - x,quadCopter.get_x()[1] - y,quadCopter.get_x()[2] - z)
	
	# glVertex3f(x,y,z)
	# glVertex3f(quadCopter.get_y()[0] - x,quadCopter.get_y()[1] - y,quadCopter.get_y()[2] - z)
	
	# glVertex3f(x,y,z)
	# glVertex3f(quadCopter.get_z()[0] - x,quadCopter.get_z()[1] - y,quadCopter.get_z()[2] - z)
	# glEnd()
	# glPopMatrix()

def setLight():

	if light0 :
		glEnable(GL_LIGHT0)
		glLightfv(GL_LIGHT0, GL_AMBIENT, lightKa)
		glLightfv(GL_LIGHT0, GL_DIFFUSE, lightKd)
		glLightfv(GL_LIGHT0, GL_SPECULAR, lightKs)
		glLightfv(GL_LIGHT0, GL_POSITION, lightPos)
	#else glDisable(GL_LIGHT0)

def draw_world():

	initializeWindow()
	initializeSetting()
	manual.printManual()
	prepareCamera()
	glutMainLoop()
	

def reshape(new_width, new_height):
	global width, height
	width, height = new_width, new_height
	glViewport(0, 0, width, height)

def normalize_xy(x,y):
	x,y = x / float(width), (height - y) / float(height)
	return 2.0 * x - 1.0, 2.0 * y - 1.0


def keyboard(ch, x, y):
	global play
	global delay
	global quadCopter
	if ch == chr(27): #esc
		sys.exit(0)

	elif ch == chr(111): # 'o'
		print "simulation stop"
		play = False
	
	elif ch == chr(112): # 'p'
		print "simulation start"
		play = True
	elif ch == chr(109): # 'm' print manual
		manual.printManual()
	elif ch ==  chr(110): # 'n' increase delay
		if delay < 10:
			delay += 1
		else:
			delay += 10
		print "Increase delay : ", delay
	elif ch == chr(98): # 'b' decrease delay
		if delay <= 10 and delay > 1:
			delay -= 1
			print "Decrease delay : ", delay
		elif delay > 10:
			delay -= 10
			print "Decrease delay :", delay
		else:
			print "delay : ", delay

	#------------------drone command-------
	elif ch ==  'r':
		global trajectory
		global actual
		plot_err(trajectory, actual)
	elif ch == 't':
		print "make Json File"
		ListToJsonFile(actual, "droneOuput.txt")
		


	else:
		x,y = normalize_xy(x,y)
		camera.keyboard(ch, x,y)

def mouse(button, state, x, y):
	x,y = normalize_xy(x,y)
	camera.mouse(button, state, x,y)

def motion(x,y):
	x, y = normalize_xy(x, y)
	camera.motion(x,y)

#rendering part ------
def display():

	global index
	camera.update()
	if camera.is_animating():
		glClearColor(0.5,0.5,0.5,0.0)
	else:
		glClearColor(20.0,20,20,1.0)
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

	glMatrixMode(GL_MODELVIEW)
	#glEnable(GL_DEPTH_TEST)
	#glEnable(GL_LIGHTING)
	
	#setLight()


	glMaterialfv(GL_FRONT, GL_SPECULAR, matKs)
	glMaterialfv(GL_FRONT, GL_SHININESS, matShininess)


	#------------------DRAW AND UPDATE-----------
	glPushMatrix()
	glColor3f(50,50,50)
	drawFloor()
	glPopMatrix()
	drawMap()
	draw_trajectory()
	draw_quadCopter(index)

	if index < len(trajectory):
		if play == True:
			#if stateEvaluation(index) == True:
				#print index
			index+=1

	#--------------------------------------------
	#glDisable(GL_LIGHTING)
	glutSwapBuffers()
	glutPostRedisplay()
	# --> maybe have to put this : pygame.time.wait(10)

def initializeWindow():
	glutInit(['viewer'])
	glutInitWindowPosition(100,100)
	glutInitWindowSize(width, height)
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
	glutCreateWindow('Quadrotor simultar by YS')
	
	glutReshapeFunc(reshape)
	glutDisplayFunc(display)
	glutKeyboardFunc(keyboard)
	glutMouseFunc(mouse)
	glutMotionFunc(motion)
	#glutTimerFunc(delay, on_timer, 0)

def initializeSetting():
	glClearDepth(1.0)
	glClearColor(0.0, 0.0, 0.0, 0.0)
	
	glEnable(GL_DEPTH_TEST)
	glEnable(GL_LIGHTING)
	glEnable(GL_LIGHT0)
	glEnable(GL_NORMALIZE)
	glEnable(GL_COLOR_MATERIAL)

	glLightfv(GL_LIGHT0, GL_POSITION, (1.0, 1.0, 1.0, 1.0))

def prepareCamera():
	import camera

	global camera
	global delay
	camera = camera.Camera() # << MODEL???
	#camera.adjust_to_model()
	camera.see()
def on_timer(value):
	glutPostRedisplay()
	glutTimerFunc(delay, on_timer, 0)

def normalize(v):
	norm = np.linalg.norm(v)
	if norm == 0:
		return v
	return v / norm

if __name__ =="__main__":
	render()
