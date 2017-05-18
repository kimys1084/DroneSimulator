from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from model.quadcopter import Quadcopter
#import attitude_controller as controller
import position_controller as controller
import numpy as np
NUMTILES = 10
MAPSIZE =100
width = 1600
height = 1200
play = True
index = 0
delay = 100
def DrawMap():

	glColor3f(255,255,255)
	glBegin(GL_LINES)
	interval = MAPSIZE / NUMTILES
	#draw xyz plain
	for i in range(0, NUMTILES+1):
		glVertex3f(0, i*interval, 0)
		glVertex3f(MAPSIZE, i*interval, 0)
	
		glVertex3f(0, i*interval, 0)
		glVertex3f(0, i*interval, MAPSIZE)

		glVertex3f(i*interval, 0, 0)
		glVertex3f(i*interval, MAPSIZE, 0)

		glVertex3f(i*interval, 0, 0)
		glVertex3f(i*interval, 0, MAPSIZE)

		glVertex3f(0, 0, i*interval)
		glVertex3f(MAPSIZE, 0, i*interval)
	
		glVertex3f(0, 0, i*interval)
		glVertex3f(0, MAPSIZE, i*interval)
	glEnd()

	glPointSize(10)
	glBegin(GL_POINTS)

	glColor3f(255,0,0)
	glVertex3f(0,0,0)
	glColor3f(255,255,255)

	glEnd()


def draw_trajectory():
	
	glColor3f(0,255,0)
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
	


#draw quadcopter
def draw_quadCopter(index):
	global play
	if play == True:	
		update_state(index)

	if index >= len(trajectory)-1:
		index = len(trajectory)-1
		
	cur_pos = trajectory[index].pos
	frame = quadCopter.world_frame()
	#z_des = quadCopter.get_z()
	#y_des = quadCopter.get_y()
	#x_des = quadCopter.get_x()
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
	rx, ry, rz = quadCopter.attitude()
	
	glPushMatrix()

	glTranslatef(x,y,z)
	glRotatef(ry*180.0/3.14, 0,1,0)
	glRotatef(rx*180.0/3.14, 1,0,0)
	glRotatef(rz*180.0/3.14, 0,0,1)

	
	glPushMatrix()
	
	glScalef(0.7, 0.1, 0.1)
	glColor3f(1,0,1)
	glutSolidCube(1.0)
	glColor3f(0,0,0)
	glutWireCube(1.0)
	
	glPopMatrix()
	
	glPushMatrix()
	glRotatef(90,0,0,1)
	glScalef(0.7, 0.1, 0.1)
	glColor3f(1,0,1)
	glutSolidCube(1.0)
	glColor3f(0,0,0)
	glutWireCube(1.0)
	glPopMatrix()


	glPopMatrix()

	
	
	glBegin(GL_LINES)
	'''
	glVertex3fv(body_data[0])
	glVertex3fv(body_data[2])

	glVertex3fv(body_data[1])
	glVertex3fv(body_data[3])
	'''
	
	#desired force vector

	glColor3f(0,255,0)
	glVertex3fv(center)
	glVertex3fv(center + acc)

	glEnd()	

def draw_world():


	global play
	global index
	initializeWindow()
	initializeSetting()
	printManual()
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
		printManual()
	elif ch ==  chr(110): # 'n' increase delay
		delay += 10
		print "Increase delay : ", delay
	elif ch == chr(98): # 'b' decrease delay
		if delay <= 10:
			print "delay : ", delay
		else:
			delay -= 10
			print "Decrease delay :", delay

	#------------------drone command-------
	elif ch == chr(116): # 't' yaw +
		quadCopter.set_index(1)	
		
	elif ch == chr(103): # 'g' yaw -
		quadCopter.set_index(2)	

	elif ch == chr(104): # 'h' roll +
		quadCopter.set_index(3)	

	elif ch == chr(102): # 'f' roll -
		quadCopter.set_index(4)	


	else:
		x,y = normalize_xy(x,y)
		camera.keyboard(ch, x,y)
def printManual():
	print "----------quadrotor simulator by YS KIM----------"
	print ""
	print "o : simulation stop"
	print "p : simulation play"
	print "---- camera control------------------------------"
	print "w : dolly in"
	print "s : dolly out"
	print "d : zoom in"
	print "a : zoom out"
	print "m : print manual"
	print "n : Increase delaay"
	print "b : Decrease delay"
	print "SHIFT + DRAG : translate camera"
	print "DRAG : rotate camera"

def mouse(button, state, x, y):
	x,y = normalize_xy(x,y)
	camera.mouse(button, state, x,y)

def motion(x,y):
	x, y = normalize_xy(x, y)
	camera.motion(x,y)
def display():

	global index
	camera.update()
	if camera.is_animating():
		glClearColor(0.5,0.5,0.5,0.0)
	else:
		glClearColor(0.0,0.0,0.0,0.0)
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

	glMatrixMode(GL_MODELVIEW)


	#------------------DRAW AND UPDATE-----------
	DrawMap()
	draw_trajectory()
	draw_quadCopter(index)

	if index < len(trajectory):
		if play == True:
			index+=1

	#--------------------------------------------
	glutSwapBuffers()

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
	glutTimerFunc(delay, on_timer, 0)

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
