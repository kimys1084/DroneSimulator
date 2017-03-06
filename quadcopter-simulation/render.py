import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

from model.quadcopter import Quadcopter
import controller3 as controller
import numpy as np
NUMTILES = 10
MAPSIZE =100


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

	if play == True:	
		update_state(index)

	if index >= len(trajectory)-1:
		index = len(trajectory)-1
		
	cur_pos = trajectory[index].pos
	frame = quadCopter.world_frame()
	z_des = quadCopter.get_z()
	y_des = quadCopter.get_y()
	x_des = quadCopter.get_x()
	body_data = []
	for col in range(0,4):
		body_data.append(frame[:,col])

	
	center = (body_data[0] + body_data[2]) / 2
	
	line1 = (body_data[1] - center) *2
	line2 = (body_data[2] - center) *2
	up_vector = normalize(np.cross(line1, line2))*2

	glBegin(GL_POINTS)
	glColor3f(255,125,0)
	glVertex3fv(cur_pos)
	glEnd()
	glColor3f(255,255,255)

	glBegin(GL_LINES)
	glVertex3fv(body_data[0])
	glVertex3fv(body_data[2])

	glVertex3fv(body_data[1])
	glVertex3fv(body_data[3])

	
	
	#for debug
	#draw body coordinate
	glColor3f(255,0,0)

	glVertex3fv(center)
	glVertex3fv(line1+center)

	glColor3f(0,255,0)
	glVertex3fv(center)
	glVertex3fv(line2+center)

	glColor3f(0,0,255)
	glVertex3fv(center)
	glVertex3fv(up_vector+center)
	
	#desired force vector
	glColor3f(50,255,100)	
	glVertex3fv(center)
	glVertex3fv(center + z_des)

	glColor3f(0,100,255)	
	glVertex3fv(center)
	glVertex3fv(center + y_des)
	
	glVertex3fv(center)
	glVertex3fv(center + x_des)
	
	
	glColor3f(255,255,255)
	glEnd()	

def draw_world():
	pygame.init()
	display = (1600,1200)
	global play
	play = True
	pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
	gluPerspective(45, (display[0]/display[1]), 0.1, 500.0)

	glTranslatef(-10,-10,-50)
	glRotatef(-90,1,0,0)
	glRotatef(-90,0,0,1)	
	#trajectory index
	index = 0

	while True:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
				quit()
			
			if event.type == pygame.KEYDOWN:
				
				if event.key == pygame.K_LEFT:
					glTranslatef(0, 1, 0)
				
				if event.key == pygame.K_RIGHT:
					glTranslatef(0, -1, 0)

				if event.key == pygame.K_UP:
					glTranslatef(0, 0, -1)
				 
				if event.key == pygame.K_DOWN:
					glTranslatef(0, 0, 1)
				#pause
				if event.key == pygame.K_s:
					print "simulation stop"
					play = False
				if event.key == pygame.K_d:
					print "simulation play"
					play = True
					
			
			if event.type == pygame.MOUSEBUTTONDOWN:
				
				if event.button ==4:
					glTranslatef(3, 0, 0.0)
	
				if event.button == 5:
					glTranslatef(-3, 0, -0.0)

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		DrawMap()
		draw_quadCopter(index)
		draw_trajectory()
		pygame.display.flip()
		pygame.time.wait(10)
		if index < len(trajectory):
			if play == True:
				index+=1

def normalize(v):
	norm = np.linalg.norm(v)
	if norm == 0:
		return v
	return v / norm

if __name__ =="__main__":
	render()
