import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

NUMTILES = 10
MAPSIZE =100
trajectory = []

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

def attitudeControl(quad, time, desired_state):
	
	F, M = controller.run(quad, desired_state)
	quad.update(dt, F, M)
	time +=dt

def draw_trajectory():
	
	glColor3f(0,255,0)
	glBegin(GL_LINES)
	for i in range(0, len(trajectory)):
		glVertex3fv(trajectory[i])

	glEnd()
	glColor3f(0,0,0)


def init(input_trajectory):

	for i in range(0, len(input_trajectory)):
		trajectory.append(input_trajectory[i].pos)

def draw_world():
	pygame.init()
	display = (800,600)
	
	pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
	gluPerspective(45, (display[0]/display[1]), 0.1, 500.0)

	glTranslatef(-20,-20,-100)

	glRotatef(10,3,1,1)
	while True:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
				quit()
			
			if event.type == pygame.KEYDOWN:
				
				if event.key == pygame.K_LEFT:
					glTranslatef(3, 0, 0)
				
				if event.key == pygame.K_RIGHT:
					glTranslatef(-3, 0, 0)

				if event.key == pygame.K_UP:
					glTranslatef(0, 3, 0)
				 
				if event.key == pygame.K_DOWN:
					glTranslatef(0, -3, 0)
			
			if event.type == pygame.MOUSEBUTTONDOWN:
				
				if event.button ==4:
					glTranslatef(0, 0, 3.0)
	
				if event.button == 5:
					glTranslatef(0, 0, -3.0)

					
		

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		#Cube()
		DrawMap()
		draw_trajectory()
		pygame.display.flip()
		pygame.time.wait(10)


if __name__ =="__main__":
	render()
