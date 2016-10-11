#!/usr/bin/env python

#import libraries
import testMotor as motor
import testOdometer as odo
import pygame
import time
import sys

pygame.init()

displayWidth = 190
displayHeight = 90
screen = pygame.display.set_mode((displayWidth,displayHeight))		# set display parameters of the window

# preset colours to use
black = (0,0,0)
white = (255,255,255)
red = (255,0,0)

font = pygame.font.SysFont('Avenir Next', 20)		# preset font to use

pygame.display.set_caption("Line Test")

def detect_close():
	for event in pygame.event.get():
		if event.type == pygame.QUIT:		# safe quit on closing the window
			pygame.quit()
			sys.exit()
		if event.type == pygame.KEYDOWN:		# look for key presses
			if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:		# safe quit on "q" press or "ESC" press
				pygame.quit()
				sys.exit()

def main():
	# initialise local variables
	fwd = 127
	turn = 127
	distance = 2173
	near = distance * 0.1
	reached = False
	readings = [0,0]
	initReadings = [0,0]
	prevReadings = [0,0]
	prevRollovers = [0,0]
	isFirst = 0
	
	ser = motor.set_serial()
	ser.write(chr(fwd))
	ser.write(chr(turn))
	
	while not reached:
		detect_close()
		constants = odo.init_pins()		# initialise the pins and create the constants dictionary
					
		data = odo.read_raw(constants)		# read from odometers
		
		if isFirst != 0:
			readings[0] = odo.bit_slicer(data,constants['READING_LOW_0_BIT'],constants['READING_BIT_LENGTH'])
			readings[1] = odo.bit_slicer(data,constants['READING_LOW_1_BIT'],constants['READING_BIT_LENGTH'])
			
			readings[0] = readings[0] - initReadings[0]
			readings[1] = readings[1] - initReadings[1]
			
		else:
			isFirst = -1
			initReadings[0] = odo.bit_slicer(data,constants['READING_LOW_0_BIT'],constants['READING_BIT_LENGTH'])
			initReadings[1] = odo.bit_slicer(data,constants['READING_LOW_1_BIT'],constants['READING_BIT_LENGTH'])
					
		readings = odo.handle_rollovers(readings,constants, prevReadings, prevRollovers)
		
		difference = (distance - readings[0], distance - readings[1])
		
		if (readings[0] < distance) or (readings[1] < distance):
			if (difference < near):
				fwd = fwd + 10
				ser.write(chr(fwd))
			elif (fwd < 217) and (fwd > 37):
				fwd = fwd - 10		# minus for now until the fwd is reversed
				ser.write(chr(fwd))
		else:
			fwd = 127
			ser.write(chr(fwd))
			reached = True
					
		screen.fill(black)		# set the screen background to black (should be anyway as default)
        
		# display text to the screen, blit can also render images and draw on the screen
		screen.blit(font.render("Telemetry: (forward, turn)",True,white),(15,5))
		screen.blit(font.render(str(motor.get_telemetry(ser)),True,white),(110,25))
		screen.blit(font.render("Odometers: (left, right)",True,white),(15,45))
		screen.blit(font.render(str((int(readings[0]), int(-readings[1]))),True,white),(95,65))
        
		pygame.display.update()		# update the display on each loop

		time.sleep(0.016)		# sleep for 0.016 seconds
		
if __name__ == "__main__":
	main()
	pygame.quit()
	quit()
