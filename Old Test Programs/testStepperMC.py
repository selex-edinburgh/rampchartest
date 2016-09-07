#!/usr/bin/env python

# import libraries
import smbus
import time
import serial
import pygame

bus = smbus.SMBus(1)        # set up the smbus interface

pygame.init()
# preset colours to use
black = (0,0,0)
white = (255,255,255)
red = (255,0,0)
# set up display parameters
displayWidth = 190
displayHeight = 25
screen = pygame.display.set_mode((displayWidth,displayHeight))
font = pygame.font.SysFont('Avenir Next', 20)		# preset font to use
pygame.display.set_caption("Stepper Test")

def init_stepper(busData, controlCodes):
	print "Initialising Stepper"
	try:
		bus.write_block_data(busData['address'],controlCodes['initMotor'],busData['txBytes'])
	except Exception as err:
		print err
		print "Failed to write to the bus"
	
def rotate_stepper(busData, controlCodes):
	print "Attempting rotation"
	try:
		bus.write_block_data(busData['address'],controlCodes['betweenAngles'],[20,-20])
	except Exception as err:
		print err
	
def read_angle(busData, controlCodes):
	rxBytes = bus.read_i2c_block_data(busData['address'], controlCodes['readAngle'],busData['numBytes'])
	irAngle = rxBytes[0]*256 + rxBytes[1]
	irAngle = irAngle/11.32 -135
	return irAngle

def detect_close():
	for event in pygame.event.get():
		if event.type == pygame.QUIT:		# safe quit on closing the window
			pygame.quit()
			sys.exit()
		if event.type == pygame.KEYDOWN:		# look for key presses
			if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:		# safe quit on "q" press or "ESC" press
				pygame.quit()
				sys.exit()

def screen_display(irAngle):		
	screen.fill(black)		# set the screen background to black (should be anyway as default)   
	screen.blit(font.render("Stepper Angle: ",True,white),(15,5))
	screen.blit(font.render(str(irAngle),True,white),(95,5))
	pygame.display.update()		# update the display on each loop
	
def main():
	busData = {
	'address' : 4,
	'txBytes' : [0,0],
	'rxBytes' : 0, 
	'numBytes': 4
	}
	
	controlCodes = {
	'stop' : 32,            #32
	'initMotor' : 33,       #37
	'fixedAngle' : 39,      #43 max speed
	'betweenAngles' : 42,   #46 med speed
	'readAngle' : 116       #112
	}
	irAngle = 0
	
	init_stepper(busData,controlCodes)
	time.sleep(5)
	rotate_stepper(busData,controlCodes)
	time.sleep(1)

	while True:
		irAngle = read_angle(busData, controlCodes)
		screen_display(irAngle)
		time.sleep(1)
	
main()
