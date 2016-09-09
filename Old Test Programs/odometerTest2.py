#desktop/testPrograms/odometerTest2.py   1/9/16
#This program tests the odometers on each wheel independently by hand movement
#Rollover and Continuous Modes are selected by keyboard c

#!/usr/bin/env python

# import libraries
import time
import sys
import pygame
#import operator

try:		# try and import the GPIO library and catch a RunTimeError on fail
	import RPi.GPIO as GPIO
except RunTimeError as err:	# catch the RunTimeError and output a substitute response
	print err
	print ("Error: Can't import RPi.GPIO")

'''
   This function will read from the odometers directly and pass the
   result back to the main(). 
   The Odometers use a simple synchronised data link where the R-Pi pulses a clock pin high    and low, and this tells the odometers to release individual bits of data which we assemble into separate 16 bit words. Each 360 degree rotation of the wheel is divided up into 1024 steps. Every rotation the data counts from 0 to 1024 and then starts again. We call this a rollover and identify it by a step change in data from 0 to 1024 or visa versa.
   The GPIO Clock pins on both odometers are connected together so that the 16 data bits are clocked out simultaneously. The GPIO Chip Select Pins are also connected together so that both odometers are triggered to start together and data readings are sent simultaneously.
   The Individual Data pin values are read separately from different GPIO pins.
'''

def read_raw(constants):
	# initialise local variables
	a = 0
	readBit = [0,0]
	data = [0,0]
	'''
		Here there is pin manipulation to bring the chip select pin high
		then low before reading from the data pins.
	'''
	GPIO.output(constants['CHIP_SELECT_PIN'],GPIO.HIGH)
	time.sleep(constants['TICK'])
	GPIO.output(constants['CLOCK_PIN'],GPIO.HIGH)
	time.sleep(constants['TICK'])
	GPIO.output(constants['CHIP_SELECT_PIN'],GPIO.LOW)
	time.sleep(constants['TICK'])

	while a < (constants['READING_BIT_LENGTH'] + constants['READING_LOW_0_BIT']):		# loop 16 times to get a 16 bit data read from the odometers
		GPIO.output(constants['CLOCK_PIN'], GPIO.LOW)		# as the data pins are read the clock pulses high and low
		time.sleep(constants['TICK'])
		GPIO.output(constants['CLOCK_PIN'],GPIO.HIGH)
		time.sleep(constants['TICK'])

		readBit[0]= GPIO.input(constants['DATA_PIN_0']) #read a bit from Rt odom pin_0
		readBit[1]= GPIO.input(constants['DATA_PIN_1']) #read a bit from Lt odom pin_1 

		data[0] =((data[0] << 1) + readBit[0])	#form each bit into a 16 bit package
		data[1] =((data[1] << 1) + readBit[1])	#using a shift left each bit can be pushed to create the word

		a += 1

	return ((data[1] << 16) | data[0])		# return the two data packets as one 32 bit data block

'''
	This function will take in a bit length and return a chunk of that
	number of a fixed length. This is called from the main().
'''
def bit_slicer(startBit, lowBit, count):
	mask = (1 << count) - 1
	return (startBit >> lowBit) & mask

'''
	This function will handle any rollovers that may occur during
	runtime.
	It will detect a rollover as a jump in readings by > 512 and use this to achieve a continuous distance count.
	This function is called from main().
'''
def handle_rollovers(readings, constants, prevReadings, prevRollovers):
	# initialise local variables
	values = [0,0]

	rolloverRange = 1 << (constants['READING_BIT_LENGTH'])
	big_jump = rolloverRange / 2
	for i in range(0,2):
		change = readings[i] - prevReadings[i]
		prevReadings[i] = readings[i]
		prevRollovers[i] += -1 if (change > big_jump) else (change < -big_jump)
		values[i] = readings[i] + rolloverRange * prevRollovers[i]
	return values

'''
	This function will initialise the pins.
	It is passed in constants from the main() and uses them to set the pins
	to either input to output after initially clearing the pins to their
	default state.
'''
def init_pins():
	# initialise the Dictionary
	constants = {
		'DATA_PIN_0' : 22,		#Data Right Odom GPIO 22  (was 4)
		'DATA_PIN_1' : 18,		#Data Left Odom  GPIO 18
		'CHIP_SELECT_PIN' : 24,	#Chip Select     GPIO 24
		'CLOCK_PIN' : 23,		#Clock           GPIO 23
		'TICK' : 0.01,
		'READING_LOW_0_BIT' : 6,      #bits 6-16 contain rotation data from Lt odom
		'READING_LOW_1_BIT' : 22,     #bits 22 to 32 contain rotation data from Rt odom
		'READING_BIT_LENGTH' : 10,    #10 binary bits contain a decimal number of 1024 
		'ROLLOVER_RANGE' : 1024
	}

	GPIO.setwarnings(False)
	GPIO.cleanup()
	GPIO.setmode(GPIO.BCM)

	GPIO.setup(constants['DATA_PIN_0'], GPIO.IN)
	GPIO.setup(constants['DATA_PIN_1'], GPIO.IN)
	GPIO.setup(constants['CHIP_SELECT_PIN'], GPIO.OUT)
	GPIO.setup(constants['CLOCK_PIN'], GPIO.OUT)

	return constants

'''
	This function will be the main function of the program. It creates a
	Dictionary of constants that can be used in other functions.
	The function will setup the GPIO pins and then run an infinite loop that
	will read from the odometers, slice the reading into a readable chunk,
	and then handle any rollovers that might occur before outputting the
	result.
'''
def main():
	# initialise local variables
	crashed = False
	mode = "Continuous"
	values = [0,0]
	readings = [0,0]
	prevReadings = [0,0]
	prevRollovers = [0,0]
	init_readings = [0,0]
	is_first = 0
	pygame.init()		# initialise pygame

	displayWidth = 190
	displayHeight = 100
	screen = pygame.display.set_mode((displayWidth,displayHeight))	# set display parameters of the window

	pygame.display.set_caption("Odo Test")		# set title of the window

	# preset colours to use
	black = (0,0,0)
	white = (255,255,255)
	red = (255,0,0)

	font = pygame.font.SysFont('Avenir Next', 20)		# preset font to use

	constants = init_pins()		# initialise and setup the pins to use

	while not crashed:		# infinite loop with a catch variable
		for event in pygame.event.get():
			if event.type == pygame.QUIT:		# safe quit on closing the window
				pygame.quit()
				sys.exit()
			if event.type == pygame.KEYDOWN:		# look for key presses
				if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:		# safe quit on "q" press or "ESC" press
					pygame.quit()
					sys.exit()
				elif event.key == pygame.K_c:
					if mode == "Rollover":
						mode = "Continuous"
					elif mode == "Continuous":
						mode = "Rollover"


		data = read_raw(constants)		# read from odometers

		readings[0] = bit_slicer(data,constants['READING_LOW_0_BIT'],constants['READING_BIT_LENGTH'])
		readings[1] = bit_slicer(data,constants['READING_LOW_1_BIT'],constants['READING_BIT_LENGTH'])
		
		if mode == "Continuous":
			values = handle_rollovers(readings,constants, prevReadings, prevRollovers)		# handle rollovers
		elif mode == "Rollover":
			values = readings
			
		screen.fill(black)		# set the screen background

		screen.blit(font.render("Odometers: (left, right)",True,white),(15,5))
		screen.blit(font.render(str((int(values[0]), int(constants['ROLLOVER_RANGE']-values[1]))),True,white),(95,25))
		screen.blit(font.render(str(("Mode: ", mode)),True,white), (15,45))
		screen.blit(font.render("Press 'c' to change mode",True,white),(15,65))

		pygame.display.update()        # update the display on each loop

# program start
if __name__ == "__main__":
	main()
	pygame.quit()
	quit()
