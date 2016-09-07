#desktop/testPrograms/odometerTest3.py   2/9/16
#This program tests the odometers on each wheel independently by hand movement
#Rollover and Continuous Modes are selected by keyboard c

#!/usr/bin/env python

# import libraries
import time
import serial
import sys
import pygame

# Initialise display parameters of Pygame display window
pygame.init()       
black = (0,0,0)         # Preset colours to use
white = (255,255,255)
red = (255,0,0)
displayWidth = 230      # Size of Window
displayHeight = 105
screen = pygame.display.set_mode((displayWidth,displayHeight))  
screen.fill(black)      # set the screen background to black
font = pygame.font.SysFont('Avenir Next', 20)   # Preset font to use
pygame.display.set_caption("Odo Test")     # Window Title

# Set up GPIO pins
try:    # try and import the GPIO library and catch a RunTimeError on fail
    import RPi.GPIO as GPIO
except RunTimeError as err: # catch the RunTimeError and output a response
    print err
    print ("Error: Can't import RPi.GPIO")

#Initialise GPIO
GPIO.setwarnings(False)
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)      #GPIO number designation (not pin nos)
GPIO.setup(18, GPIO.IN)     #Data pin Left Odometer
GPIO.setup(22, GPIO.IN)     #Data pin Right Odometer
GPIO.setup(24, GPIO.OUT)    #Chip Select pin common to both odometers
GPIO.setup(23, GPIO.OUT)    #Clock pin common to both odometers

'''
   This function will read from the odometers directly and pass the
   result back to the main(). 
   The Odometers use a simple synchronised data link where the R-Pi
   pulses a clock pin high and low to tell the odometers to release
   individual bits of data which we assemble into separate 16 bit words.
   Each 360 degree rotation of the wheel is divided up into 1024 steps.
   Every rotation the data counts from 0 to 1024 and then starts again.
   We call this a rollover and identify it by a step change in data
   from 0 to 1024 or visa versa.
   The GPIO Clock pins on both odometers are connected together so that
   the 16 data bits are clocked out simultaneously.
   The GPIO Chip Select Pins are also connected together so that both
   odometers are triggered to start together and data readings
   are sent simultaneously so there is no distance lag between wheels.
   The Individual Data values are read separately from different GPIO pins.
'''
'''
def read_raw(constants):
    # initialise local variables
    a = 0               # bit count index number
    readBit = [0,0]
    data = [0,0]

    # Bring the chip select pin high and then low before reading data.
    GPIO.output(constants['CHIP_SELECT_PIN'],GPIO.HIGH)
    time.sleep(constants['TICK'])
    GPIO.output(constants['CLOCK_PIN'],GPIO.HIGH)
    time.sleep(constants['TICK'])
    GPIO.output(constants['CHIP_SELECT_PIN'],GPIO.LOW)
    time.sleep(constants['TICK'])

    # loop 16 times to get a 16 bit data read from the odometers
    while a < (constants['READING_BIT_LENGTH'] + constants['READING_LOW_0_BIT']):   
        # as the data pins are read the clock is pulsed high and low
        GPIO.output(constants['CLOCK_PIN'], GPIO.LOW)	
        time.sleep(constants['TICK'])
        GPIO.output(constants['CLOCK_PIN'],GPIO.HIGH)
        time.sleep(constants['TICK'])
        readBit[0]= GPIO.input(constants['DATA_PIN_0']) #read a bit from Rt odom pin_0
        readBit[1]= GPIO.input(constants['DATA_PIN_1']) #read a bit from Lt odom pin_1 
        
        # using a shift left each bit can be pushed to create the word
        data[0] =((data[0] << 1) + readBit[0])	#form each bit into a 16 bit word
        data[1] =((data[1] << 1) + readBit[1])	
        
        a += 1
    return ((data[1] << 16) | data[0])	# return the two data packets as one 32 bit data block
'''
'''
   The read_odometers function will read angular and status data from the
   odometers directly and pass the result back to the main(). 
'''
def read_Odometers():
    
    # initialise local variables
    TICK = 0.000005     #Half Odom Serial clock period 5us=100K bits/s(Max 1MHz)
    i = 15              #bit count index number
    readBitLt = 0       #Lt Odom Data bit
    readBitRt = 0       #Rt Odom Data bit
    angDataLt = 0       #Lt Odom angular data in 10 bit word
    angDataRt = 0       #Rt Odom angular data in 10 bit word
    statusLt = 0        #Lt Odom status in 6 bit word
    statusRt = 0        #Rt Odom status in 6 bit word   

    # Read the Odometers
    # Bring the chip select pin high and then low before reading data.
    GPIO.output(24, True)   #Chip Select pin High (normal state)
    time.sleep(TICK)        #Half odom Serial clock period
    GPIO.output(23, True)   #Clock pin High (normal state)
    time.sleep(TICK)
    GPIO.output(24, False)  #Chip Select pin Low (both odom triggered to output data)
    time.sleep(TICK)        #Wait min of 500ns
    
    # bit data changes on each rising edge of clock 
    while i >= 0:   # loop 16 times to read 16 bits of data from the odometers
                    # the most significant bit (msb) is first (bit 15) 
                    # the least significant bit (lsb) is last (bit 0)   
        # pulse the clock Low and High and read data pins high and low
        GPIO.output(23, False)  #Clock pin Low
        time.sleep(TICK)
        GPIO.output(23, True)   #Clock pin High 
        time.sleep(TICK)        #Wait half clock period and read bit data
        
        readBitLt = GPIO.input(18)    #read a bit from Lt odometer Data pin
        readBitRt = GPIO.input(22)    #read a bit from Rt odometer Data pin

        # shift each bit left to form two 10 bit binary words for angle data
        # and two 5 bit binary words for chip and magnet status 
        #The first bit received is the most significant bit (msb bit 15)
        if i > 5:  #first 10 bits contain angular rotation data (0 to 1024)
            angDataLt =((angDataLt << 1) + readBitLt)   #bits 15 to 6
            angDataRt =((angDataRt << 1) + readBitRt) 
        else:      #last 6 bits contain odometer status data for analysis
            statusLt = ((statusLt << 1) + readBitLt)    #bits 5 to 0
            statusRt = ((statusRt << 1) + readBitLt)

        i -= 1      #decrement bit count index number

    time.sleep(TICK)        #Complete clock cycle
    GPIO.output(24, True)   #Chip Select pin High (normal state)
    return (angDataLt, angDataRt, statusLt, statusRt)

'''    
    The handle_rollovers function will detect any rollovers that may
    occur during runtime and thereby achieve a continuous distance count
    A rollover is a jump in angDataLt or angDataRt between 0 and 1024
    One wheel rotation is divided into 3 equal sectors (1024/3 = 341 bits)
    if the new angle & the previous angles are in sectors adjacent to the jump
    a rollover has occurred and 1024 is added or subtracted from odom distance
    Note: Time interval between odometer reads must be less than time taken
    for wheel to rotate 2/3 of turn (this determines max wheel speed allowed)
    This function is called from main().
'''
def handle_rollovers(angDataLt,angDataRt,prevAngDataLt,prevAngDataRt,\
    prevOdomDistLt,odomDistRt):
    # Calculate change in odometer angles
    changeLt = angDataLt - prevAngDataLt  #change in dataLt since last reading
    changeRt = angDataRt - prevAngDataRt  #change in dataRt since last reading

    if (angDataLt <341) and (prevAngDataLt >683):  #positive rollover sector
        odomDistLt = prevOdomDistLt + 1024 + changeLt
    elif (angDataLt >683) and (prevAngDataLt <341):#negative rollover sector
        odomDistLt = prevOdomDistLt - 1024 + changeLt
    else:
        odomDistLt = prevOdomDistLt + changeLt         #no rollover 

    if (angDataRt <341) and (prevAngDataRt >683):  #positive rollover
        odomDistRt = odomDistRt + 1024 + changeRt
    elif (angDataRt >683) and (prevAngDataRt <341):#negative rollover
        odomDistRt = odomDistRt - 1024 + changeRt
    else:
        odomDistRt = odomDistRt + changeRt         #no rollover
    
    prevAngDataLt = angDataLt     #load new dataLt into prevDataLt for next loop
    prevAngDataRt = angDataRt     #load new dataRt into prevDataRt for next loop
            
    return(odomDistLt,odomDistRt,prevAngDataLt,prevAngDataRt)

'''
    The bit_slicer function will take in a bit length and return a chunk
    of that number of a fixed length. This is called from main().
'''
'''
#NO NEED TO USE 32 BIT PACKAGE. BIT SLICE EACH WORD SEPARATELY (bits 6 to 15) TO EXTRACT ODOM DATA
def bit_slicer(startBit, lowBit, count):
    mask = (1 << count) - 1
    return (startBit >> lowBit) & mask
'''
'''
    The handle_rollovers function will handle any rollovers that
    may occur during runtime.
    It will detect a rollover as a jump in readings by > 512
    and use this to achieve a continuous distance count.
    This function is called from main().
'''
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
'''
	The init_pins function will initialise the GPIO pins.
	It is passed in constants from the main() and uses them to set the pins
	to either input to output after initially clearing the pins to their
	default state.
'''
'''
def init_pins():
    # initialise the Dictionary
    constants = {
        'DATA_PIN_0' : 22,      #Data Right Odom GPIO 22  (was 4)
	'DATA_PIN_1' : 18,      #Data Left Odom  GPIO 18
	'CHIP_SELECT_PIN' : 24,	#Chip Select     GPIO 24
	'CLOCK_PIN' : 23,	#Clock           GPIO 23
	'TICK' : 0.01,
	'READING_LOW_0_BIT' : 6,    #bits 6-15 contain rotation data from Lt odom
	'READING_LOW_1_BIT' : 22,   #bits 22 to 31 contain rotation data from Rt odom
	'READING_BIT_LENGTH' : 10,  #10 binary bits contain a decimal number of 1024 
	'ROLLOVER_RANGE' : 1024     #1024 to 0 and visa versa
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
'''
    The main function is the main function of the program.
    It creates a Dictionary of constants that can be used
    in other functions.
    The function will setup the GPIO pins and then run
    an infinite loop that will read from the odometers,
    slice the reading into a readable chunk,
    and then handle any rollovers that might occur
    before outputting the result.
'''
def main():
    # Commands to be actioned once on start of program
    # initialise local variables
    mode = "Continuous"
    loop1 = True        #boolean variable to allow exit from main loop
    '''
    values = [0,0]
    readings = [0,0]
    prevReadings = [0,0]
    prevRollovers = [0,0]
    init_readings = [0,0]
    is_first = 0
    '''
    pygame.init()   # initialise pygame
    odomDistLt = 0      #start condition for distance travelled by left wheel
    odomDistRt = 0      #start condition for distance travelled by right wheel
    prevAngDataLt= 0    #start condition for Lt odom angle data on previous read
    prevAngDataRt= 0    #start condition for Rt odom angle data on previous read
    
    # Read odometers once to get wheel angle offsets at robot start position
    (angDataLt,angDataRt, statusLt,statusRt) = read_Odometers() #function call
                                                #to obtain raw data and status
    prevAngDataLt = angDataLt   #start condition for prevAngDataLt
    prevAngDataRt = angDataRt   #start condition for prevAngDataRt 

    '''
    constants = init_pins()     # initialise and setup the pins to use
    '''
    
    # Commands to be actioned endlessly in a loop until program stopped    
    while loop1 == True:    #loops until loop1 is declared False

        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:   # safe quit on closing the window
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:    # look for key presses
                # safe quit on "q" press or "ESC" press
                if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:		
                    pygame.quit()
                    sys.exit()
                '''
                elif event.key == pygame.K_c:
                    if mode == "Rollover":
                        mode = "Continuous"
                    elif mode == "Continuous":
                        mode = "Rollover"
                '''

        # read odometers for raw angle data and status
        (angDataLt,angDataRt,statusLt,statusRt) = read_Odometers()

        # calculate odometer distances by actioning rollovers
        (odomDistLt,odomDistRt,prevAngDataLt,prevAngDataRt)= handle_rollovers\
            (angDataLt,angDataRt,prevAngDataLt,prevAngDataRt,\
             odomDistLt,odomDistRt)
        # correct for right odometer and motor operating in reverse
        correctedOdomDistLt1 = odomDistLt 
        correctedOdomDistRt1 = -odomDistRt
        
        '''
        data = read_raw(constants)		# read from odometers

        readings[0] = bit_slicer(data,constants['READING_LOW_0_BIT'],constants['READING_BIT_LENGTH'])
        readings[1] = bit_slicer(data,constants['READING_LOW_1_BIT'],constants['READING_BIT_LENGTH'])
		
        if mode == "Continuous":
            values = handle_rollovers(readings,constants, prevReadings, prevRollovers)	# handle rollovers
        elif mode == "Rollover":
            values = readings
        '''			
        # Pygame Display Code
        # update the pygame data display
        screen.fill(black)  #Blank out previous digits in display window

        # display odometer output text and values to the screen
        screen.blit(font.render("Odometers: ",True,white),(15,5))
        screen.blit(font.render("(left,   right)",True,white),(135,5))
        screen.blit(font.render(str(angDataLt),True,white),(140,25))
        screen.blit(font.render(str(angDataRt),True,white),(175,25))
        screen.blit(font.render(str(odomDistLt),True,white),(140,45))
        screen.blit(font.render(str(odomDistRt),True,white),(175,45))
        # test displays to ensue correct odom readings
        screen.blit(font.render("Raw",True,white),(15,25))
        screen.blit(font.render("Raw Continuous",True,white),(15,45))
        screen.blit(font.render("RC  Continuous",True,white),(15,65))
        screen.blit(font.render(str(correctedOdomDistLt1),True,white),(140,65))
        screen.blit(font.render(str(correctedOdomDistRt1),True,white),(175,65))
        '''              
        screen.blit(font.render(str(int(values[0])),True,white),(95,25))
        screen.blit(font.render(str(int(constants['ROLLOVER_RANGE']-values[1])),True,white),(125,25))
        screen.blit(font.render(str(("Mode: ", mode)),True,white), (15,45))
        screen.blit(font.render("Press 'c' to change mode",True,white),(15,65))
        '''
        pygame.display.update()        # update the display on each loop

# program start
main()
pygame.quit()
sys.exit
quit()
