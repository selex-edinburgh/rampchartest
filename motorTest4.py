#desktop/testPrograms/motorTest4  3/9/16
#This program tests the Rampaging Chariot Motors operating under R-Pi control.
#The Master Motor Drive Board must have the AUTO/MANUAL jumper selected to AUTO.
#Battery power to the Motor Drive Boards must be ON
#Arrow keys control Motor speed relative to datum of 127
#Space bar Stops Motors

#!/usr/bin/env python

import time
import serial
import sys
import pygame

'''
    Pygame is a set of Python modules designed for writing games.
    We are using it for display and graphics as it is very powerful,
    fast and easy to use. Its libaries contain core functions written
    in C and Assembly code which run very fast
    (can easily be 100 or more times faster than python code).
    Pygame is free. Released under the General Public License,
    which means you can create open source, free, freeware, shareware,
    and commercial games with it. See the licence for full details.
'''
pygame.init() 	        # Set display parameters of Pygame display window
black = (0,0,0) 	# Preset colours to use
white = (255,255,255)
red = (255,0,0)
displayWidth = 190 	# Size of Window
displayHeight = 75
screen = pygame.display.set_mode((displayWidth,displayHeight)) 	
screen.fill(black)	# set the screen background to black (default)
font = pygame.font.SysFont('Avenir Next', 20)	# Preset font to use
pygame.display.set_caption("Motor Test")	        # Window Title

'''
    The setSerial function sets up the serial interface used to communicate 
    with the Rampaging Chariot Master Motor Drive Board.
    It should explain any errors that occur by using an (except) function.
    If you are using a Raspberry pi 3B the serial port is called:
    port = '/dev/ttyS0'
    If you are using a Raspberry Pi 2B  the serial port is called:
    port = '/dev/ttyAMA0' 
    The 2B version of R-Pi is not recommended, but may work.
    This function is called from main()
'''
def setSerial():
    try:
        ser= serial.Serial(         #Set up Serial Interface
	    port = '/dev/ttyS0',    #If a Raspberry Pi 3B (UART using Tx pin8, Rx pin10)
#            port = '/dev/ttyAMA0',  #If a Raspberry Pi 2B (UART using Tx pin8, Rx pin10)
            baudrate = 38400,       #bits/sec
            bytesize=8, parity='N', stopbits=1, #8-N-1  protocal 
            timeout=1               #1 sec
        )
        return ser
    except Exception as err:
        print err

'''
    The getTelemetry function listens for a reply from the Master Motor Drive Board
    Two Telemetry bytes are expected.
    The ord function on the output changes ascii characters it receives 
    into the integer value of that character.
    This function is called from main()
'''
def getTelemetry(ser):
    try:
        telemetry = ser.read(4)
        return ord(telemetry[0]),ord(telemetry[1]),ord(telemetry[2]),ord(telemetry[3]) 	
    except Exception as err:
        print err

'''
    Here is where the first function call from when the program starts.
    The Rampaging Chariot motors are controlled by two commands that 
    mimic the "Forward Speed" and "Turn Rate" commands sent manually via
    the standard radio control system.
    Each byte contains a number between 0 and 255.
    For the "fwd" byte, Stationary is 127 with greater numbers demanding
    increasing forward speed and lower numbers demanding increasing backwards speed.
    For the "turn" byte, No turn is 127 with greater numbers demanding
    increasing Left Turn rate and lower numbers demanding increasing Right Turn rate.

    The main will start off by initialising the two motor commands. 
    Then it will set up the Serial Interface by calling setSerial. 
    Finally a loop is called that will listen and filter any keyboard events. 
    In this case it will listen for the key presses of: Q, ESC, Up, Down, Left and Right.
    Q and ESC will both safely kill the program. 
    Up and Down will put the fwd speed up or down by a positive or negative increment of 5,
    Left and Right will change the turn rate by a similar increment.
    These values are then written out to the Serial Interface and the 
    screen is then updated to show the results. 
'''

def main():
#    initPygame()        # call Function to Initialise Pygame
    ser = setSerial()   # set up the Serial link to the Master Motor Drive Board
    fwd = 127		# initialise Motor Commands:  no forward movement
    turn = 127		# no Turn Rate

    while True:         # infinite loop as true always evaluates to true
        for event in pygame.event.get():            #Listen for key presses
            if event.type == pygame.QUIT:
                fwd, turn = 127, 127                #Stop Motors
                ser.write(chr(fwd))                 #Send Motor Commands
                ser.write(chr(turn))
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:	    #detect a key press
                if event.key == pygame.K_UP:	    #increase forward speed
                    fwd += 5
                    if fwd > 247:
                        fwd = 247                   #limit fwd output
                if event.key == pygame.K_SPACE:     #STOP
                    fwd  = 127                      #stop motors
                    turn = 127                      
                elif event.key == pygame.K_DOWN:    #decrease forward speed 
                    fwd -= 5
                    if fwd < 7:
                        fwd = 7                     #limit fwd output
                elif event.key == pygame.K_RIGHT:   #increase Right turn
                    turn += 5
                    if turn > 247:
                        turn = 247                  #limit turn output
                elif event.key == pygame.K_LEFT:    #decrease Right Turn
                    turn -= 5
                    if turn < 7:
                        turn = 7                    #limit turn output
                elif event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                    fwd, turn = 127, 127            #Stop Motors
                    ser.write(chr(fwd))             #Send Motor Commands
                    ser.write(chr(turn)) 
                    pygame.quit()		    #exit programme
                    sys.exit()
                    
	# write the resulting fwd and turn to the motors via serial
	
        if (fwd < 254) and (fwd > 1):
            ser.write(chr(fwd))
        else:
            fwd = 127
            ser.write(chr(fwd))
            print "Too much fwd has been applied. Stopping the wheels"
        if (turn < 254) and (turn > 1):
            ser.write(chr(turn))
        else:
            turn = 127
            ser.write(chr(turn))
            print "Too much turn has been applied. Stopping the wheels"

        screen.fill(black)  #Blank out previous digits in display window

        telemetry = getTelemetry(ser)
                
	# display text to the screen, blit can also render images and draw on the screen
        screen.blit(font.render("Telemetry: (forward, turn)",True,white),(15,5))
        screen.blit(font.render(str(telemetry[0]),True,white),(50,25))
        screen.blit(font.render(str(telemetry[1]),True,white),(100,25))
        screen.blit(font.render(str(telemetry[2]),True,white),(50,45))
        screen.blit(font.render(str(telemetry[3]),True,white),(100,45))

        pygame.display.update()
            
        time.sleep(0.016)	#Wait for 16 msec and repeat loop

main()
pygame.quit()
quit()
