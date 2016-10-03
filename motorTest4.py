# desktop/testPrograms/motorTest4  12/9/16
# This program tests the Rampaging Chariot Motors operating under R-Pi control.
# The Master Motor Drive Board must have the AUTO/MANUAL jumper selected to AUTO.
# Battery power to the Motor Drive Boards must be ON
# Arrow keys increment Motor speed relative to datum of 127
# Space = Stop Motors
# End = Exit program        BUG

# import libraries
import sys              # import standard python module
import serial           # import standard python module
import time             # import standard python module
import pygame           # import pygame

'''
    Pygame is a set of Python modules designed for writing games.
    We are using it for display and graphics as it is very powerful,
    fast and easy to use. Its libaries contain core functions written
    in C and Assembly code which run very fast
    (this can easily be 100 or more times faster than python code).
    Pygame is free. Released under the General Public License,
    which means you can create open source, free, freeware, shareware,
    and commercial games with it. See the licence for full details.
'''
# Initialise display parameters of Pygame display window
pygame.init() 	        # Set display parameters of Pygame display window
black = (0,0,0) 	# Preset colours to use
white = (255,255,255)
red = (255,0,0)
displayWidth = 190 	# Size of Window
displayHeight = 75
screen = pygame.display.set_mode((displayWidth, displayHeight)) 	
screen.fill(black)	# set the screen background to black (default)
font = pygame.font.SysFont('Avenir Next', 18)	# Preset font & size
pygame.display.set_caption("Motor Test")	# Window Title

'''
    The setSerial function sets up the serial interface used to communicate 
    with the Rampaging Chariot Master Motor Drive Board.
    It should explain any errors that occur by using an (except) function.
    If you are using a Raspberry pi 3B the serial port is called:
    port = '/dev/ttyS0'
    If you are using a Raspberry Pi 2B  the serial port is called:
    port = '/dev/ttyAMA0' 
    Both use UART Tx pin 8 and Rx pin 10
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
    Four Telemetry bytes are expected.
    Bytes [0] & [1] are the fwd and turn bytes sent out.
    Byte [3] is output power. Byte [4] TBD
    This function is called from main()
'''
def getTelemetry(ser):
    try:
        telemetry = ser.read(4)
        #change ascii characters into integer value of that character
        #and return values to main()
        return ord(telemetry[0]),ord(telemetry[1]),ord(telemetry[2]),ord(telemetry[3]) 	
    except Exception as err:
        print err       #print any error warnings

'''
    This is the main and first function called by the program. 
    First it sets up the Serial Interface by calling setSerial.
    It then initialises the two chariot motor commands fwd & turn
    The Rampaging Chariot motors are controlled by two commands that 
    mimic the "Forward Speed" and "Turn Rate" commands sent manually via
    the standard radio control system.
    Each byte contains a number between 0 and 255.
    For the "fwd" byte, Stationary is 127 with greater numbers demanding
    increasing forward speed and lower numbers demanding increasing backwards speed.
    For the "turn" byte, No turn is 127 with greater numbers demanding
    increasing Left Turn rate and lower numbers demanding increasing Right Turn rate.

    The main function then starts a continuous loop, within which commands are
    actioned endlessly until the program is stopped.
    The motor code section listens for particular keyboard key presses. 
        Arrow keys increment and decrement Motor speed relative to datum of 127
        Space bar Stops the Motor movement
        End will exit program
    The motor control values are then written out to the Serial Interface and
    the pygame screen is then updated to show the results. 
'''
def main():
    # Commands to be actioned once on start of program
    # initialise data links and local variables 
    ser = setSerial()   #set up the Serial link to Master Motor Drive Board
    pygame.init()       #call Function to Initialise Pygame
    loop1 = True        #boolean variable to allow exit from main loop
    fwd = 127		#motor command no forward movement
    turn = 127		#motor command no Turn Rate
    fwdMax = 126        #max fwd speed (126 or less)
    turnMax = 126       #max turn rate (126 or less)
    status = "running"  #Program status
    ending = True
    
    # Commands to be actioned endlessly in a loop until program is stopped    
    
    while loop1 == True:    #loops until loop1 is declared False

        # Motor Code
        for event in pygame.event.get():
            # if a designated keyboard key is pressed do following actions:
            if event.type == pygame.KEYDOWN:	    #detect a key press
                if event.key == pygame.K_UP:	    #increase forward speed
                    fwd += 5
                    if fwd > 247:                   #limit fwd output
                        fwd = 247                   
                if event.key == pygame.K_SPACE:     #STOP
                    fwd  = 127                      #stop motors
                    turn = 127
                    
                if event.key == pygame.K_DOWN:      #decrease forward speed 
                    fwd -= 5
                    if fwd < 7:
                        fwd = 7                     #limit fwd output
                if event.key == pygame.K_RIGHT:     #increase Right turn
                    turn += 5
                    if turn > 247:                  #limit turn output
                        turn = 247                  
                if event.key == pygame.K_LEFT:      #decrease Right Turn
                    turn -= 5
                    if turn < 7:                    #limit turn output
                        turn = 7                    
                if event.key == pygame.K_END:       #END PROGRAM  (End Key)
                    fwd  = 127                      #no forward movement   
                    turn = 127                      #no turn movement
                    status = "stop"                 #enters endless safe loop
                    
        # limit values & write the fwd and turn to the motors via serial link
        if fwdMax > 126:
            fwdMax = 126    #valid number wrt 127 to fit within a single byte
            
        if (fwd < 127 + fwdMax) and (fwd > 127 - fwdMax):   #valid range 
            ser.write(chr(fwd))         #send fwd command to motor board
        else:
            fwd = 127                   #no forward movement
            ser.write(chr(fwd))         #send fwd command to motor board

        if turnMax > 126:
            fwdMax = 126    #valid number wrt 127 to fit within a single byte

        if (turn < 127 + turnMax) and (turn > 127 - turnMax): #valid range
            ser.write(chr(turn))        #send turn command to motor board
        else:
            turn = 127                  #no turn movement
            ser.write(chr(turn))        #send turn command to motor board
                    
#        telemetry = getTelemetry(ser)  #Receive Telemetry from chariot serial
                

        # Pygame Display Code
        # update the pygame data display
        screen.fill(black)  #Blank out previous digits in display window

	# display text to the screen,
	#blit can also render images and draw on the screen
        screen.blit(font.render("Fwd Speed   Turn Rate",True,white),(15,5))
        screen.blit(font.render(str(fwd),True,white),(50,25))
        screen.blit(font.render(str(turn),True,white),(100,25))
#        screen.blit(font.render("Status:",  status,True,white),(15,45))
        screen.blit(font.render(status,True,white),(65,45))

        #Display telemetry values  -Not Used
#        screen.blit(font.render(str(telemetry[0]),True,white),(50,25))
#        screen.blit(font.render(str(telemetry[1]),True,white),(100,25))
#        screen.blit(font.render(str(telemetry[2]),True,white),(50,45))
#        screen.blit(font.render(str(telemetry[3]),True,white),(100,45))

        pygame.display.update() # update the display on each loop
            
        time.sleep(0.016)	#Wait for 16 msec and repeat loop

        
        if status == "stop":
            while ending == True:    #endless loop requiring ctrl+c to quit program
                time.sleep(0.016)
                fwd  = 127
                turn = 127
                ser.write(chr(fwd))         #send fwd command to motor
                ser.write(chr(turn))        #send turn command to motor
                ending = False
            loop1 = False

# Program start            
main()              # call main() function containing a continuous loop

# Program exit                                      ???when END key pressed.
pygame.quit()       # uninstall all pygame modules
sys.exit()          # exit from Python
