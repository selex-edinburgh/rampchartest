# desktop/testPrograms/odometerTest4.py   12/9/16
# This program tests the odometers on each wheel independently
#  by hand movement
# Rollover and Continuous Modes are selected by keyboard c
# End = Exit program        BUG

# import libraries
import sys              # import standard python module
import serial           # import standard python module
import time             # import standard python module
import math             # import standard python module
import pygame           # import pygame

# Initialise display parameters of Pygame display window
pygame.init()       
black = (0,0,0)         # Preset colours to use
white = (255,255,255)
red = (255,0,0)
displayWidth = 370      # Size of Window
displayHeight = 100      #
screen = pygame.display.set_mode((displayWidth,displayHeight))  
screen.fill(black)      # set the screen background to black
font = pygame.font.SysFont('Avenir Next', 18) # Preset font & size
pygame.display.set_caption("Odom Test")       # Window Title


'''
    The Odometers use a simple synchronised two way data link.
    The GPIO pins are used for this serial link and must be designated and
    the direction of data flow defined.
'''
# Set up GPIO pins
try:    # try and import the GPIO library and catch a RunTimeError on fail
    import RPi.GPIO as GPIO
except RunTimeError as err: # catch the RunTimeError and output a response
    print err
    print ("Error: Can't import RPi.GPIO")

#Initialise GPIO
GPIO.setwarnings(False)     #Inhibits display of unwanted GPIO warnings  
GPIO.cleanup()              #Clears any previous GPIO pin designations
GPIO.setmode(GPIO.BCM)      #GPIO number designation (not pin nos)
GPIO.setup(18, GPIO.IN)     #Data pin Left Odometer
GPIO.setup(22, GPIO.IN)     #Data pin Right Odometer
GPIO.setup(24, GPIO.OUT)    #Chip Select pin common to both odometers
GPIO.setup(23, GPIO.OUT)    #Clock pin common to both odometers

'''
   The read_odometers function will read angle and status data from the
   odometers directly and pass the result back to the main(). 

   The Odometers use a simple synchronised two way data link.
   The GPIO Chip Select pins on both odometers are connected together
   so that both odometers are triggered to start simultaneously.
   The GPIO Clock pins on both odometers are also connected together
   so that the 16 data bits are clocked out simultaneously.
   The Odometer Data output pins are connected to separate GPIO pins
   and the bit values are read separately.
   We assemble these individual bits of data into separate 16 bit words.
   Each 360 degree rotation of the wheel is divided up into 1024 steps.
   During every rotation the data counts from 0 to 1024 and then starts again.
   The step change between 0 and 1024 and visa versa is called a rollover
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
    This is the main function of the program. 
    First it calls the readOdometers function once to obtain the raw odometer
    angle information for the start position (zero distance). 
    
    It then starts a continuous loop, within which commands are actioned
    endlessly until the program is stopped.
    The odometer code section sets up the GPIO pins and within an infinite
    loop, reads from the odometers and handles any rollovers before
    outputting the result.
'''
def main():
    # Commands to be actioned once on start of program
    # initialise data links and local variables 
    pygame.init()       #call Function to Initialise Pygame
    loop1 = True        #boolean variable to allow exit from main loop
    mode = "Continuous"
    loop1 = True        #boolean variable to allow exit from main loop
    odomDistLt = 0      #start condition for distance travelled by left wheel
    odomDistRt = 0      #start condition for distance travelled by right wheel
    prevAngDataLt= 0    #start condition for Lt odom angle data on previous read
    prevAngDataRt= 0    #start condition for Rt odom angle data on previous read
    
    # Read odometers once to get wheel angle offsets at robot start position
    (angDataLt,angDataRt, statusLt,statusRt) = read_Odometers() #function call
                                                #to obtain raw data and status
    startAngLt = angDataLt    #start condition for angDataLt
    startAngRt = angDataRt    #start condition for angDataRt
    prevAngDataLt = angDataLt   #start condition for prevAngDataLt
    prevAngDataRt = angDataRt   #start condition for prevAngDataRt
    '''
    Commands to be actioned endlessly in a loop until program stopped    
    '''
    while loop1 == True:    #loops until loop1 is declared False

        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:   # safe quit on closing the window
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:    # look for key presses
                # safe quit on "END" press or "ESC" press
                if event.key == pygame.K_END or event.key == pygame.K_ESCAPE:		
                    pygame.quit()
                    sys.exit()

        # read odometers for raw angle data and status
        (angDataLt,angDataRt,statusLt,statusRt) = read_Odometers()

        # correct for odometer start position for display
        correctedStartAngLt = angDataLt - startAngLt
        correctedStartAngRt = angDataRt - startAngRt
        if correctedStartAngLt < 0:
            correctedStartAngLt = correctedStartAngLt + 1024
        if correctedStartAngRt < 0:
            correctedStartAngRt = correctedStartAngRt + 1024

        # calculate odometer distances by actioning rollovers
        (odomDistLt,odomDistRt,prevAngDataLt,prevAngDataRt)= handle_rollovers\
            (angDataLt,angDataRt,prevAngDataLt,prevAngDataRt,\
             odomDistLt,odomDistRt)
        # correct for right odometer and motor operating in reverse
        odomDistRtReversed = -odomDistRt    #value for display
        

        # Pygame Display Code
        # update the pygame data display
        screen.fill(black)  #Blank out previous digits in display window


        # Display odometer output text and values to the screen
        screen.blit(font.render("Left Odom   Right Odom ",\
            True,white),(100,10))
        screen.blit(font.render("Rollover",True,white),(15,25))
        screen.blit(font.render("Raw Data",True,white),(235,25))
        screen.blit(font.render("Corrected Start Posn",True,white),(235,40))
        screen.blit(font.render("Corrected Start Posn",True,white),(235,60))
        screen.blit(font.render("Continuous",True,white),(15,60))
        screen.blit(font.render("Rt Odom Reversed",True,white),(235,75))

        # Display odometer test values to test for correct odom readings
        screen.blit(font.render(str(angDataLt),True,white),(123,25))
        screen.blit(font.render(str(angDataRt),True,white),(193,25))
        screen.blit(font.render(str(correctedStartAngLt),True,white),(123,40))
        screen.blit(font.render(str(correctedStartAngRt),True,white),(193,40))
        screen.blit(font.render(str(odomDistLt),True,white),(123,60))
        screen.blit(font.render(str(odomDistRt),True,white),(193,60))
        screen.blit(font.render(str(odomDistLt),True,white),(123,75))
        screen.blit(font.render(str(odomDistRtReversed),True,white),(193,75))


        '''
        # display odometer output text and values to the screen
        screen.blit(font.render("Odometers: ",True,white),(15,5))
        screen.blit(font.render("(left,   right)",True,white),(135,5))
        screen.blit(font.render("Raw",True,white),(15,25))
        screen.blit(font.render(str(angDataLt),True,white),(140,25))
        screen.blit(font.render(str(angDataRt),True,white),(175,25))
        screen.blit(font.render("Raw Continuous",True,white),(15,45))
        screen.blit(font.render(str(odomDistLt),True,white),(140,45))
        screen.blit(font.render(str(odomDistRt),True,white),(175,45))
        # test displays to ensue correct odom readings / might want to leave out\
            # the corrected right wheel value to be shown in combo program
        screen.blit(font.render("RC  Continuous",True,white),(15,65))
#        screen.blit(font.render(str(correctedOdomDistLt1),True,white),(140,65))
#        screen.blit(font.render(str(correctedOdomDistRt1),True,white),(175,65))
        '''
        '''              
        screen.blit(font.render(str(int(values[0])),True,white),(95,25))
        screen.blit(font.render(str(int(constants['ROLLOVER_RANGE']-values[1])),True,white),(125,25))
        screen.blit(font.render(str(("Mode: ", mode)),True,white), (15,45))
        screen.blit(font.render("Press 'c' to change mode",True,white),(15,65))
        '''
        pygame.display.update()        # update the display on each loop

# program start
main()

# Program exit when END key pressed.
pygame.quit()       # uninstall all pygame modules
sys.exit()          # exit from Python
