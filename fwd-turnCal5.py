#desktop/testPrograms/fwd-turnCal4.pi   3/9/16
#This program combines MotorTest and OdometerTest to allow calibration of the
#wheel/odometer combination in a straight line and in a turn on-the-spot.
#s = start movement forward
#r = start movement reverse
#space = stop movement

import time
import serial
import sys
import pygame

# Initialise display parameters of Pygame display window
pygame.init()       
black = (0,0,0)         # Preset colours to use
white = (255,255,255)
red = (255,0,0)
displayWidth = 250      # Size of Window
displayHeight = 105
screen = pygame.display.set_mode((displayWidth,displayHeight))  
screen.fill(black)      # set the screen background to black
font = pygame.font.SysFont('Avenir Next', 20)   # Preset font to use
pygame.display.set_caption("Fwd-Turn Calibrate")     # Window Title

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
    The setSerial function will set up the serial interface UART used to
    communicate with the Rampaging Chariot Master Motor Drive Board.
    Change the port designation dependant on R-Pi 2B or 3B.
    Both use UART Tx pin 8 and Rx pin 10
'''
def setSerial():
    try:
        ser= serial.Serial(         #Set up Serial Interface
            port = '/dev/ttyS0',        #If a Raspberry Pi 3B 
#            port = '/dev/ttyAMA0',  #If a Raspberry Pi 2B 
            baudrate = 38400,       #bits/sec
            bytesize=8, parity='N', stopbits=1,     #8-N-1  protocal 
            timeout=1                               #1 sec
        )
        return ser
    except Exception as err:
        print err

'''
    The getTelemetry function listens for a reply from the Master Motor Drive Board
    Four Telemetry bytes are expected.
    First two are the fwd and turn bytes sent out. second two are TBD
'''
def getTelemetry(ser):  #Not currently used in this program
    try:
        telemetry = ser.read(4)
        # changes ascii characters into the integer value of that character
        return ord(telemetry[0]),ord(telemetry[1]),ord(telemetry[2]),ord(telemetry[3])  
    except Exception as err:
        print err

'''
   The read_odometers function will read angular and status data from the
   odometers directly and pass the result back to the main(). 
'''
def read_Odometers():
    
    # initialise local variables
    tick = 0.000005     #Half Odom Serial clock period 5us=100K bits/s(Max 1MHz)
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
    time.sleep(tick)        #Half odom Serial clock period
    GPIO.output(23, True)   #Clock pin High (normal state)
    time.sleep(tick)
    GPIO.output(24, False)  #Chip Select pin Low (both odom triggered to output data)
    time.sleep(tick)        #Wait min of 500ns
    
    # bit data changes on each rising edge of clock 
    while i >= 0:   # loop 16 times to read 16 bits of data from the odometers
                    # the most significant bit (msb) is first (bit 15) 
                    # the least significant bit (lsb) is last (bit 0)   
        # pulse the clock Low and High and read data pins high and low
        GPIO.output(23, False)  #Clock pin Low
        time.sleep(tick)
        GPIO.output(23, True)   #Clock pin High 
        time.sleep(tick)        #Wait half clock period and read bit data
        
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

    time.sleep(tick)        #Complete clock cycle
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
    First it sets up the Serial Interface by calling setSerial. 
    It then initialises the two motor commands fwd & turn
    Finally a loop is called that will listen and filter any keyboard events. 
    Q and ESC will both safely kill the program.
    s will start the calibration movement
    c will change between straight line and turn on the spot
    space will stop the movement.

    This function will also setup the GPIO pins and within an infinite loop,
    read from the odometers and handle any rollovers before outputting the
    result.
'''
def main():
    # Commands to be actioned once on start of program
    # initialise local variables
    ser = setSerial()   #set up the Serial link to Master Motor Drive Board
    pygame.init()       #call Function to Initialise Pygame
    loop1 = True        #boolean variable to allow exit from main loop
    fwd = 127		#motor command no forward movement
    turn = 127		#motor command no Turn Rate
    odomDistLt = 0      #start condition for distance travelled by left wheel
    odomDistRt = 0      #start condition for distance travelled by right wheel
    prevAngDataLt= 0  #start condition for Lt odom angle data on previous read
    prevAngDataRt= 0  #start condition for Rt odom angle data on previous read

    # Read odometers once to get wheel angle offsets at robot start position
    (angDataLt,angDataRt, statusLt,statusRt) = read_Odometers() #function call
                                                #to obtain raw data and status
    odomAngOffsetLt = angDataLt    #store left wheel offset
    odomAngOffsetRt = angDataRt    #store right wheel offset
#    prevAngDataLt = angDataLt   #start condition for prevAngDataLt
#    prevAngDataRt = angDataRt   #start condition for prevAngDataRt 

    # Commands to be actioned endlessly in a loop until program stopped    
    while loop1 == True:    #loops until loop1 is declared False

        # Motor Code
        for event in pygame.event.get():
            # if a designated keyboard key is pressed do following actions:
            if event.type == pygame.KEYDOWN:    #detect a key press
                if event.key == pygame.K_s:     #key s 'START fwd'
                    fwd  = 157                  #forward
                    turn = 127                  #no turn movement                 
                if event.key == pygame.K_r:     #key r 'START reverse'
                    fwd  = 97                   #reverse
                    turn = 127                  #no turn movement
                if event.key == pygame.K_SPACE: #STOP  (Space Bar)
                    fwd  = 127                  #no forward movement
                    turn = 127                  #no turn movement
                if event.key == pygame.K_END:   #STOP  (End Key)
                    fwd  = 127                  #no forward movement   
                    turn = 127                  #no turn movement
                if event.key == pygame.K_RIGHT: #key right arrow 'START right'
                    fwd  = 127                  #no forward movement 
                    turn = 157                  #turn right
                if event.key == pygame.K_LEFT:  #key left arrow 'START left'
                    fwd  = 127                  #no forward movement 
                    turn = 97                   #turn left
        # limit values & write the fwd and turn to the motors via serial link
        if (fwd < 254) and (fwd > 1):   #valid range to fit in an 8bit byte
            ser.write(chr(fwd))         #send fwd command to motor board
        else:
            fwd = 127                   #no forward movement
            ser.write(chr(fwd))         #send fwd command to motor board
        if (turn < 254) and (turn > 1): #valid range to fit in an 8bit byte
             ser.write(chr(turn))       #send turn command to motor board
        else:
            turn = 127                  #no turn movement
            ser.write(chr(turn))        #send turn command to motor board

#       telemetry = getTelemetry(ser)  #Receive Telemetry from chariot serial


        # Odometer Code
        # read odometers for raw angle data and status
        (angDataLt,angDataRt,statusLt,statusRt) = read_Odometers()

        # calculate odometer distances by actioning rollovers
        (odomDistLt,odomDistRt,prevAngDataLt,prevAngDataRt)= handle_rollovers\
            (angDataLt,angDataRt,prevAngDataLt,prevAngDataRt,\
             odomDistLt,odomDistRt)
        # correct for right odometer and motor operating in reverse
        correctedOdomDistLt1 = odomDistLt 
        correctedOdomDistRt1 = -odomDistRt
        # correct for both odometers initial position at start
        correctedOdomDistLt2 = correctedOdomDistLt1 - odomAngOffsetLt
        correctedOdomDistRt2 = correctedOdomDistRt1 + odomAngOffsetRt
        # correct for wheel diameters to get both readings in mm from start
        #code TBD

        
        
        # Pygame Display Code
        # update the pygame data display
        screen.fill(black)  #Blank out previous digits in display window

        # display motor output text and values to the screen
        screen.blit(font.render("Fwd Speed   Turn Rate",True,white),(15,5))
        screen.blit(font.render(str(fwd),True,white),(45,25))
        screen.blit(font.render(str(turn),True,white),(123,25))

        # display odometer output text and values to the screen
        screen.blit(font.render("Left Odom   Right Odom    Heading",True,white),(15,45))
        screen.blit(font.render(str(angDataLt),True,white),(45,60))
        screen.blit(font.render(str(angDataRt),True,white),(123,60))
        screen.blit(font.render(str(odomDistLt),True,white),(45,70))
        screen.blit(font.render(str(odomDistRt),True,white),(123,70))
        #test displays to ensue correct odom readings
        screen.blit(font.render(str(correctedOdomDistLt1),True,white),(45,80))
        screen.blit(font.render(str(correctedOdomDistRt1),True,white),(123,80))
        screen.blit(font.render(str(correctedOdomDistLt2),True,white),(45,90))
        screen.blit(font.render(str(correctedOdomDistRt2),True,white),(123,90))
        
        #Display telemetry values
#        screen.blit(font.render(str(telemetry[0]),True,white),(50,25))
#        screen.blit(font.render(str(telemetry[1]),True,white),(100,25))
#        screen.blit(font.render(str(telemetry[2]),True,white),(50,45))
#        screen.blit(font.render(str(telemetry[3]),True,white),(100,45))

        pygame.display.update() # update the display on each loop

        time.sleep(0.016)       #Wait for 16 msec and repeat loop

# Program start            
main()
fwd, turn = 127, 127            #Stop Motors
ser.write(chr(fwd))             #Send Motor Commands
ser.write(chr(turn))
pygame.quit()
sys.exit()
quit()

