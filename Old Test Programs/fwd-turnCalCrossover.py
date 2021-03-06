# desktop/testPrograms/fwd-turnCal6.pi   12/9/16
# This program combines MotorTest and OdometerTest to allow calibration of the
# Wheel/odometer combination in a straight line and in a turn on-the-spot.
# Arrow keys = start movement forward
# Space = Stop movement
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
displayWidth = 300      # Size of Window  250 x 105
displayHeight = 160
screen = pygame.display.set_mode((displayWidth,displayHeight))  
screen.fill(black)      # set the screen background to black
font = pygame.font.SysFont('Avenir Next', 18)#Preset font & size
pygame.display.set_caption("Fwd-Turn Calibrate") #Window Title

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
    The setSerial function will set up the serial interface UART used to
    communicate with the Rampaging Chariot Master Motor Drive Board.
    Change the port designation dependant on R-Pi 2B or 3B.
    Both use UART Tx pin 8 and Rx pin 10
'''
def setSerial():
    try:
        ser= serial.Serial(         #Set up Serial Interface
            port = '/dev/ttyS0',    #If a Raspberry Pi 3B 
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
    It is currently not used
'''
def getTelemetry(ser):  #Not currently used in this program (development only)
    try:
        telemetry = ser.read(4)
        # change ascii characters into the integer value of that character
        #and return values to main()
        return ord(telemetry[0]),ord(telemetry[1]),ord(telemetry[2]),ord(telemetry[3])  
    except Exception as err:
        print err       #print any error warnings

'''
   The read_odometers function will read angle and status data from the two
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
        # pulse the clock pin Low and High and then read the two data pins
        GPIO.output(23, False)  #Clock pin Low
        time.sleep(TICK)
        GPIO.output(23, True)   #Clock pin High 
        time.sleep(TICK)        #Wait half clock period and then read bit data        
        readBitLt = GPIO.input(18)    #read a bit from Lt odometer Data pin
        readBitRt = GPIO.input(22)    #read a bit from Rt odometer Data pin

        # shift each bit left to form two 10 bit binary words for angle data
        # and two 5 bit binary words for odometer chip and magnet status 
        #The first bit received is the most significant bit (msb bit 15)
        if i > 5:  #first 10 bits contain angular rotation data (0 to 1024)
            angDataLt =((angDataLt << 1) + readBitLt)   #bits 15 to 6
            angDataRt =((angDataRt << 1) + readBitRt) 
        else:      #last 6 bits contain odometer status data for analysis
            statusLt = ((statusLt << 1) + readBitLt)    #bits 5 to 0
            statusRt = ((statusRt << 1) + readBitLt)

        i -= 1      #decrement bit count index number

    time.sleep(TICK)        #Complete clock cycle
    GPIO.output(24, True)   #Chip Select pin High (back to normal state)
    return (angDataLt, angDataRt, statusLt, statusRt)   #return values to main()

'''
    The read_correct_odo function corrects 
'''
def read_correct_odo():
    (angDataLt,angDataRt, statusLt,statusRt) = read_Odometers()
    return (angDataLt,1024-angDataRt, statusLt,statusRt)

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
    odomDistLt,odomDistRt):
    # Calculate change in odometer angles
    changeLt = angDataLt - prevAngDataLt  #change in dataLt since last reading
    changeRt = angDataRt - prevAngDataRt  #change in dataRt since last reading

    if (angDataLt <341) and (prevAngDataLt >683):  #adjacent rollover sectors
        odomDistLt = odomDistLt + 1024 + changeLt  #positive rollover
    elif (angDataLt >683) and (prevAngDataLt <341):#adjacent rollover sectors
        odomDistLt = odomDistLt - 1024 + changeLt  #negative rollover 
    else:
        odomDistLt = odomDistLt + changeLt         #no rollover 

    if (angDataRt <341) and (prevAngDataRt >683):  #adjacent rollover sectors
        odomDistRt = odomDistRt + 1024 + changeRt  #positive rollover
    elif (angDataRt >683) and (prevAngDataRt <341):#adjacent rollover sectors
        odomDistRt = odomDistRt - 1024 + changeRt  #negative rollover
    else:
        odomDistRt = odomDistRt + changeRt         #no rollover
    
    prevAngDataLt = angDataLt     #load new dataLt into prevDataLt for next loop
    prevAngDataRt = angDataRt     #load new dataRt into prevDataRt for next loop
            
    return(odomDistLt,odomDistRt,prevAngDataLt,prevAngDataRt)

'''
    The longCtrl() function handles the longitudinal chassis movement algorithms
'''
def longCtrl(legMode,fwdMax,fwdMin,fwd,turn,wptDist,wptHdg,\
        chassisDist,decelLong):
    if legMode == "lineFwd":
    # Decel to stop without skid at waypoint 
        dist2Go = wptDist-chassisDist   #distance to go to waypoint
        decelDist = (fwd - 127)*10      #dist from wpt to start deceleration
        if dist2Go < decelDist:         #start deceleration
            fwd = int(fwd - decelLong)  #rate of deceleration
            if fwd < 127 + fwdMin:      #speed to allow a stop without skid
                fwd = 127 + fwdMin   
        if dist2Go <= 0:                #reached waypoint
            fwd = 127                   #stop
    return(int(fwd),int(turn),dist2Go)

'''
    The latCtrl() function handles the lateral chassis movement algorithms
'''
def latCtrl(legMode,turnMax,turnMin,fwd,turn,wptDist,wptHdg,chassisHdg,decelLat):
    if legMode == "lineFwd":
        hdg2Go = wptHdg-chassisHdg       #heading error
        turn = 127 + (hdg2Go * decelLat) #bias turn to maintain desired heading
        
    if legMode == "turnRt":
        hdg2Go = wptHdg-chassisHdg        
    # Decel to required Heading without skid at waypoint  
        if hdg2Go < (turnMax * decelLat):  #Dist from wpt to start deceleration
            turn = 127 + (hdg2Go/decelLat) + turnMin   #Decelerate to slow speed
            if turn < 127 + turnMin:
                turn = 127 + turnMin
#if legMode == "turnLt":
#code TBD
                
        if hdg2Go <= 0:     #reached waypoint
            turn = 127      #Stop
    return(int(fwd),int(turn),hdg2Go)
        
'''
    This is the main function of the program. 
    First it sets up the Serial Interface by calling setSerial.
    It then calls the readOdometers function once to obtain the raw odometer
    angle information for the start position (zero distance). 
    It then initialises the two motor commands fwd & turn
    
    It then starts a continuous loop, within which commands are actioned
    endlessly until the program is stopped.
    The motor code section listens for particular keyboard key presses. 
        'arrow keys' start the calibration movement
        'space' will stop the movement.
        'End' will exit program
    The odometer code section sets up the GPIO pins and within an infinite
    loop, reads from the odometers and handles any rollovers before
    outputting the result.
'''
def main():
    # Commands to be actioned once on start of program
    # initialise data links and local variables 
    ser = setSerial()   #set up the Serial link to Master Motor Drive Board
    pygame.init()       #call Function to Initialise Pygame
    loop1 = True        #boolean variable to allow exit from main loop
    fwd = 127		#motor command no forward movement
    turn = 127		#motor command no Turn Rate
    odomDistLt = 0      #start condition for distance travelled by left wheel
    odomDistRt = 0      #start condition for distance travelled by right wheel
    prevAngDataLt= 0    #start condition for Lt odom angle data on previous read
    prevAngDataRt= 0    #start condition for Rt odom angle data on previous read
    wheelDiaLt = 150    #diameter of left drive wheel mm
    wheelDiaRt = 150    #diameter of right drive wheel mm
    wheelTrack = 237    #distance between track of left & right drive wheels mm
    status = "stopped"     #Program status

    legMode = "stop" #default mode
    wptDist = 2000   #2000 distance required at waypoint - change for testing
    wptTrack = 0     #0 line dirn from start to waypoint - change for testing 
    wptHdg = 0       #0 turn heading required at waypoint - change for testing

    chassisDist = 0     #total distance from start
    dist2Go = wptDist   #Distance to go to waypoint
    fwdMax = 60         #max fwd speed (126)
    fwdMin = 15         #minimum fwd speed to allow instant stop without skid
    decelLong = 0.2     #rate of longitudinal deceleration= 60*decelLong per sec
    
    chassisHdg = 0      #Start Heading of Chassis
    hdg2Go = wptHdg     #Heading  to go to waypoint
    turnMax = 60        #max turn rate
    turnMin = 15        #minimum turn rate to allow instant stop without skid
    decelLat = 1.2      #rate of lateral deceleration

    
    
    # Read odometers once to get wheel angle offsets at robot start position
    (angDataLt,angDataRt, statusLt,statusRt) = read_correct_odo() #function call
                                                #to obtain raw data and status
    prevAngDataLt = angDataLt   #start condition for prevAngDataLt
    prevAngDataRt = angDataRt   #start condition for prevAngDataRt

    '''
    Commands to be actioned endlessly in a loop until program is stopped    
    '''
    while loop1 == True:    #loops until loop1 is declared False

        # Motor Code
        for event in pygame.event.get():
            # if a designated keyboard key is pressed do following actions:
            if event.type == pygame.KEYDOWN:    #detect a key press
                if event.key == pygame.K_UP:	#Up Arrow= Start Forward
                    legMode = "lineFwd"         #straight line to next wpt
                    fwd  = 147                  #forward
                    turn = 127                  #no turn movement
                    status = "running"          #Setting screen output to "running"
                if event.key == pygame.K_DOWN:  #Down Arrow= Start Backwards
                    legMode = "lineBack"        #straight line to next wpt
                    fwd  = 97                   #reverse
                    turn = 127                  #no turn movement
                    status = "running"          #Setting screen output to "running"
                if event.key == pygame.K_RIGHT: #Right arrow= Start Right turn
                    legMode = "turnRt"          #turn right on spot
                    fwd  = 127                  #no forward movement 
                    turn = 157                  #turn right
                    status = "running"          #Setting screen output to "running"
                if event.key == pygame.K_LEFT:  #Left arrow= Start Left turn
                    legMode = "turnLt"          #turn left on spot
                    fwd  = 127                  #no forward movement 
                    turn = 97                   #turn left
                    status = "running"          #Setting screen output to "running"
                if event.key == pygame.K_SPACE: #STOP MOVEMENT  (Space Bar)
                    fwd  = 127                  #no forward movement
                    turn = 127                  #no turn movement
                    status = "stopped"                 #Setting screen output to "stop"
                if event.key == pygame.K_END:   #END PROGRAM  (End Key)
                    fwd  = 127                  #no forward movement   
                    turn = 127                  #no turn movement
                    status = "end"             #Setting screen output to "end"
                    for n in range(3):          #Send stop command 3 times
                        ser.write(chr(fwd))     #send fwd command to motor
                        ser.write(chr(turn))    #send turn command to motor
                        time.sleep(0.016)       #Wait for 16 msec
                    loop1 = False               #Set loop1 to False to exit current loop

        # Call the longitudinal and lateral control functions to test movement
        if legMode == "lineFwd":
            (fwd,turn,dist2Go) = longCtrl(legMode,fwdMax,fwdMin,fwd,turn,\
            wptDist,wptTrack,chassisDist,decelLong)
            
            (fwd,turn,Hdg2Go) = latCtrl(legMode,turnMax,turnMin,fwd,turn,\
            wptDist,wptHdg,chassisHdg,decelLat)            

        if legMode == "turnRt":
            (fwd,turn,hdg2Go) = latCtrl(legMode,turnMax,turnMin,fwd,turn,\
                wptDist,wptHdg,chassisHdg,decelLat)            

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

#       telemetry = getTelemetry(ser)  #Receive Telemetry from chariot serial


        # Odometer Code
        # read odometers for raw angle data and status
        (angDataLt,angDataRt,statusLt,statusRt) = read_correct_odo()

        # calculate odometer distances by actioning rollovers
        (odomDistLt,odomDistRt,prevAngDataLt,prevAngDataRt)= handle_rollovers\
            (angDataLt,angDataRt,prevAngDataLt,prevAngDataRt,\
             odomDistLt,odomDistRt)
        # correct for right odometer and motor operating in reverse
#        correctedOdomDistLt1 = odomDistLt 
        odomDistRtReversed = odomDistRt    #value for display
        # correct for both odometers initial position at start
#        correctedOdomDistLt2 = correctedOdomDistLt1 - odomAngOffsetLt
#        correctedOdomDistRt2 = correctedOdomDistRt1 + odomAngOffsetRt

        # Calculate distance moved by each wheel in mm 
        # wheelDist = (wheelDia * pi) * odomDist / 1024
        wheelDistLt = wheelDiaLt * math.pi * odomDistLt /1024
        wheelDistRt = wheelDiaRt * math.pi * -odomDistRt /1024
        wheelDistLt = round(wheelDistLt,1)  #round number to one decimal place
        wheelDistRt = round(wheelDistRt,1)  #round number to one decimal place
        chassisDist = (wheelDistLt + wheelDistRt) /2 #total distance from start
        
        #Calculate Heading 
#        WHEELCIRCUMFERENCE = 500
#        TRACKWIDTH = 237    
#        heading = (360*(((odomDistLt-odomDistRtReversed)/2)*\
#                       ((WHEELCIRCUMFERENCE)/(1024*math.pi*TRACKWIDTH))))%360
#        headingRounded = round(heading, 1) # rounded for ease of use

        # Calculate Heading in degrees with respect to North (y axis)
        # using difference in distance travelled by each wheel
        differenceInDist = (wheelDistLt - wheelDistRt)
        # chassis heading in Radians = difference in distance / wheeltrack
        chassisHdg = math.degrees(differenceInDist/wheelTrack) #change to deg      

#        dist2Go = wptDist-chassisDist
        hdg2Go = wptHdg-chassisHdg

        
        # Pygame Display Code
        # update the pygame data display
        screen.fill(black)  #Blank out previous digits in display window

        # Display motor output text and values to the screen
        screen.blit(font.render("Fwd Speed   Turn Rate",True,white),(100,5))
        screen.blit(font.render(str(fwd),True,white),(125,20))
        screen.blit(font.render(str(turn),True,white),(188,20))

        # Display odometer output text and values to the screen
        screen.blit(font.render("Left Odom   Right Odom    Heading",\
            True,white),(100,40))
        screen.blit(font.render("Rollover",True,white),(15,55))
        screen.blit(font.render("Continuous",True,white),(15,70))
        screen.blit(font.render("Rt Reversed",True,white),(15,85))
        screen.blit(font.render("Wheel Dist",True,white),(15,100))        
        screen.blit(font.render("ChassisDist",True,white),(15,115))        
        screen.blit(font.render("Dist2Go/Hdg2Go",True,white),(15,130))        

        # Display odometer test values to test for correct odom readings
        screen.blit(font.render(str(angDataLt),True,white),(123,55))
        screen.blit(font.render(str(angDataRt),True,white),(193,55))
        screen.blit(font.render(str(odomDistLt),True,white),(123,70))
        screen.blit(font.render(str(odomDistRt),True,white),(193,70))
        screen.blit(font.render(str(odomDistLt),True,white),(123,85))
        screen.blit(font.render(str(odomDistRtReversed),True,white),(193,85))

        # Scalings applied to give correct wheel distances and heading        
        screen.blit(font.render(str(int(wheelDistLt)),True,white),(123,100))
        screen.blit(font.render(str(int(wheelDistRt)),True,white),(193,100))
#        screen.blit(font.render(str(headingRounded),True,white),(255,90))
        screen.blit(font.render(str(round(chassisHdg,1)),\
            True,white),(255,100))

#        screen.blit(font.render(str(int(wptDist)),True,white),(123,115))
        screen.blit(font.render(str(int(chassisDist)),True,white),(158,115))
        screen.blit(font.render(str(int(dist2Go)),True,white),(158,130))
        screen.blit(font.render(str(int(hdg2Go)),True,white),(255,130))
        screen.blit(font.render(status,True,white),(150,145))
        #Display telemetry values  -Not Used
#        screen.blit(font.render(str(telemetry[0]),True,white),(50,25))
#        screen.blit(font.render(str(telemetry[1]),True,white),(100,25))
#        screen.blit(font.render(str(telemetry[2]),True,white),(50,45))
#        screen.blit(font.render(str(telemetry[3]),True,white),(100,45))

        pygame.display.update() # update the display on each loop

        time.sleep(0.016)       #Wait for 16 msec and repeat loop

# Program start            
main()              # call main() function containing a continuous loop

# Program exit when END key pressed.
pygame.quit()       # uninstall all pygame modules
sys.exit()          # exit from Python

