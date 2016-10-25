# desktop/testPrograms/fwd-turnCal.pi   24/10/16
# This program combines MotorTest and OdometerTest to allow calibration of the
# Wheel/odometer combination in a straight line and in a turn on-the-spot.
# Arrow keys = start movement forward
# Space = Stop movement
# End = Exit programme
# reverse and turn left not implemented yet

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
displayHeight = 145
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
    The getTelemetry function listens for a reply from the Master Motor Drive
    Board. It is currently not used.
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
    GPIO.output(24, False)  #Chip Select Low(both odom triggered to output data)
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
        # and two 6 bit binary words for odometer chip and magnet status 
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

    # Repeat for the other odometer 
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
    The speed profile has 5 phases: Accelerate, Cruise, Decelerate, Creep, and
    then Stop at the next waypoint without skidding.(see manual for explanation)
'''
def longCtrl(legMode,fwdSpd,fwdMin,wptDist,chassisDist,legDistLt,legDistRt,\
        accelLong,decelLong):       #chassisDist not used this programme
    if legMode == "lineFwd":
    #Accelerate, Cruise, Decelerate, Creep and Stop at waypoint
        chassisDist = (legDistLt+legDistRt)/2  #total distance from start of leg
        dist2Go = wptDist-chassisDist   #distance to go to waypoint
        accelSpd = (chassisDist*accelLong)+fwdMin  #increase speed as dist from WP
        decelSpd = (dist2Go * decelLong)+(fwdMin/2) #reduce speed as dist2Go reduces

        fwd = 0
        longPhase = "stop"
        if accelSpd < fwdSpd:        
            fwd = accelSpd              #accel phase
            longPhase = "accel"         
        if accelSpd >= fwdSpd:        
            fwd = fwdSpd                #cruise phase
            longPhase = "cruise"
        if (decelSpd < accelSpd) and (decelSpd < fwdSpd): #may be no cruise phase
            fwd = decelSpd              #decel phase
            longPhase = "decel"
        if decelSpd < fwdMin:
            fwd = fwdMin                #creep phase
            longPhase = "creep"
        if dist2Go <= 0:                #reached waypoint
            fwd = 0                     #Stop phase
            longPhase = "stop"
            
        fwd = 127+ fwd                  #correct for zero datum
    return(int(fwd),dist2Go,chassisDist,longPhase)

'''
    The latCtrl() function handles the lateral chassis movement algorithms
    The turn rate profile is Accelerate, Cruise, Decelerate, Creep, and
    stop the turn at the next waypoint heading without skidding.
    The chassis heading in degrees with respect to North (y axis)is calculated
    using the difference in distance travelled by each wheel:
      chassis heading in Radians = difference in distance / wheeltrack
      chassis Hdg in degrees = difference in distance *0.24 deg   [180/pi/237]
    Due to the offset position of the two drive wheels and odometers the wheels
    skid laterally during a turn-on-the-spot and an additional correction
    factor of 0.58 is required. Finally a small correction for friction is
    used to calibrate the turn.  (The maths is covered in Appendix ?)
'''
def latCtrl(legMode,turnSpd,turnMin,wptHdg,chassisHdg,legDistLt,legDistRt,\
        accelLat,decelLat):         #chassisHdg not used this program
    #maintain heading during straight line   
    if legMode == "lineFwd":        
        #calculate heading change from start of leg & convert radians to deg
        # = difference in distance *0.24 deg      [180/pi/wheeltrack(237)]
        legChassisHdg = (legDistLt-legDistRt)*0.24  #no wheel offset factor reqd
        chassisHdg = legChassisHdg      #+wptHdg? not reqd this programe
        hdg2Go = wptHdg - chassisHdg    #heading error
        turn = hdg2Go * decelLat        #required turn rate to regain track angle
        latPhase = "decel"                 #decelerating correction to hdg error
        turn = 127 + turn               #correct for zero datum
        
    #turn on the spot to new waypoint heading   
    if legMode == "turnRt":
        friction = 1.00                 #wheel lateral skid friction (was 1.03)
        #calculate total change from start of leg/turn & convert radians to deg
        # = difference in distance *0.24 deg      [180/pi/wheeltrack(237)]
        legChassisHdg = (legDistLt-legDistRt)*0.24  
        legChassisHdg = legChassisHdg *0.58 *friction #correct for offset wheel posn   
        chassisHdg = legChassisHdg      #+wptHdg? not reqd this programe
        #Calc Heading in degrees with respect to North (y axis) not reqd this programe

        #turn to new waypoint heading and stop without skidding
        hdg2Go = wptHdg - chassisHdg    #hdg2Go to reach waypoint heading
    #Accelerate, Cruise, Decelerate, Creep and Stop at waypoint without skidding    
        accelRate = (chassisHdg*accelLat)+turnMin  #increase speed as dist from WP
        decelRate = (hdg2Go*decelLat)+(turnMin/2) #reduce rate as dist2Go reduces

        turn = 0
        latPhase = "stop"
        if accelRate < turnSpd:        
            turn = accelRate            #accel phase
            latPhase = "accel"         
        if accelRate >= turnSpd:        
            turn = turnSpd              #cruise phase
            latPhase = "cruise"
        if (decelRate < accelRate) and (decelRate < turnSpd): #may be no cruise phase
            turn = decelRate            #decel phase
            latPhase = "decel"
        if decelRate < turnMin:
            turn = turnMin              #creep phase
            latPhase = "creep"
        if hdg2Go <= 0:                 #reached waypoint heading
            turn = 0                     #Stop phase
            latPhase = "stop"
            
        turn = 127+ turn                  #correct for zero datum
                
    #if legMode == "turnLt":
    #code TBD

    return(turn,hdg2Go,chassisHdg,latPhase)
        
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
    fwdMax = 100        #max fwd speed 
    turnMax = 100       #max turn rate 
    odomDistLt = 0      #start condition for distance travelled by left wheel
    odomDistRt = 0      #start condition for distance travelled by right wheel
    prevAngDataLt= 0    #start condition for Lt odom angle data on previous read
    prevAngDataRt= 0    #start condition for Rt odom angle data on previous read
    legMode = "stop"    #default mode
    longPhase = "stop"  #speed profile phase
    latPhase = "stop"   #turn profile phase
    chassisDist = 0     #distance from start of current leg
    chassisHdg = 0      #heading of chassis at start of current leg

    #Longitudinal and Lateral parameters that control the test manoeuvres.
    previousWPnum = 0   #previous waypoint number
    WPnum = 1           #next waypoint number
    lateralCorrect= True  #turn on/off lateral correction for test 1 (lineFwd)
    reqdWptDist = 2000  #2000 distance required at waypoint - change for testing
    reqdWptHdg = 180    #180 heading required at waypoint - change for testing 
    fwdSpd = 60         #standard forward speed (normal range 30 to 60)
    turnSpd = 60        #standard turn-rate (normal range 40 to 70)

    #Chassis parameters
    wheelDiaLt = 150    #diameter of left drive wheel mm
    wheelDiaRt = 150    #diameter of right drive wheel mm
    wheelTrack = 237    #distance between track of left & right drive wheels mm

    #Longitudinal Calibration parameters
    motorBias = 4       #difference in power between the left and right motors
    fwdMin = 15         #10 minimum fwd speed to allow instant stop without skid
    turnMin = 25        #10 minimum turn rate to allow instant stop without skid
    accelLong = 0.2     #0.2 rate of longitudinal acceleration
    decelLong = 0.1     #0.1 rate of longitudinal deceleration
    accelLat = 1        #1 rate of lateral acceleration
    decelLat = 1.2      #1.2 rate of lateral deceleration
    #frictionLt = 1     #friction calibration factor for Left drive wheel
    #frictionRt = 1     #friction calibration factor for Right drive wheel

    dist2Go = reqdWptDist   #distance to go to waypoint
    hdg2Go = reqdWptHdg          #heading  to go to waypoint   
    # Read odometers once to get wheel angle offsets at robot start position
    (angDataLt,angDataRt, statusLt,statusRt) = read_Odometers() #function call
                                                #to obtain raw data and status
    prevAngDataLt = angDataLt   #start condition for prevAngDataLt
    prevAngDataRt = angDataRt   #start condition for prevAngDataRt

    '''
    Commands to be actioned endlessly in a loop until program is stopped
    1.  Read Odometers
    2.  Calculate Logitudinal and Lateral control corrections
    3.  Action Key presses
    4.  Send Output to Motors
    5.  Display Parameters Code
    '''
    
    while loop1 == True:    #loop until loop1 is declared False

        # Odometer Code
        # read odometers for raw angle data and status
        (angDataLt,angDataRt,statusLt,statusRt) = read_Odometers()

        # calculate odometer distances in bits by actioning rollovers
        (odomDistLt,odomDistRt,prevAngDataLt,prevAngDataRt)= handle_rollovers\
            (angDataLt,angDataRt,prevAngDataLt,prevAngDataRt,\
             odomDistLt,odomDistRt)

        # correct for right odometer and motor operating in reverse
        odomDistRtReversed = -odomDistRt    #value for display
        
        # Convert odometer distances in bits into distance moved by wheel in mm 
        # wheelDist = (wheelDia * pi) * odomDist / 1024     
        wheelDistLt = wheelDiaLt * math.pi * odomDistLt/1024  
        wheelDistRt = wheelDiaRt * math.pi * -odomDistRt/1024 
        wheelDistLt = round(wheelDistLt,1) #round number to one decimal place
        wheelDistRt = round(wheelDistRt,1) #round number to one decimal place

        # save wheel distances at start of each leg (Waypoiont)
        if WPnum > previousWPnum:           #waypoint has changed
            startDistLt = wheelDistLt       #store Left wheel distance
            startDistRt = wheelDistRt       #store Right wheel distance
            previousWPnum = WPnum       #change prevWPnum after storing dist
        legDistLt = wheelDistLt-startDistLt     #Calc leg Distance Left
        legDistRt = wheelDistRt-startDistRt     #Calc leg Distance Right

        # Longitudinal and Lateral Control Code
        # Call the longitudinal and lateral control functions to test movement
        if legMode == "lineFwd":
            (fwd,dist2Go,chassisDist,longPhase)= longCtrl(legMode,fwdSpd,fwdMin,\
                wptDist,chassisDist,legDistLt,legDistRt,decelLong,accelLong)

            if lateralCorrect == True:  #correct Hdg to maintain straight track        
                (turn,hdg2Go,chassisHdg,latPhase)= latCtrl(legMode,turnSpd,turnMin,\
                    wptHdg,chassisHdg,legDistLt,legDistRt,accelLat,decelLat)            

        if legMode == "turnRt":
            (turn,hdg2Go,chassisHdg,latPhase)= latCtrl(legMode,turnSpd,turnMin,\
                wptHdg,chassisHdg,legDistLt,legDistRt,accelLat,decelLat)            


        #Key Press Code
        for event in pygame.event.get():
            # if a designated keyboard key is pressed do following actions:
            if event.type == pygame.KEYDOWN:    #detect a key press
                if event.key == pygame.K_UP:	#START FORWARDS (Up Arrow)
                    legMode = "lineFwd"         #straight line to next wpt
                    wptDist = reqdWptDist       #distance required at waypoint
                    wptHdg = 0                  #heading required at waypoint           
                    fwd  = 127 + fwdSpd         #forward speed
                    turn = 127                  #no turn movement
                if event.key == pygame.K_DOWN:  #START BACKWARDS (Down Arrow)
                    legMode = "lineBack"        #straight line to next wpt
                    fwd  = 127 - fwdSpd         #reverse speed
                    turn = 127                  #no turn movement
                if event.key == pygame.K_RIGHT: #START RIGHT TURN (Right arrow)
                    legMode = "turnRt"          #turn right on spot
                    wptDist = 0                 #distance required at waypoint
                    wptHdg = reqdWptHdg         #heading required at waypoint           
                    fwd  = 127                  #no forward movement 
                    turn = 127 + turnSpd        #turn right 
                if event.key == pygame.K_LEFT:  #SART LEFT TURN (Left arrow) 
                    legMode = "turnLt"          #turn left on spot
                    wptDist = 0                 #distance required at waypoint
                    wptHdg = -reqdWptHdg        #heading required at waypoint           
                    fwd  = 127                  #no forward movement 
                    turn = 127 - turnSpd        #turn left 
                if event.key == pygame.K_SPACE: #STOP MOVEMENT  (Space Bar)
                    legMode = "stop"            #turn left on spot
                    fwd  = 127                  #no forward movement
                    turn = 127                  #no turn movement
                if event.key == pygame.K_END:   #END PROGRAM  (End Key)
                    fwd  = 127                  #no forward movement   
                    turn = 127                  #no turn movement
                    for n in range(10):          #Send stop command 5 times
                        ser.write(chr(fwd))     #send fwd command to motor
                        ser.write(chr(turn))    #send turn command to motor
                        time.sleep(0.016)       #Wait for 16 msec
                    loop1 = False               #Set loop1 to False to exit current loop

        # Output to Motors Code FOR SAFETY REASONS DO NOT CHANGE THIS CODE BLOCK
        # limit values & write the fwd and turn to the motors via serial link
        if fwdMax > 126:
            fwdMax = 126    #valid number wrt 127 to fit within a single byte
            
        #check that fwd is always within byte range 255 and 0 to avoid overflow
        if (fwd < 255) and (fwd > 0):   #valid range
            fwd = int(fwd)              #change float to integer type
            ser.write(chr(fwd))         #send fwd command to motor board
        else:
            fwd = 127                   #no forward movement
            ser.write(chr(fwd))         #send fwd command to motor board

        if turnMax > 126:
            turnMax = 126    #valid number wrt 127 to fit within a single byte
            
        #check that turn is always within byte range 255 and 0 to avoid overflow
        if (turn < 255-motorBias) and (turn > 0-motorBias): #valid range
            turn = int(turn)            #change float to integer type
            ser.write(chr(turn+motorBias))  #send turn command to motor board
        else:
            turn = 127                  #no turn movement
            ser.write(chr(turn))        #send turn command to motor board

        # Display Parameters Code
        # update the pygame data display
        screen.fill(black)  #Blank out previous digits in display window

        # Display motor output text and values to the screen
        screen.blit(font.render("Fwd Speed   Turn Rate  Motor Bias",\
            True,white),(100,5))
        screen.blit(font.render(str(fwd),True,white),(125,20))
        screen.blit(font.render(str(turn),True,white),(188,20))
        screen.blit(font.render(str(motorBias),True,white),(255,20))

        # Display odometer output text and values to the screen
        screen.blit(font.render("Left Odom   Right Odom    Heading",\
            True,white),(100,40))
        screen.blit(font.render("Odom Dist",True,white),(12,55))
        screen.blit(font.render("Wheel Dist mm",True,white),(12,70))        
        screen.blit(font.render("ChassisDist & Hdg",True,white),(12,85))        
        screen.blit(font.render("Dist2Go & Hdg2Go",True,white),(12,100))        

        # Display odometer test values to test for correct odom readings
        screen.blit(font.render(str(odomDistLt),True,white),(123,55))
        screen.blit(font.render(str(odomDistRtReversed),True,white),(193,55))

        # Scalings applied to give correct wheel distances and heading        
        screen.blit(font.render(str(int(wheelDistLt)),True,white),(123,70))
        screen.blit(font.render(str(int(wheelDistRt)),True,white),(193,70))
        screen.blit(font.render(str(int(chassisDist)),True,white),(158,85))
        screen.blit(font.render(str(round(chassisHdg,1)),\
            True,white),(255,85))
        screen.blit(font.render(str(int(dist2Go)),True,white),(158,100))
        screen.blit(font.render(str(int(hdg2Go)),True,white),(255,100))

        screen.blit(font.render("FwdPhase:",True,white),(12,115))
        if longPhase =="accel":
            screen.blit(font.render("accel",True,white),(85,115))
        if longPhase =="cruise":
            screen.blit(font.render("cruise",True,white),(130,115))
        if longPhase =="decel":
            screen.blit(font.render("decel",True,white),(180,115))
        if longPhase =="creep":
            screen.blit(font.render("creep",True,white),(225,115))
        if longPhase =="stop":
            screen.blit(font.render("stop",True,white),(270,115))
            
        screen.blit(font.render("TurnPhase:",True,white),(12,130))
        if latPhase =="accel":
            screen.blit(font.render("accel",True,white),(85,130))
        if latPhase =="cruise":
            screen.blit(font.render("cruise",True,white),(130,130))
        if latPhase =="decel":
            screen.blit(font.render("decel",True,white),(180,130))
        if latPhase =="creep":
            screen.blit(font.render("creep",True,white),(225,130))
        if latPhase =="stop":
            screen.blit(font.render("stop",True,white),(270,130))

        pygame.display.update() # update the display on each loop

        time.sleep(0.016)       #Wait for 16 msec and repeat loop
            
# Program start            
main()              # call main() function containing a continuous loop

# Program exit when END key pressed.
pygame.quit()       # uninstall all pygame modules
sys.exit()          # exit from Python

