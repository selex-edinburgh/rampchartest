# desktop/testPrograms/servoTest.pi   23/10/16
# This program tests the Stepper, motor, Micro switch and Infra-red Sensor
#  with provision to test the Servo Motor and Ultra-Sonic Sensor
# It also allows the motors and distance sensors to be calibrated
# Different sensors and functions are selected by means of 'Control Codes'
# Insert the control codes required for WRITE and READ and then run programme

#import libraries
import sys
import time
import math
import pygame
import smbus

# set up I2C serial link
bus = smbus.SMBus(1)    #There are two I2C SMbus available on the R-Pi
address = 4             #I2C Sensors Board Address
numBytes = 4            #I2C Number of Bytes to be Sent
txBytes = [0,0]         #Define txBytes as 2 Bytes
rxBytes = [0,0,0,0]     #Define rxBytes as 4 Bytes
                        #rxBytes=(rxByte[0], rxByte[1], rxByte[2], rxByte[3])

# Initialise display parameters of Pygame display window
pygame.init()
black = (0,0,0)         # Preset colours to use
white = (255,255,255)
red = (255,0,0)
# Set display parameters of the window
displayWidth = 190      # Size of Window  190 x 100
displayHeight = 100
screen = pygame.display.set_mode((displayWidth,displayHeight)) 
font = pygame.font.SysFont('Avenir Next', 20)	# Preset font to use
pygame.display.set_caption("Sensors Test")      # Window Title

'''
    This function displays the angle of each sensor and its associated
    distance reading. When selected it will display the distance of the
    nearest obstacle detected on each scan and the associated angle.
'''
def screen_display(angle, distance, caption):		
    if caption== "I-R":     #blank digits before writing new values
        pygame.draw.rect(screen,black,[120,45,60,35]) #rect[x,y,width,height]
        screen.blit(font.render("I-R Angle: ",True,white),(15,45))
        screen.blit(font.render("I-R Range: ",True,white),(15,65))
        screen.blit(font.render(str(angle),True,white),(125,45))
        screen.blit(font.render(str(distance),True,white),(125,65))
#For Ultra-Sonic Sensor insert code here to display "U-S Angle" and "U-S Range        
    pygame.display.update()		# update the display on each loop

'''
    The WRITE_I2C function sends a message from the R-Pi master to the
    Sensors Board slave
    The message contains the Address with READ/WRITE bit(0) followed by 4 Bytes.
    These 4 Bytes are:   control, numBytes, txByte[0], txByte[1]
     (For a WRITE command the parameter numBytes is inserted by the R-Pi)
'''
def write_I2C(address, control, txBytes):     #txBytes= txByte[0], txByte[1]
    try:
        bus.write_block_data(address, control, txBytes)		
    except Exception as err:
        print err       #error does not work as it's a write command
        print "Failed to write to the bus"

'''
    The READ_I2C function sends a message from the R-Pi master to the
    Sensors Board Slave requesting the Sensors Board to send it data.
    The R-Pi sends address with READ/WRITE bit(0) followed by control Byte.
    The Sensors Board prepares the data and waits.
    The R-Pi resends address with READ/WRITE bit(1) and reads back 4 Bytes.
    These 4 Bytes are rxByte[0], rxByte[1], rxByte[2], rxByte[3]
'''
def read_I2C(address,control,numBytes):
    try:
        rxBytes = bus.read_i2c_block_data(address, control, numBytes)
    except Exception as err:
        print err
        print "Failed to write to the bus"        
    return (rxBytes[0], rxBytes[1], rxBytes[2], rxBytes[3])

'''
    This is the main function of the program. 
    You select the Control Code for the sensor and function required
     and this is sent as a WRITE command to the Sensors Board       
    The programme then starts a continuous loop, within which READ commands
    are actioned endlessly until the program is stopped.
    The sensor data is displayed in the window
    'End' will exit program
'''

def main():
# Choose Control Code to WRITE Commands to Stepper Motor
#        control= 37    Stepper Reinitialise (32+4+Speed 1)
#        control= 41-43 Stepper Go To Fixed Angle (32+8+Speed 1-3)
#        control= 45-47 Stepper Scan Between 2 Angles (32+12+Speed 1-3)
#        control= 73-75 Servo Go To Fixed Angle (64+8+Speed 1-3)
#        control= 77-79 Servo Scan Between 2 Angles (64+12+Speed 1-3)

    control = 37    #change this number

    steprFixedAngle = 0         # +/- degrees (0.1 deg accuracy available)
    steprScanAngleLt = 45       # +/- degrees (to 1 deg accuracy)
    steprScanAngleRt = -45      # +/- degrees (to 1 deg accuracy)
    servoFixedAngle = 0         # +/- degrees (0.1 deg accuracy available)
    servoScanAngleLt =  45      # +/- degrees (to 1 deg accuracy)
    servoScanAngleRt = -45      # +/- degrees (to 1 deg accuracy)
        
# Calibration values        
    steprScaling= 11.32  #stepper movement mechanical scaling in bits/degree
    steprDatum = 3200    #offset of Stepper Motor micro switch datum in steps
    servoScaling = 9.26  #servo movement scaling in bits/degree
    servoOffset = 8.1    #servo shaft offset from chassis centreline in degrees
    IRslope = 1          #slope of calibration straight line "m"
    IRoffset = 0         #offset of calibration straight line "c"

# Convert control codes into bytes for transmission
# Angles are converted into steps (11.32/deg) and the centre offset of 2048 steps added         

    if control ==37:       #Stepper Motor Reinitialise to Centre Datum
        print "Initialising Stepper Motor at Speed", control-36
        #Convert stepperDatum to Hi & Lo Bytes steps for transmission
        txBytes[0] = int(steprDatum/256)                #stepperDatum Hi Byte
        txBytes[1] = int(steprDatum - txBytes[0]*256)   #stepperDatum Lo Byte
#       print(txBytes[0], txBytes[1])                   #print to screen
        write_I2C(address, control, txBytes)            #sent to sensors

    if control >=41 and control <=43:       #Stepper Motor Go To Fixed Angle
        print "Stepper Motor To Fixed Angle at Speed",(control-40)
        #Convert steprFixedAngle to Hi & Lo Bytes steps for transmission
        steprFixedAngle = steprFixedAngle*steprScaling + 2048  #scale for steps/deg & add centre offset
        txBytes[0] = int(steprFixedAngle/256)              #steprFixedAngle Hi Byte
        txBytes[1] = int(steprFixedAngle - txBytes[0]*256) #steprFixedAngle Lo Byte
#       print(txBytes[0],txBytes[1],int(steprFixedAngle)) #print to screen
        write_I2C(address, control, txBytes)          #sent to sensors

    if control >=45 and control <=47:  #Stepper Motor Scan Between Two Angles
        print "Stepper Motor Scanning at Speed",(control-44)
        #Convert steprScanAngleLt & Rt to Byte steps for transmission
        steprScanAngleLt = steprScanAngleLt*steprScaling + 2048  #scale for steps/deg & add offset
        steprScanAngleRt = steprScanAngleRt*steprScaling + 2048  #scale for steps/deg & add offset
        txBytes[0] = int(steprScanAngleLt/16)        #convert to Byte (0 to 255)
        txBytes[1] = int(steprScanAngleRt/16)        #convert to Byte (0 to 255)
#       print(txBytes[0],txBytes[1])            #print to screen
        write_I2C(address, control, txBytes)    #sent to sensors   

#Insert code here for Servo Motor Go To Fixed Angle and Scan Between Two Angles

    time.sleep(0.015)           #Must Wait at least 15msec 
    
# Choose Control Code to READ Data from Stepper Motor and I-R Sensor
#        control= 112 Send Back Current I-R Range & Stepper Angle (96+16)
#        control= 116 Send Back Nearest I-R Range & Stepper Angle (96+20)
#        control= 144 Send Back Current U-S Range & Servo Angle (128+16)
#        control= 148 Send Back Nearest U-S Range & Servo Angle (128+20)
    control = 112  #Change this number to display different parameters

# Loop to READ the Motor (angle) and Sensor Range (distance)
    while True:        # this is the READ data and display loop

#Insert code here to change between I-R and U-S each read loop
#to display both parameters       

        for event in pygame.event.get():
            if event.type == pygame.QUIT:       #end program
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:	#detect a key press
                if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:		# safe quit on "q" press or "ESC" press
                    pygame.quit()
                    sys.exit()
                if event.key == pygame.K_END:   #END PROGRAM  (End Key)
                    pygame.quit()
                    sys.exit()
                
        rxBytes = read_I2C(address,control,numBytes)  #Read 4 Bytes                
        angle = rxBytes[0]*256 + rxBytes[1]     #Add High and Low Bytes
        distance = rxBytes[2]*256 + rxBytes[3]  #Add High and Low Bytes

# Select and Scale Sensor Readings for rangeIR and AngleIR
        if control==112:   #Get Current I-R Range & Stepper Angle
            angleIR = (angle-2048)/11.32 #remove offset & scale for steps/deg
            rangeIR = (distance*10)      #Scale IR sensor Range                        
#           if rangeIR > 300:            #Limit rangeIR
#               rangeIR = 300
            angleIR = '{0:.1f}'.format(angleIR) #format to two decimal places
            rangeIR = '{0:.0f}'.format(rangeIR) #format to two decimal places
            screen_display(angleIR, rangeIR,"I-R")#Sub Routine to disply values
                                
        if control==116:   #Get Nearest I-R Range & Stepper Angle
            angleIR = (angle-2048)/11.32 #remove offset & scale for steps/deg
            rangeIR = distance*10        #Scale IR sensor Range
#           if rangeIR > 300:            #Limit rangeIR
#               rangeIR = 300
            angleIR = '{0:.1f}'.format(angleIR) #format to two decimal places
            rangeIR = '{0:.0f}'.format(rangeIR) #format to two decimal places
            screen_display(angleIR, rangeIR,"I-R")#Sub Routine to display values

# Insert code here to Select and Scale Sensor Readings for rangeUS and AngleUS


        time.sleep(0.008)                 #Wait at least 0.002
main()

#---------------------------------------------------------

 
