#teststepper.py
import math
import time
import sys
import pygame
import smbus

# set up I2C serial link
bus = smbus.SMBus(1)     #There are two I2C SMbus available on the R-Pi
address = 4              #I2C Sensors Board Address
numBytes = 4             #I2C Number of Bytes to be Sent
txBytes = [0,0]          #Define txBytes as 2 Bytes
rxBytes = [0,0,0,0]      #Define rxBytes as 4 Bytes
#rxBytes = (rxByte[0], rxByte[1], rxByte[2], rxByte[3])

pygame.init()
# Preset colours to use
black = (0,0,0)
white = (255,255,255)
red = (255,0,0)
# Set display parameters of the window
displayWidth = 190
displayHeight = 50
screen = pygame.display.set_mode((displayWidth,displayHeight)) 
font = pygame.font.SysFont('Avenir Next', 20)	# Preset font to use
pygame.display.set_caption("Stepper Test")

def screen_display(angle, distance):		
        screen.fill(black)	# set the screen background to black (default)
        screen.blit(font.render("Sensor Angle: ",True,white),(15,5))
        screen.blit(font.render(str(angle),True,white),(125,5))
        screen.blit(font.render("Sensor Range: ",True,white),(15,25))
        screen.blit(font.render(str(distance),True,white),(125,25))
        pygame.display.update()		# update the display on each loop


# I2C WRITE to Sensors Board
# The R-Pi sends address with WRITE bit(0) followed by 4 Bytes. #WRITE Command
# The 4 Bytes are:   control, numBytes, txByte[0[, txByte[1]
# For a WRITE, numBytes is inserted by the R-Pi

def write_I2C(address, control, txBytes):       
	try:
		bus.write_block_data(address, control, txBytes) #txBytes= txByte[0], txByte[1]
	except Exception as err:
		print err
		print "Failed to write to the bus"
#error does not work as write command

# I2C READ from Sensors Board
# The R-Pi sends address with READ/WRITE bit(0) followed by control Byte.       #WRITE Command
# The R-Pi then resends address with READ/WRITE bit(1) and reads back 4 Bytes.  #READ Command
#    The 4 Bytes are rxByte[0], rxByte[1], rxByte[2], rxByte[3]

def read_I2C(address,control,numBytes):
        try:
                rxBytes = bus.read_i2c_block_data(address, control, numBytes)
        except Exception as err:
                print err
                print "Failed to write to the bus"
        
 #       irAngle = rxBytes[0]*256 + rxBytes[1]   #Add High and Low Bytes
 #       irAngle = irAngle/11.32 -135            #Scale Angle
 #       irRange = rxBytes[2]*256 + rxBytes[3]   #Add High and Low Bytes
 #       irRange = irRange                       #Scale Range
 #       return (irAngle, irRange)
#        angle = rxBytes[0]*256 + rxBytes[1]      #Add High and Low Bytes
#        distance = rxBytes[2]*256 + rxBytes[3]   #Add High and Low Bytes
#        return (angle, distance)
        return (rxBytes[0], rxBytes[1], rxBytes[2], rxBytes[3])

def detect_close():
	for event in pygame.event.get():
		if event.type == pygame.QUIT:		# safe quit on closing the window
			pygame.quit()
			sys.exit()
		if event.type == pygame.KEYDOWN:		# look for key presses
			if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:		# safe quit on "q" press or "ESC" press
				pygame.quit()
				sys.exit()

def main():
# Choose Control Code to WRITE Commands to Stepper Motor
#        control= 32    Stepper Stop (32+0+Speed 0)
#        control= 37    Stepper Reinitialise (32+4+Speed 1)
#        control= 41-43 Stepper Go To Fixed Angle (32+8+Speed 1-3)
#        control= 45-47 Stepper Scan Between 2 Angles (32+12+Speed 1-3)
##        control = input ("Lt Click on screen &  Enter control code:")
        
        control = 47          #Change these numbers
        stepperDatum = 3200   #raw steps of Stepper Motor micro switch datum
        fixedAngle = -90      #degrees (to 0.1 deg accuracy)
        scanAngleLt = 45      #degrees (to 1 deg accuracy)
        scanAngleRt = -45     #degrees (to 1 deg accuracy)

# Convert control codes into bytes for transmission
# Angles are converted into steps (11.32/deg) and the centre offset of 2048 steps added         
        if control ==32:       #Stepper Motor Stop Scanning
                print "Stepper Motor Stop Scanning. Speed= 0"
                txBytes[0]= 0  #N/A 
                txBytes[1]= 0  #N/A
                write_I2C(address, control, txBytes)            #sent to sensors

        if control ==37:       #Stepper Motor Reinitialise to Centre Datum
                print "Initialising Stepper Motor at Speed", control-36
                #Convert stepperDatum to Hi & Lo Bytes steps for transmission
                txBytes[0] = int(stepperDatum/256)              #stepperDatum Hi Byte
                txBytes[1] = int(stepperDatum - txBytes[0]*256) #stepperDatum Lo Byte
#                print(txBytes[0], txBytes[1])                   #print to screen
                write_I2C(address, control, txBytes)            #sent to sensors

        if control >=41 and control <=43:       #Stepper Motor Go To Fixed Angle
                print "Stepper Motor To Fixed Angle at Speed",(control-40)
                #Convert fixedAngle to Hi & Lo Bytes steps for transmission
                fixedAngle = fixedAngle*11.32 + 2048  #scale for steps/deg & add centre offset
                txBytes[0] = int(fixedAngle/256)              #stepperDatum Hi Byte
                txBytes[1] = int(fixedAngle - txBytes[0]*256) #stepperDatum Lo Byte
#                print(txBytes[0],txBytes[1],int(fixedAngle))  #print to screen
                write_I2C(address, control, txBytes)          #sent to sensors

        if control >=45 and control <=47:  #Stepper Motor Scan Between Two Angles
                print "Stepper Motor Scanning at Speed",(control-44)
                #Convert scanAngleLt & Rt to Byte steps for transmission
                scanAngleLt = scanAngleLt*11.32 + 2048  #scale for steps/deg & add offset
                scanAngleRt = scanAngleRt*11.32 + 2048  #scale for steps/deg & add offset
                txBytes[0] = int(scanAngleLt/16)        #convert to Byte (0 to 255)
                txBytes[1] = int(scanAngleRt/16)        #convert to Byte (0 to 255)
#                print(txBytes[0],txBytes[1])            #print to screen
                write_I2C(address, control, txBytes)    #sent to sensors   
                
        time.sleep(0.005)           #Wait 5 msecs
        minRange = 300
        
# Choose Control Code to READ Data from Stepper Motor and I-R Sensor
#        control= 112 Send Back Current I-R Range & Stepper Angle (96+16)
#        control= 116 Send Back Nearest I-R Range & Stepper Angle (96+20)
        control = 112   #Change this number

# Loop to READ the  Stepper Motor (angleIR) and I-R Sensor Range (rangeIR)
        while True:
                detect_close()  #Sub Routine to detect Key presses to close programme
                rxBytes = read_I2C(address,control,numBytes)  #Read 4 Bytes                
                angle = rxBytes[0]*256 + rxBytes[1]     #Add High and Low Bytes
                distance = rxBytes[2]*256 + rxBytes[3]  #Add High and Low Bytes
# Select and Scale Sensor Readings for rangeIR and AngleIR
                if control==112:   #Get Current I-R Range & Stepper Angle
                        angleIR = (angle-2048)/11.32 #remove offset & scale for steps/deg
                        rangeIR = distance           #Scale IR sensor Range
                        if rangeIR > 300:            #Limit rangeIR
                                rangeIR = 300
                        angleIR = '{0:.1f}'.format(angleIR) # formatted to two decimal places
                        screen_display(angleIR, rangeIR) # Sub Routine to display values
                                
                if control==116:   #Get Nearest I-R Range & Stepper Angle
                        angleIR = (angle-2048)/11.32 #remove offset & scale for steps/deg
                        rangeIR = distance           #Scale IR sensor Range
                        if rangeIR > 300:            #Limit rangeIR
                                rangeIR = 300
                        angleIR = '{0:.1f}'.format(angleIR) # formatted to two decimal places
                        screen_display(angleIR, rangeIR) # Sub Routine to display values
                                
#                       if rangeIR< minRange:
#                               minRange = rangeIR
#                               print"Min range", minRange
#                       if angle > 42.5 or angle < -42:
#                               minRange = 300


                time.sleep(0.005)		 # sleep for 5 ms              	
#                time.sleep(0.010)		 # sleep for 10ms
main()

#---------------------------------------------------------
'''        
def init_stepper(busData, controlCodes):
def init_stepper(control):
	print "Initialising Stepper"
	try:
		bus.write_block_data(busData['address'],controlCodes['initMotor'],busData['txBytes'])
	except Exception as err:
		print err
		print "Failed to write to the bus"

        busData = {
        'address' : 4,
        'txBytes' : [0,0],
        'rxBytes' : 0, 
        'numBytes': 4
        }

        controlCodes = {
        'stop' : 32,            #32 zero speed
        'initMotor' : 33,       #37 slow speed
        'fixedAngle' : 39,      #43 max speed
        'betweenAngles' : 42,   #46 medium speed
        'readAngle' : 116       #112
        }

#       init_stepper(control)
#	init_stepper(busData,controlCodes)
#	time.sleep(5)
#	rotate_stepper(busData,controlCodes) #Move stepper motor to center

'''	


# testinfrared.py
'''
import math
import time
import sys
import pygame
import smbus

# set up I2C serial link
bus = smbus.SMBus(1)     # There are two I2C SMbus available on the R-Pi
address = 4              #I2C Sensors Board Address
numBytes = 4             #I2C Number of Bytes Received and Transmitted by Sensors Board

pygame.init()
# Preset colours to use
black = (0,0,0)
white = (255,255,255)
red = (255,0,0)
# Set display parameters of the window
displayWidth = 190
displayHeight = 50
screen = pygame.display.set_mode((displayWidth,displayHeight)) 
font = pygame.font.SysFont('Avenir Next', 20)	# Preset font to use
pygame.display.set_caption("IR Test")

def screen_display(irAngle, irRange):		
        screen.fill(black)	# set the screen background to black (default)
        screen.blit(font.render("IR Angle: ",True,white),(15,5))
        screen.blit(font.render(str(irAngle),True,white),(95,5))
        screen.blit(font.render("IR Range: ",True,white),(15,25))
        screen.blit(font.render(str(irRange),True,white),(95,25))
        pygame.display.update()		# update the display on each loop

def detect_close():
        for event in pygame.event.get():
                if event.type == pygame.QUIT:	# safe quit on closing the window
                        pygame.quit()
                        sys.exit()
                if event.type == pygame.KEYDOWN:        # detect "q" or "ESC" key presses
                        if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                                pygame.quit()
                                sys.exit()

def read_IR(address,Control,numBytes):
        rxBytes = bus.read_i2c_block_data(address, Control, numBytes)
        irAngle = rxBytes[0]*256 + rxBytes[1]   #Add High and Low Bytes
        irAngle = irAngle/11.32 -135            #Scale Angle
        irRange = rxBytes[2]*256 + rxBytes[3]   #Add High and Low Bytes
        irRange = irRange                       #Scale Range
        return (irAngle, irRange)
    
def main():
        Control = 112     #I2C Sensor Control Code to Read IR Sensor Angle & IR Range data
        readBytes = (0,0)   #I2c Read two bytes of data (irAngle and irRange)
    
        while True:         # repeat until programme closed
                detect_close()  #Sub Routine to detect Specific Key presses to close programme

                readBytes = read_IR(address,Control,numBytes) #Sub Routine to READ IR sensor

                irAngle = '{0:.2f}'.format(readBytes[0]) # formatted to two decimal places
                irRange = readBytes[1]                
                if irRange > 200:
                        irRange = 200

                screen_display(irAngle, irRange) # Sub Routine to display values
                time.sleep(0.005)		 # sleep for 5 ms
		
main()                  # Call Sub Routine main()
'''
#-------------------------------------------------------------
'''
def read_angle(busData, controlCodes):
	rxBytes = bus.read_i2c_block_data(busData['address'], controlCodes['readAngle'],busData['numBytes'])
	irAngle = rxBytes[0]*256 + rxBytes[1]
	irAngle = irAngle/11.32 -135
	return irAngle

def rotate_stepper(busData, controlCodes):
	print "Attempting rotation"
	try:
		bus.write_block_data(busData['address'],controlCodes['betweenAngles'],[20,-20])
	except Exception as err:
		print err
'''	



 
