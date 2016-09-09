#desktop/testPrograms/fwd-turnCal.pi   2/9/16
#This program combines MotorTest and OdometerTest to allow calibration of the
# wheel/odometer combination in a straight line and in a turn on-the-spot.
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
displayHeight = 85
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
GPIO.setup(18, GPIO.IN)     #DATA_PIN_1  Left Odometer
GPIO.setup(22, GPIO.IN)     #DATA_PIN_0  Right Odometer
GPIO.setup(24, GPIO.OUT)    #CHIP_SELECT_PIN
GPIO.setup(23, GPIO.OUT)    #CLOCK_PIN

'''
    The init_pins function will initialise the pins.
    It is passed in constants from the main() and uses them to set the pins
    to either input to output after initially clearing the pins to their
    default state.

#def init_pins():


    # initialise the Dictionary
#    constants = {
#        'DATA_PIN_0' : 22,  #Data Right Odom GPIO 22  (was 4)
#        'DATA_PIN_1' : 18,  #Data Left Odom  GPIO 18
#        'CHIP_SELECT_PIN' : 24, #Chip Select     GPIO 24
#        'CLOCK_PIN' : 23,   #Clock           GPIO 23
#        'TICK' : 0.01,      #10ms  That's a Baud rate of 50
        
        'READING_LOW_0_BIT' : 6,    #bits 6-16 contain rotation data from Lt odom
        'READING_LOW_1_BIT' : 22,   #bits 22 to 32 contain rotation data from Rt odom
        'READING_BIT_LENGTH' : 10,  #10 binary bits contain a decimal number of 1024 
        'ROLLOVER_RANGE' : 1024     #1024 to 0 and visa versa
    }
#    GPIO.setup(constants['DATA_PIN_0'], GPIO.IN)
#    GPIO.setup(constants['DATA_PIN_1'], GPIO.IN)
#    GPIO.setup(constants['CHIP_SELECT_PIN'], GPIO.OUT)
#    GPIO.setup(constants['CLOCK_PIN'], GPIO.OUT)
    
#    return constants
'''

'''
    The setSerial function will set up the serial interface UART used to
    communicate with the Rampaging Chariot Master Motor Drive Board.
    Change the port designation dependant on R-Pi 2B or 3B.
    Both use UART Tx pin 8 and Rx pin 10
'''
def setSerial():
    try:
        ser= serial.Serial(         #Set up Serial Interface
#       port = '/dev/ttyS0',        #If a Raspberry Pi 3B 
            port = '/dev/ttyAMA0',  #If a Raspberry Pi 2B 
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
    First two are the fwd and turn bytes sent out. second 2 are TBD
'''
def getTelemetry(ser):  #Not used in this program
    try:
        telemetry = ser.read(4)
        # changes ascii characters into the integer value of that character
        return ord(telemetry[0]),ord(telemetry[1]),ord(telemetry[2]),ord(telemetry[3])  
    except Exception as err:
        print err
'''
NEW
   This function will read from the odometers directly and pass the
   result back to the main(). 
'''
def read_Odometers():
    
    # initialise local variables
#    'TICK' : 0.01      #10ms That's a Baud rate of 50 = very slow
    tick = 0.000005     #Half Odom Serial clock period. 5us=100K bits/s (Max 1MHz)
    i = 15              #bit count index number
    readBitLt = 0       #Lt Odom Data bit
    readBitRt = 0       #Rt Odom Data bit
    angDataLt = 0          #Lt Odom data in 10 bit word
    angDataRt = 0          #Rt Odom data in 10 bit word
    statusLt = 0        #Lt Odom status in 6 bit word
    statusRt = 0        #Rt Odom status in 6 bit word   

# Read the Odometers
    # Bring the chip select pin high and then low before reading data.
    GPIO.output(24, True)   #Chip Select pin High (normal state)
    time.sleep(tick)
    GPIO.output(23, True)   #Clock pin High (normal state)
    time.sleep(tick)
    GPIO.output(24, False)  #Chip Select pin Low (both odom triggered to output data)
    time.sleep(tick)        #Wait min of 500ns
    
    #bit data changes on each rising edge of clock 
    while i >= 0:       # loop 16 times to read 16 bits of data from the odometers
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
        if i > 5:  #first 10 bits contain rotation data 
            angDataLt =((angDataLt << 1) + readBitLt)   #bits 15 to 6
            angDataRt =((angDataRt << 1) + readBitRt) 
        else:        #last 6 bits contain odometer status data for analysis
            statusLt = ((statusLt << 1) + readBitLt)    #bits 5 to 0
            statusRt = ((statusRt << 1) + readBitLt)

        i -= 1      #decrement bit count index number

    time.sleep(tick)        #Complete clock cycle
    GPIO.output(24, True)   #Chip Select pin High (normal state)
    return (angDataLt, angDataRt, statusLt, statusRt)

'''    
#    GPIO.setup(18, GPIO.IN)     #DATA_PIN_1  Left Odometer
#    GPIO.setup(22, GPIO.IN)     #DATA_PIN_0  Right Odometer
#    GPIO.setup(24, GPIO.OUT)    #CHIP_SELECT_PIN
#    GPIO.setup(23, GPIO.OUT)    #CLOCK_PIN
#   'READING_LOW_0_BIT' : 6,    #bits 6-16 contain rotation data from Lt odom
#   'READING_LOW_1_BIT' : 22,   #bits 22 to 32 contain rotation data from Rt odom
#   'READING_BIT_LENGTH' : 10,  #10 binary bits contain a decimal number of 1024 
#   'ROLLOVER_RANGE' : 1024     #1024 to 0 and visa versa


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
    while a < (constants['READING_BIT_LENGTH'] + constants['READING_LOW_0_BIT']):       # loop 16 times to get a 16 bit data read from the odometers
        # as the data pins are read the clock is pulsed high and low
        GPIO.output(constants['CLOCK_PIN'], GPIO.LOW)   # as the data pins are read the clock pulses high and low
        time.sleep(constants['TICK'])
        GPIO.output(constants['CLOCK_PIN'],GPIO.HIGH)
        time.sleep(constants['TICK'])

        readBit[0]= GPIO.input(constants['DATA_PIN_0']) #read a bit from Rt odom pin_0
        readBit[1]= GPIO.input(constants['DATA_PIN_1']) #read a bit from Lt odom pin_1 

        # using a shift left each bit can be pushed to create the word
        data[0] =((data[0] << 1) + readBit[0])  #form each bit into a 16 bit package
        data[1] =((data[1] << 1) + readBit[1])  #using a shift left each bit can be pushed to create the word

        a += 1
    return ((data[1] << 16) | data[0])      # return the two data packets as one 32 bit data block


    The bit_slicer function will take in a bit length and return a chunk
    of that number of a fixed length. This is called from main().

def bit_slicer(startBit, lowBit, count):
    mask = (1 << count) - 1
    return (startBit >> lowBit) & mask




    The handle_rollovers function will detect any rollovers that may
    occur during runtime and thereby achieve a continuous distance count
    A rollover is defined as a jump in angDataLt or angDataRt between 0 and 1024
    One wheel rotation is devided into 3 equal sectors (1024/3 = 341 bits)
    if the new angle and the previous angles are in sectors adjacent to the jump
    a rollover has occurred and 1024 is added or subtracted from the odom distance.
    Note: Time interval between odometer reads must be less than time taken for
    wheel to rotate 2/3 of a turn (this determines max wheel speed allowed)
    This function is called from main().
'''
def handle_rollovers(angDataLt,angDataRt,prevAngDataLt,prevAngDataRt,odomDistLt,odomDistRt):
    # Calculate change in odometer angles
    changeLt = angDataLt - prevAngDataLt  #change in dataLt since last reading
    changeRt = angDataRt - prevAngDataRt  #change in dataRt since last reading

    if (angDataLt <341) and (prevAngDataLt >683):   #positive rollover
        odomDistLt = odomDistLt + 1024 + changeLt
    elif (angDataLt >683) and (prevAngDataLt <341):   #negative rollover
        odomDistLt = odomDistLt - 1024 + changeLt
    else:
        odomDistLt = odomDistLt + changeLt

    if (angDataRt <341) and (prevAngDataRt >683):   #positive rollover
        odomDistRt = odomDistRt + 1024 + changeRt
    elif (angDataRt >683) and (prevAngDataRt <341):   #negative rollover
        odomDistRt = odomDistRt - 1024 + changeRt
    else:
        odomDistRt = odomDistRt + changeRt
    
    prevAngDataLt = angDataLt     #load new dataLt into prevDataLt for next loop
    prevAngDataRt = angDataRt     #load new dataRt into prevDataRt for next loop

            
    return(odomDistLt,odomDistRt,prevAngDataLt,prevAngDataRt)
'''    
#def handle_rollovers(readings, constants, prevReadings, prevRollovers):
#    rolloverRange = 1 << (constants['READING_BIT_LENGTH'])    #READING_BIT_LENGTH=10
    rolloverRange = 1024
    big_jump = rolloverRange / 2                    
    for i in range(0,2):
        change = readings[i] - prevReadings[i]
        prevReadings[i] = readings[i]
        prevRollovers[i] += -1 if (change > big_jump) else (change < -big_jump)
        values[i] = readings[i] + rolloverRange * prevRollovers[i]
    return values


    The init_pins function will initialise the pins.
    It is passed in constants from the main() and uses them to set the pins
    to either input to output after initially clearing the pins to their
    default state.

def init_pins():
    # initialise the Dictionary
    constants = {
        'DATA_PIN_0' : 22,  #Data Right Odom GPIO 22  (was 4)
        'DATA_PIN_1' : 18,  #Data Left Odom  GPIO 18
        'CHIP_SELECT_PIN' : 24, #Chip Select     GPIO 24
        'CLOCK_PIN' : 23,   #Clock           GPIO 23
#        'TICK' : 0.01,      #10ms  That's a Baud rate of 50
#        'TICK' : 0.0001,     #100us = Baud Rate of 5K bits/sec
        'TICK' : 0.00005,     #50us = Spec Baud Rate of 10K bits/sec
        
        'READING_LOW_0_BIT' : 6,    #bits 6-16 contain rotation data from Lt odom
        'READING_LOW_1_BIT' : 22,   #bits 22 to 32 contain rotation data from Rt odom
        'READING_BIT_LENGTH' : 10,  #10 binary bits contain a decimal number of 1024 
        'ROLLOVER_RANGE' : 1024     #1024 to 0 and visa versa
    }
#    GPIO.setwarnings(False)
#    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)      #GPIO number designation (not pin nos)

    GPIO.setup(constants['DATA_PIN_0'], GPIO.IN)
    GPIO.setup(constants['DATA_PIN_1'], GPIO.IN)
    GPIO.setup(constants['CHIP_SELECT_PIN'], GPIO.OUT)
    GPIO.setup(constants['CLOCK_PIN'], GPIO.OUT)
    
    return constants

#--------

    This is the main function of the program. 
    The Rampaging Chariot motors are controlled by two commands
    "Forward Speed" and "Turn Rate" 
    Each byte contains a number between 0 and 255.
    Stationary is 127 with greater numbers demanding increasing
    forward speed or turn rate and lower numbers demanding increasing
    backwards speed or turn rate.

    The main will start off by initialising the two motor commands. 
    Then it will set up the Serial Interface by calling setSerial. 
    Finally a loop is called that will listen and filter any keyboard events. 
    Q and ESC will both safely kill the program.
    s will start the calibration movement
    c will change between straight line and turn on the spot
    space will stop the movement.

    This function will also setup the GPIO pins and within an infinite loop,
    read from the odometers, slice the reading into a readable chunk,
    and then handle any rollovers that might occur before outputting the
    result.
'''
def main():
    # Commands to be actioned once on start of program
    # initialise local variables

    ser = setSerial()   # set up the Serial link to the Master Motor Drive Board
    loop1 = True        # boolean variable to allow exit from main loop
    fwd = 127		# initialise Motor Commands:  no forward movement
    turn = 127		# no Turn Rate

    odomDistLt = 0
    odomDistRt = 0
    prevAngDataLt = 0      #start condition
    prevAngDataRt = 0      #start condition
    # Read odometers once to get wheel angle offsets at robot start position
    # call read_Odometers() function to obtain raw data and status
    (angDataLt,angDataRt, statusLt,statusRt) = read_Odometers() #function call

    pygame.init()           # call Function to Initialise Pygame

    # Commands to actioned endlessly in a loop until program stopped    
    #while True:            # infinite loop as True always evaluates to true
    while loop1 == True:    # loops until loop1 is declared False

        # Motor Code
        for event in pygame.event.get():
            # if a designated keyboard key is pressed do the following actions:
            if event.type == pygame.KEYDOWN:    #detect a key press
                if event.key == pygame.K_s:     #key s 'START fwd'
                    fwd  = 187                  #forward
                    turn = 127                  #no turn                  
                if event.key == pygame.K_r:     #key r 'START reverse'
                    fwd  = 77                   #reverse
                    turn = 127                  #no turn
                if event.key == pygame.K_SPACE: #STOP  (Space Bar)
                    fwd  = 127                  #
                    turn = 127                  #
                if event.key == pygame.K_END:   #STOP  (End Key)
                    fwd  = 127                  #
                    turn = 127                  #
                    
        # limit values & write the fwd and turn to the motors via serial link
        if (fwd < 254) and (fwd > 1):   # valid range to fit in an 8bit byte
            ser.write(chr(fwd))
        else:
            fwd = 127
            ser.write(chr(fwd))
        if (turn < 254) and (turn > 1):
             ser.write(chr(turn))
        else:
            turn = 127
            ser.write(chr(turn))

#        telemetry = getTelemetry(ser)  #Receive Telemetry from chariot serial

#---------------------
        # Odometer Code
        # read odometers for raw data and status
        (angDataLt,angDataRt, statusLt,statusRt) = read_Odometers()    #function call

        # calculate odometer distances by actioning rollovers
        (odomDistLt, odomDistRt, prevAngDataLt, prevAngDataRt) = handle_rollovers(angDataLt,angDataRt,prevAngDataLt,prevAngDataRt,odomDistLt,odomDistRt)
        
#        readings[0] = bit_slicer(data,constants['READING_LOW_0_BIT'],constants['READING_BIT_LENGTH'])
#        readings[1] = bit_slicer(data,constants['READING_LOW_1_BIT'],constants['READING_BIT_LENGTH'])
#        values = handle_rollovers(readings, constants, prevReadings, prevRollovers)      # handle rollovers
#        values = readings      #For Rollover Mode
#        screen.fill(black)     # set the screen background
#        screen.blit(font.render("Odometers: (left, right)",True,white),(15,5))
#        screen.blit(font.render(str((int(values[0]), int(constants['ROLLOVER_RANGE']-values[1]))),True,white),(95,25))
#---------------------
#        pygame.display.update()        # update the display on each loop

        # update the pygame data display        
        screen.fill(black)  #Blank out previous digits in display window

        # display motor output text and values to the screen
        screen.blit(font.render("Fwd Speed   Turn Rate",True,white),(15,5))
        screen.blit(font.render(str(fwd),True,white),(45,25))
        screen.blit(font.render(str(turn),True,white),(123,25))

        # display odometer output text and values to the screen
        screen.blit(font.render("Left Odom   Right Odom   Heading",True,white),(15,45))
        screen.blit(font.render(str(angDataLt),True,white),(45,60))
        screen.blit(font.render(str(angDataRt),True,white),(123,60))
        screen.blit(font.render(str(odomDistLt),True,white),(45,70))
        screen.blit(font.render(str(odomDistRt),True,white),(123,70))
        
        #Display telemetry values
#        screen.blit(font.render(str(telemetry[0]),True,white),(50,25))
#        screen.blit(font.render(str(telemetry[1]),True,white),(100,25))
#        screen.blit(font.render(str(telemetry[2]),True,white),(50,45))
#        screen.blit(font.render(str(telemetry[3]),True,white),(100,45))

        pygame.display.update()     # update the display on each loop

        time.sleep(0.016)       #Wait for 16 msec and repeat loop

# program start            
main()
fwd, turn = 127, 127            #Stop Motors
ser.write(chr(fwd))             #Send Motor Commands
ser.write(chr(turn))
pygame.quit()
sys.exit()
quit()

# program start
#if __name__ == "__main__":
#   main()
#   pygame.quit()
#   quit()
