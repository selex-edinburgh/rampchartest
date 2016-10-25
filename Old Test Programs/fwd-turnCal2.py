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
displayHeight = 85  #100
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
def getTelemetry(ser):
    try:
        telemetry = ser.read(4)
        # changes ascii characters into the integer value of that character
        return ord(telemetry[0]),ord(telemetry[1]),ord(telemetry[2]),ord(telemetry[3])  
    except Exception as err:
        print err
'''
   This function will read from the odometers directly and pass the
   result back to the main(). 
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

'''
    The bit_slicer function will take in a bit length and return a chunk
    of that number of a fixed length. This is called from main().
'''
def bit_slicer(startBit, lowBit, count):
    mask = (1 << count) - 1
    return (startBit >> lowBit) & mask

'''
    The handle_rollovers function will handle any rollovers that may
    occur during runtime.
    It will detect a rollover as a jump in readings by >512
    and use this to achieve a continuous distance count.
    This function is called from main().
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
    The init_pins function will initialise the pins.
    It is passed in constants from the main() and uses them to set the pins
    to either input to output after initially clearing the pins to their
    default state.
'''
def init_pins():
    # initialise the Dictionary
    constants = {
        'DATA_PIN_0' : 22,  #Data Right Odom GPIO 22  (was 4)
        'DATA_PIN_1' : 18,  #Data Left Odom  GPIO 18
        'CHIP_SELECT_PIN' : 24, #Chip Select     GPIO 24
        'CLOCK_PIN' : 23,   #Clock           GPIO 23
        'TICK' : 0.01,
        'READING_LOW_0_BIT' : 6,    #bits 6-16 contain rotation data from Lt odom
        'READING_LOW_1_BIT' : 22,   #bits 22 to 32 contain rotation data from Rt odom
        'READING_BIT_LENGTH' : 10,  #10 binary bits contain a decimal number of 1024 
        'ROLLOVER_RANGE' : 1024     #1024 to 0 and visa versa
    }
    
    GPIO.setwarnings(False)
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)  #GPIO number designation (not pin nos)

    GPIO.setup(constants['DATA_PIN_0'], GPIO.IN)
    GPIO.setup(constants['DATA_PIN_1'], GPIO.IN)
    GPIO.setup(constants['CHIP_SELECT_PIN'], GPIO.OUT)
    GPIO.setup(constants['CLOCK_PIN'], GPIO.OUT)
    
    return constants

'''
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
#    initPygame()        # call Function to Initialise Pygame
    ser = setSerial()   # set up the Serial link to the Master Motor Drive Board
    loop1 = True        # boolean variable to allow exit from main loop
    fwd = 127		# initialise Motor Commands:  no forward movement
    turn = 127		# no Turn Rate

    # initialise local variables
    mode = "Continuous"
    values = [0,0]
    readings = [0,0]
    prevReadings = [0,0]
    prevRollovers = [0,0]
    init_readings = [0,0]
    is_first = 0
    pygame.init()               #initialise pygame

    constants = init_pins()     # initialise and setup the GPIO pins to use

#    while True:            # infinite loop as True always evaluates to true
    while loop1 == True:    # loops until loop1 is declared False

        #Motor Code   
        for event in pygame.event.get():
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
                    
        # limit values & write the fwd and turn to the motors via serial 
        if (fwd < 254) and (fwd > 1):
            ser.write(chr(fwd))
        else:
            fwd = 127
            ser.write(chr(fwd))
        if (turn < 254) and (turn > 1):
             ser.write(chr(turn))
        else:
            turn = 127
            ser.write(chr(turn))

        '''                

        # Odometer Code            

        data = read_raw(constants)      # read from odometers

        readings[0] = bit_slicer(data,constants['READING_LOW_0_BIT'],constants['READING_BIT_LENGTH'])
        readings[1] = bit_slicer(data,constants['READING_LOW_1_BIT'],constants['READING_BIT_LENGTH'])

        values = handle_rollovers(readings,constants, prevReadings, prevRollovers)      # handle rollovers
#        values = readings      #For Rollover Mode

#        screen.fill(black)     # set the screen background
#        screen.blit(font.render("Odometers: (left, right)",True,white),(15,5))
#        screen.blit(font.render(str((int(values[0]), int(constants['ROLLOVER_RANGE']-values[1]))),True,white),(95,25))

        '''
#--------------------------------------
#        pygame.display.update()        # update the display on each loop
        
        screen.fill(black)  #Blank out previous digits in display window

#        telemetry = getTelemetry(ser)  #Receive Telemetry from chariot serial

        # display motor output text and values to the screen
        screen.blit(font.render("Fwd Speed   Turn Rate",True,white),(15,5))
        screen.blit(font.render(str(fwd),True,white),(45,25))
        screen.blit(font.render(str(turn),True,white),(123,25))

        # display odometer output text and values to the screen
        screen.blit(font.render("Left Odom   Right Odom   Heading",True,white),(15,45))
        screen.blit(font.render(str(int(values[0])),True,white),(45,65))
        screen.blit(font.render(str(int(constants['ROLLOVER_RANGE']-values[1])),True,white),(123,65))
#        screen.blit(font.render(str((int(values[0]), int(constants['ROLLOVER_RANGE']-values[1]))),True,white),(160,65))

        #Display telemetrt values
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
