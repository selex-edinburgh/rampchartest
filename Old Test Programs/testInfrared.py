# testinfrared.py
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

def read_I2C(address,Control,numBytes):
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

                readBytes = read_I2C(address,Control,numBytes) #Sub Routine to READ IR sensor

                irAngle = '{0:.2f}'.format(readBytes[0]) # formatted to two decimal places
                irRange = readBytes[1]                
                if irRange > 200:
                        irRange = 200

                screen_display(irAngle, irRange) # Sub Routine to display values
                time.sleep(0.005)		 # sleep for 5 ms
		
main()                  # Call Sub Routine main()
