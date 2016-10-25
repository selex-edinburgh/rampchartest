#!/usr/bin/env python

import time
import serial
import sys
import pygame

pygame.init()

displayWidth = 190
displayHeight = 50

screen = pygame.display.set_mode((displayWidth,displayHeight))

pygame.display.set_caption("Motor Test")

black = (0,0,0)
white = (255,255,255)
red = (255,0,0)

screen.fill(black)
font = pygame.font.SysFont('Avenir Next', 20)

def getRevision():
	cpurevision = "0"
	try:
		f = open('/proc/cpuinfo','r')
		for line in f:
			if line[0:8]=='Revision':
				cpurevision = line[11:17]
		f.close()
	except:
		cpurevision = "0"
	return cpurevision

def versionCheck():
	port = "0"
	if getRevision() == 'a02082' or getRevision() == 'a22082':
		port = '/dev/ttyS0'
	elif getRevision() == 'a01041' or getRevision() == 'a21041':
		port = '/dev/ttyAMA0'
	return port

def getTelemetry(ser):
    telemetry = ser.read(4)
    return ord(telemetry[0]),ord(telemetry[1])

def setSerial():
    try:
        ser= serial.Serial(                     #Set up Serial Interface
            port=versionCheck(),                #UART using Tx pin 8, Rx pin 10, Ground pin 6 
            baudrate=38400,                     #bits/sec
            bytesize=8, parity='N', stopbits=1, #8-N-1  protocal
            timeout=1                           #1 sec
        )
        return ser
    except Exception as err:
        print err

def main():
    ser = setSerial()

    fwd = 127
    turn = 127

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    fwd += 10
                elif event.key == pygame.K_DOWN:
                    fwd -= 10
                elif event.key == pygame.K_RIGHT:
                    turn += 10
                elif event.key == pygame.K_LEFT:
                    turn -= 10
                elif event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    sys.exit()
        ser.write(chr(fwd))
        ser.write(chr(turn))
        screen.fill(black)
        screen.blit(font.render("Telemetry: (forward, turn)",True,white),(15,5))
        screen.blit(font.render(str(getTelemetry(ser)),True,white),(110,25))
        pygame.display.update()
            
        time.sleep(0.016)
            
main()
pygame.quit()
quit()
