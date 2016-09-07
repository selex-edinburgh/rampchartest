#!/usr/bin/python

try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print ("Error: importing RPi.GPIO")
import time

def read_raw_val(data, chip_select, clock):

    GPIO.setwarnings(False)
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(data,GPIO.IN)
    GPIO.setup(chip_select, GPIO.OUT)
    GPIO.setup(clock, GPIO.OUT)

    a = 0
    output = 0
    readbit = 0
    GPIO.output(chip_select, GPIO.HIGH)
    time.sleep(0.01)
    GPIO.output(clock, GPIO.HIGH)
    time.sleep(0.01)
    GPIO.output(chip_select, GPIO.LOW)
    time.sleep(0.01)
    GPIO.output(clock, GPIO.LOW)
    time.sleep(0.01)
    while a < 34:
        GPIO.output(clock, GPIO.HIGH)
        time.sleep(0.01)
        readbit = GPIO.input(data)
        output = ((output << 1) + readbit)
        GPIO.output(clock, GPIO.LOW)
        time.sleep(0.01)
        a += 1
    return output

while 1:
    rawval = read_raw_val(22,17,18)
    left = (rawval & 0x0000ffff)
    right = (rawval >> 17) & 0x0000ffff
    print "read: " + str(rawval)
    print "raw rotation left: " + str(left >> 6)
    print "raw rotation right: " + str(right >> 6)
    time.sleep(1)
