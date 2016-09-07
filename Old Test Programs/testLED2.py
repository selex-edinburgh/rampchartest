import time, signal

x = 10
while True:
    Print ("Hello")


#try:
#    import RPi.GPIO as GPIO
#except RuntimeError:
#    print("Error: importing RPi.GPIO")
#
#GPIO.setwarnings(False)                 #Inhibit GPIO Warnings  ## moved
#GPIO.cleanup()                          #Reset all LEDs to OFF  ##
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(24, GPIO.OUT)
#GPIO.setup(25, GPIO.OUT)
#GPIO.setup(23, GPIO.IN)
#GPIO.setup(18, GPIO.IN)

#try:
#    while True:
#        StopGo = GPIO.input(18)
#        if StopGo == 0:                     #StopGo Switch ON
#            GPIO.output(24, 1)              #Amber LED ON
#            print "Amber LED STATUS ON", StopGo
#        else:
#            GPIO.output(24, 0)              #Amber LED OFF
#            print '{:>30}'.format('right aligned %d' % StopGo)
#            print "Amber LED STATUS OFF", StopGo
#        time.sleep(1)
#        
#        ProgRun = GPIO.input(23)
#        if ProgRun == 0:                    #Reset Button Pressed
#            GPIO.output(25, 1)              #Green LED ON
#            print "Green LED STATUS ON", StopGo
#        else:
#            GPIO.output(25, 0)              #Green LED OFF
#            print "Green LED STATUS OFF", StopGo
#        time.sleep(0.5)
#
#
#except KeyboardInterrupt:
#    print("KeyboardInterrupt Detected! Will clean the channel and exit")
#    GPIO.cleanup(24)
#    GPIO.cleanup(25)

