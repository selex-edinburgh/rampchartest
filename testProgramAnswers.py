#testProgramAnswers.py
'''
This file contains help to answer the exercises in the test programmes.

#testMotors.py  cycling option
#Insert this code block at the end of the event key if statements.
'''
                if event.key == pygame.K_y:         #cycle over speed range
                    status = "cycling"

        #cycling code
        if status == "cycling":                     #start cycling
            if fwd >= (127+100):                    #highest speed
                increment = -1
            if fwd <= (127-100):                    #lowest speed
                increment = 1
            fwd = fwd + increment                   #accellerate or decelerate

#Where are you going to declare the variable 'increment' in the code?
#The programme may not cycle if it is in the wrong place!
increment = 1       #accelleration increment


'''
#sensorsTest.py   Add Ultra-Sonic test code
'''
#def screen_display(angle, distance, caption):		
    if caption== "U-S":
        pygame.draw.rect(screen,black,[120,5,60,35])  #blank Ultra-Sonic digits
        screen.blit(font.render("U-S Angle: ",True,white),(15,5))
        screen.blit(font.render("U-S Range: ",True,white),(15,25))
        screen.blit(font.render(str(angle),True,white),(125,5))
        screen.blit(font.render(str(distance),True,white),(125,25))


# Convert control codes into bytes for transmission
    if control >=72 and control <=75:       #Servo Motor Go To Fixed Angle
        print "Servo Motor To Fixed Angle at Speed",(control-72)
        #Convert servoFixedAngle to Hi & Lo Bytes steps for transmission
        servoFixedAngle = (servoFixedAngle+servoOffset)*servoScaling +1500  #servo movement scaling in bits/degree
        txBytes[0] = int(servoFixedAngle/256)              #stepperDatum Hi Byte
        txBytes[1] = int(servoFixedAngle - txBytes[0]*256) #stepperDatum Lo Byte
        print "Angle us =",servoFixedAngle
#       print(txBytes[0],txBytes[1],int(servoFixedAngle))  #print to screen
        write_I2C(address, control, txBytes)          #sent to sensors

    if control >=77 and control <=79:  #Servo Motor Scan Between Two Angles
        print "Servo Motor Scanning at Speed",(control-76)
        #Convert servoScanAngleLt & Rt to Byte steps for transmission
        servoScanAngleLt = (servoScanAngleLt+servoOffset)*servoScaling +1500   #servo movement scaling in bits/degree
        servoScanAngleRt = (servoScanAngleRt+servoOffset)*servoScaling +1500  #servo movement scaling in bits/degree
        txBytes[0] = int(servoScanAngleLt/16)        #convert to Byte (0 to 255)
        txBytes[1] = int(servoScanAngleRt/16)        #convert to Byte (0 to 255)
#       print(txBytes[0],txBytes[1])            #print to screen
        write_I2C(address, control, txBytes)    #sent to sensors

#change between I-R and U-S each read to display both parameters
        if control == 112:   
            control = 144
        else:
            if control == 144:
                control = 112
        if control == 116:       
            control = 148
        else:
            if control == 148:
                control = 116



# Select and Scale Sensor Readings for rangeUS and AngleUS
        if control==144:   #Get Current U-S Range & Servo Angle
            angleUS = (angle-1500)/servoScaling -servoOffset #remove offset
                                                    #and scale for steps/deg
            rangeUS = distance *0.17     #Scale IR sensor Range
#           if rangeUS > 300:            #Limit rangeIR
#               rangeUS = 300
            angleUS = '{0:.1f}'.format(angleUS) #format to two decimal places
            rangeUS = '{0:.0f}'.format(rangeUS) #format to two decimal places
            screen_display(angleUS, rangeUS,"U-S")#Sub Routine to display values
                                
        if control==148:   #Get Nearest U-S Range & Servo Angle
            angleUS = (angle-1500)/servoScaling -servoOffset #remove offset
                                                    #and scale for steps/deg
            rangeUS = distance *0.17      #Scale IR sensor Range
#           if rangeUS > 3000:            #Limit rangeIR
#               rangeUS = 3000
            angleUS = '{0:.1f}'.format(angleUS) #format to two decimal places
            rangeUS = '{0:.0f}'.format(rangeUS) #format to two decimal places
            screen_display(angleUS, rangeUS,"U-S")#Sub Routine to display values
        

