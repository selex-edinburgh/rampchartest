
#TestSquare9.py Dated 30/1/16
#Known bugs:
#   Uncommanded longitudinal movement during R-Pi start up
#   Stepper scanning glitches
#   IR Angle scaling by factor of 2
#   Read IR Angle and Range not returning correct data
#   Arc length Waypoint calc must correct/lengthen odometer distances
#   Initialise Odometers stops IR Scanning

#Update system time using LXTerminal  sudo date 0129144916  #MMDDHHMMYY
#Run Python program using LXTerminal  sudo idle
#Data logging file is '/home/pi/datalogtst.csv'  - A 'Comma Separated Values' Excel file
#File is replaced with new data each run, so rename file after each run to keep data.

import math, time, smbus, serial

#Designate Waypoint Coordinates     ---------------------------------
#First No of Waypoint pair = Total distance from start in mm.
#Second No of Waypoint pair = Heading in degrees at arrival at the Waypoint
route = 4                     #Selects Route Number Below

#Straight line
if route == 1:
    title = "Straight Line Test."
    Waypoints = [(0),(0),      #WP 0	Start Waypoint
                 (2000),(0),   #WP 1
                 (2000),(0),   #WP 2
                 (2000),(0),   #WP 3
         	]
    EndWpNo = 1

#Turn on spot 90 deg Left
if route == 2: 
    title = "Turn on Spot 90deg Left Test."
    Waypoints = [(0),(0),      #WP 0	Start Waypoint
                 (0),(180),    #WP 1
                 (0),(180),    #WP 2
                 (0),(180),    #WP 3
                ]
    EndWpNo = 1

#Turn on spot 90 deg Right
if route == 3: 
    title = "Turn on Spot 90deg Right Test."
    Waypoints = [(0),(0),      #WP 0	Start Waypoint
                 (0),(-180),   #WP 1
                 (0),(-180),   #WP 2
                 (0),(-180),   #WP 3
                ]
    EndWpNo = 1

#Square to Left
if route == 4:
    title = "Square to Left Test."
    Waypoints = [(0),(0),       #WP 0	Start Waypoint				
           	 (1000),(0),    #WP 1
                 (1000),(180),  #WP 2
                 (2000),(180),  #WP 3
                 (2000),(360),  #WP 4
                 (3000),(360),  #WP 5
                 (3000),(540),  #WP 6
                 (4000),(540),  #WP 7
                 (4000),(720),  #WP 8
                 (4000),(720),  #WP 9
                 (4000),(720),  #WP 10
                ]
    EndWpNo = 8

#Square to Right
if route == 5:
    title = "Square to Right Test."
    Waypoints = [(0),(0),        #WP 0					
            	 (1000),(0),     #WP 1
                 (1000),(-180),  #WP 2
                 (2000),(-180),  #WP 3
                 (2000),(-360),  #WP 4
                 (3000),(-360),  #WP 5
                 (3000),(-540),  #WP 6
                 (4000),(-540),  #WP 7
                 (4000),(-720),  #WP 8
                 (4000),(-720),  #WP 9
                 (4000),(-720),  #WP 10
                ]
    EndWpNo = 8

#Arch to Left       arch radius = 500mm 785mm circm for 90 deg
if route == 6:
    title = "Arch to Left Test."
    Waypoints = [(0),(0),       #WP 0					
        	 (1000),(0),    #WP 1
                 (1500),(180),  #WP 2
                 (2000),(360),  #WP 3
                 (3000),(360),  #WP 4
                 (3000),(540),  #WP 5
                 (4000),(540),  #WP 6
                 (4000),(720),  #WP 7
                 (4000),(720),  #WP 8
                 (4000),(720),  #WP 9
                ]
#    EndWpNo = 7
    EndWpNo = 4
#Arch to Right
if route == 7:
    title = "Arch to Right Test."
    Waypoints = [(0),(0),       #WP 0					
        	 (1000),(0),    #WP 1
                 (1785),(-180),  #WP 2
                 (2570),(-360),  #WP 3
                 (3570),(-360),  #WP 4
                 (3570),(-540),  #WP 5
                 (4570),(-540),  #WP 6
                 (4570),(-720),  #WP 7
                 (4570),(-720),  #WP 8
                 (4570),(-720),  #WP 9
                ]
    EndWpNo = 7


print "      ", title, "   No of Waypoints designated =", EndWpNo
    

#Set up Universal Asynchronous Receive-Transmit (UART) Serial Interface  ---------------------       
ser= serial.Serial(                     #Set up Serial Interface				
    port="/dev/ttyAMA0",                #UART using Tx pin 8, Rx pin 10, Gnd pin 6 		
    baudrate=9600,                      #bits/sec						
    bytesize=8, parity='N', stopbits=1, #8-N-1  protocol					
    timeout=1                           #1 sec							
)

#Set up GPIO Interface
try:
    import RPi.GPIO as GPIO             
except RuntimeError:
   print("Error: importing RPi.GPIO")
    
GPIO.setwarnings(False)                 #Inhibit GPIO Warnings
GPIO.cleanup()                          #Reset all LEDs to OFF          
GPIO.setmode(GPIO.BCM)                  #Set GPIO designation to GPIO numbers
GPIO.setup(18, GPIO.IN)                 #input  GPIO18 pin 12   StopGo Switch		
GPIO.setup(23, GPIO.IN)                 #input  GPIO23 pin 16   Reset/ZeroButton		
GPIO.setup(24, GPIO.OUT)                #output GPIO24 pin 18   AmberLED				
GPIO.setup(25, GPIO.OUT)		#output GPIO25 pin 22   GreenLED
print "GPIO 18 Stop/Go Switch input established.  Status= ",GPIO.input(18)
print "GPIO 23 ZeroButton input established.      Status= ",GPIO.input (23)
print "GPIO 24 Amber LED output established"
print "GPIO 25 Green LED output established"

#Initial Safety Code	
GPIO.output(25, False)      #Green LED OFF - Program Not Running 
while GPIO.input(18) == 0:  #GO Switch LIVE GO                
    GPIO.output(24, True)   #Amber LED On - State of Stop/Go Switch                  
    print "WARNING  STOP/GO SWITCH IS IN GO STATE - Switch to STOP for initialisation"
    time.sleep(1)
    
#Output LED Status and Input Switch Status		
GPIO.output(25, True)       #Green LED on - Program Running 
GPIO.output(24, False)	    #Amber LED off - State of Stop/Go Switch                 


#Define 'reset' Sub Routine.    -----------------------------
    #This resets variables to start conditions and initialises the sensors.
def reset():
    print "Reset Function Activated "
    global Waypoints
    global EndWpNo							        
    global ChangeWP
    global WPNo 
    global PrevWPDist
    global PrevWPHdg 
    global WPDist 
    global WPHdg 
    global NextWPDist 
    global NextWPHdg    
    global Dist 
    global Hdg 
    global FwdBk  
    global LtRt 
    global ReqHdg 						
    global Dist2Go
    global Hdg2Go 								
    global LegMode 							
    global MaxSpd				
    global DecelLong 				
    global DecelLat				
    global MaxTrnRate 						
    global DistLt 				
    global DistRt				
    global ScaleLtWheel 	
    global ScaleRtWheel
    global ScaleHdg
    global ScaleHdgdist 				
    global ArcFlag 							
    global ArcDist 					
    global ArcAng 			
    global ArcDir
    global LtRtFdFwd 
    global IRangle
    global IRrange
    global Temp
    global MainLoopCounter
    global RxBytes
    global ElapsedTime

#Set Variables to Start conditions
    ChangeWP = 0	     #Flag  changes to 1 when current Waypoint Distance and Heading are reached
    WPNo = 1                 #The current waypoint robot is travelling towards
    PrevWPDist= Waypoints[0] #Previous Waypoint Distance (WP 0)
    PrevWPHdg = Waypoints[1] #Previous Waypoint Heading  (WP 0)
    WPDist    = Waypoints[2] #The total planned Distance to the current waypoint (WP 1)
    WPHdg     = Waypoints[3] #The total planned Heading to the current waypoint (WP 1)
    NextWPDist= Waypoints[4] #Next waypount Total Distance  (to WP 2)    
    NextWPHdg = Waypoints[5] #Next waypoint Total Heading   (to WP 2)   
    Dist = 0                 #Total Distance in mm travelled by centre of chassis round the course
    Hdg  = 0                 #Total Heading in degrees turned by chassis from the start point
    FwdBk = 128              #Zero Longitudinal Speed Demand 
    LtRt  = 128              #Zero Lateral Turn Demand
    ReqHdg = 0		     #Required Heading wrt 0 						
    Dist2Go = WPDist         #mm
    Hdg2Go = WPHdg           #deg								
    LegMode = 'Stop'         #Stop							
    MaxSpd = 20   	     #Max Speed round course	30				
    MaxTrnRate = 20	     #Max Turn Rate	50					
    DecelLong = 7	     #Deceleration Constant Longitudinal				
    DecelLat = 10	     #Deceleration Condtant Lateral Turn				
    DistLt = 0	             #Odometer Left Wheel Distance				
    DistRt = 0	             #Odometer Right Wheel Distance				
    ScaleLtWheel = 0.92      #Correct for left wheel diameter  (theory= 471dia/512)		
    ScaleRtWheel = 0.92      #Correct for right wheel diameter  		
    ScaleHdg = 4.08	     #Heading Scale factor (1deg hdg = differential dist mm/4.08)						
    ArcFlag = 0		     #Flag							
    ArcDist = 10	     #10 to stop Div by Zero?					
    ArcAng = 0		     #Finish angle - Start angle				
    ArcDir = 0		     #Arc Direction Left or Right
    LtRtFdFwd = 0            #Feed Forward Lateral Turn Demand
    IRangle = 0              #IR Angle relative to Center
    IRrange = 0              #IR Range in mm
    Temp = 0		     #Temp store
    MainLoopCounter = 1	     #Main loop counter for test purposes
    ElapsedTime = 0          #Time to execute the main loop
    print "Variables Designated"	
   
#Set Up I2C Serial Interface.
    bus = smbus.SMBus(1)     #There are two SMbus available on the R-Pi
    address = 4       	     #Seven bit Byte: as bit 8 is used for READ/WRITE designation.
    control = 33             #WRITE 42=Scan, 39=Fixed Angle, 33=Redatum stepper
                             #READ 144=Odom, 84=Infa-Red current, 88=nearest during scan
    numbytes = 4             #No of Bytes to be transmitted including control & numbytes
    Txbyte0 = 0              #Center
    Txbyte1 = 0              #Center
    Txbytes = [Txbyte0, Txbyte1 ]

#Initialise Stepper Motor
    control = 33            #Initialise Stepper Motor at scan speed 1
    bus.write_block_data(address, control, Txbytes)     #I2C command: Redatum stepper motor
    #i2c sends bytes of 9 bits at 100K Baud
    #bus WRITE sends Address with Write bit(0). control, Numbytes, Txbyte0, TxByte1
    print "Stepper Motor Initialising"
    
    #Read Infra-Red Angle and Range
    IRangle = 5                                 #Initial number below 10 degrees
    while IRangle <10:                          #Wait until stepper angle increases to >10 deg
        control = 116                           #Send command to I-R Sensor for Angle and Range
        Rxbytes = bus.read_i2c_block_data(address, control, numbytes) #Send I-R Sensor Angle and Range
        #print "Received Bytes>   ", Rxbytes[0], Rxbytes[1], Rxbytes[2], Rxbytes[3]
        IRangle = Rxbytes[0]*256 + Rxbytes[1]   #Extract high and low bytes of I-R Angle deg
        IRangle = IRangle/11.32 -135   	    #Correct for steps to I-R Angle 		
        IRrange = Rxbytes[2]*256 + Rxbytes[3]   #Extract high and low bytes of I-R Angle deg	
#       IRrange = IRrange*ScaleIRrange          #Correct for range = 		
#        print "A   I-R Angle",int(IRangle),"degrees,   I-R Range",int(IRrange),"mm."
        time.sleep(0.5)                         #Test angle every half second
        
    while int(IRangle) > 0:                     #Wait until stepper angle reduces to <=0 deg
        control = 116                           #Send command to I-R Sensor for Angle and Range
        Rxbytes = bus.read_i2c_block_data(address, control, numbytes) #Send I-R Sensor Angle and Range
        #print "Received Bytes>   ", Rxbytes[0], Rxbytes[1], Rxbytes[2], Rxbytes[3]
        IRangle = Rxbytes[0]*256 + Rxbytes[1]   #Extract high and low bytes of I-R Angle deg
        IRangle = IRangle/11.32 -135   	        #Correct for steps to I-R Angle 		
        IRrange = Rxbytes[2]*256 + Rxbytes[3]   #Extract high and low bytes of I-R Angle deg	
#        IRrange = IRrange*ScaleIRrange         #Correct for range = 		
#        print "B   I-R Angle",int(IRangle),"degrees,   I-R Range",int(IRrange),"mm."
        time.sleep(0.5)                         #Test angle every half second

    print "Finished Stepper Motor Initialisation to Center"
    print "I-R Angle",int(IRangle),"degrees,   I-R Range",int(IRrange),"mm."

#Reset Odometers to Start Conditions
    control = 160
    bus.write_block_data(address, control, Txbytes)   #Reset Odometers to Start conditions
    print "Odometers Reset to Start Conditions"
    
    #Read Odometer Data
    control = 176
    Rxbytes = bus.read_i2c_block_data(address, control, numbytes) #Read Odometers
    #print "Received Bytes>   ", Rxbytes[0], Rxbytes[1], Rxbytes[2], Rxbytes[3]   #Test
    DistLt = Rxbytes[0]*256 + Rxbytes[1] -5000      #Extract high and low bytes of LtWheel Distance	
    DistLt = DistLt * ScaleLtWheel	            #correct for wheel diameter mm = 0.58*bits		
    DistRt = Rxbytes[2]*256 + Rxbytes[3] -5000      #Extract high and low bytes of RtWheel Distance	
    DistRt = DistRt * ScaleLtWheel	            #correct for wheel diameter mm = 0.58*bits		
    Dist = DistLt/2 + DistRt/2		            #Dist travelled by center of chassis from start point		
    Hdg = (DistRt-DistLt)/ScaleHdg   #Positive=Left #Heading from differential wheel dist wrt start Hdg 		
    print "Lt Wheel",int(DistLt), "Rt Wheel",int(DistRt),"Heading",int(Hdg),"degrees"
    print " "

#Start I-R Scanning
    control = 42                            #Scan between two angles 
#    Txbyte0 = 45                            #Left Scan Angle  -100 to +100 Degrees
#    Txbyte1 = -45                           #Right Scan Angle -100 to +100 Degrees
    Txbyte0 = 20                            #Left Scan Angle  -100 to +100 Degrees
    Txbyte1 = -20                           #Right Scan Angle -100 to +100 Degrees
    Txbyte0 = (Txbyte0 +135)*0.943 +0.5     #0.943 = Motor gearing of 11.32/12 to minimise PIC maths
    Txbyte1 = (Txbyte1 +135)*0.943 +0.5     #0.943 = Motor gearing of 11.32/12 to minimise PIC maths
    if Txbyte0 < 1: Txbyte0 =1              #Limit to Byte values
    if Txbyte0 > 254:  Txbyte0 =254         #Limit to Byte values
    if Txbyte1 < 1: Txbyte1 =1              #Limit to Byte values
    if Txbyte1 > 254: Txbyte1 =254          #Limit to Byte values
    Txbytes = [int(Txbyte0), int(Txbyte1) ]
    bus.write_block_data(address, control, Txbytes)   #Start I-R Scanning
    print "Infr-Red Ranger scanning between:",int(Txbyte0 /0.943-135),"and",int(Txbyte1 /0.943-135)

    time.sleep(0.5)


      
reset()                     #Call sub routine to Initialise 


def mainloop():	#***************************************************************
    global Waypoints
    global EndWpNo							        
    global ChangeWP
    global WPNo 
    global PrevWPDist
    global PrevWPHdg 
    global WPDist 
    global WPHdg 
    global NextWPDist 
    global NextWPHdg    
    global Dist 
    global Hdg 
    global FwdBk  
    global LtRt 
    global ReqHdg 						
    global Dist2Go
    global Hdg2Go 								
    global LegMode 							
    global MaxSpd				
    global DecelLong 				
    global DecelLat				
    global MaxTrnRate 						
    global DistLt 				
    global DistRt				
    global ScaleLtWheel 	
    global ScaleRtWheel
    global ScaleHdg
    global ScaleHdgdist 				
    global ArcFlag 							
    global ArcDist 					
    global ArcAng 			
    global ArcDir
    global LtRtFdFwd
    global IRangle
    global IRrange
    global Temp
    global MainLoopCounter
    global ElapsedTime


#HALT Logic     -----------------------------------------
    if GPIO.input(18) == 1:             #STOP/GO Switch =STOP
        GPIO.output(24, False)          #Amber LED Off - State of Stop/Go Switch
        print "Program on hold.  Ready to Start"        
    while GPIO.input(18) == 1:          #STOP/GO Switch =STOP           
        FwdBk = 128                                                         
        LtRt = 128                                                          
        ser.write(chr(FwdBk))		#Output to Motor Drive Board	    			
        ser.write(chr(LtRt))            #Output to Motor Drive Board        
        time.sleep(0.2)                                                     

    if MainLoopCounter == 1:            #Programe Running and First Pass
        #Print file header to screen
        print "/home/pi/datalogtst.csv/"
        print time.strftime ("%H:%M:%S %b %d %Y")   #Print time,month,day,year

        #Open a file to log data separated by commas for excel compatibility =.csv
        logfile=open('/home/pi/datalogtst.csv','w')     #Open file. w= delete old values a=append

        #Write time & date file header
        logfile.write(time.strftime("%H:%M:%S %b %d %Y\n")) #Write time & date file header+new line
        #Write column headings \n
        logfile.write("Time,WPNo,LegMode,Dist2Go,Hdg2Go,DistLt,DistRt,Dist,Hdg,FwdBk,LtRt,IR Angle,IR Range,LoopTime,\n")
        logfile.close()                 #Close file
        print "Autonomous Robot Program Running"        
    
    GPIO.output(24, True)		#Switch Amber LED on - State of Stop/Go Switch
    MainLoopCounter=MainLoopCounter+1	#Increment MainLoopCounter for test
    start_time = time.time()            #Read start of loop time

    time.sleep(0.001)                   #Pause for 1ms  

#READ ODOMETERS     -------------------------------------
    bus = smbus.SMBus(1)    #There are two SMbus available on the R-Pi
    address = 4       	    #Seven bit Byte: as bit 8 is used for READ/WRITE designation.
    control = 176   	    #Tells sensor board slave to read odometers
    numbytes = 4      	    #Number of bytes to be received on a READ instruction

    time.sleep(0.001)                   #Pause for 1ms       
    RxBytes = bus.read_i2c_block_data(address, control, numbytes)   #R-Pi  I2C READ

    # The R-Pi will send: address+0(for WRITE), control, restart(bus held high), address+1(for READ), 
    # The Sensor Interface then sends data to R-Pi   RxByte[0], RxByte[1], RxByte[2], RxByte[3]
    # The R-Pi will extend or shorten the RxByte array according to numbytes it designated above.
    #if (MainLoopCounter)%10 == 0:        #Modulus = 0 print every ten loops +1
    #    print "Received Bytes>   ", RxBytes[0], RxBytes[1], RxBytes[2], RxBytes[3]     

    DistLt = RxBytes[0]*256 + RxBytes[1] -5000      #Extract high and low bytes of LtWheel Distance	
    DistLt = DistLt * ScaleLtWheel	            #correct for wheel diameter mm = 0.58*bits		
    DistRt = RxBytes[2]*256 + RxBytes[3] -5000      #Extract high and low bytes of RtWheel Distance	
    DistRt = DistRt * ScaleLtWheel	            #correct for wheel diameter mm = 0.58*bits		
    Dist = DistLt/2 + DistRt/2		            #Dist travelled by center of chassis from start point		
    Hdg = (DistRt-DistLt)/ScaleHdg   #Positive=Left #Heading from differential wheel dist wrt start Hdg 		
#    time.sleep(0.001)                               #Pause for 1ms

#Waypoint Navigation Logic    --------------------------------
    #LEG MODES  Determine LegMode Codes 0 to 3 from Waypoint Data
    #WPDist is the total Distance of the next Waypoint from start (via previous Waypoints)
    if WPDist == PrevWPDist and WPHdg == PrevWPHdg: #Two consecutive identical waypoints			    
        LegMode = "Stop"                            #Stop and wait for start command
    if WPDist != PrevWPDist and WPHdg == PrevWPHdg:			    
        LegMode = "Line"                            #Straight Line
    if WPDist != PrevWPDist and WPHdg != PrevWPHdg: 
        LegMode = "Arc"                             #Arc
#        print "Leg Mode2 = Arc selected. Currently Under Development"
    if WPDist == PrevWPDist and WPHdg != PrevWPHdg: 
        LegMode = "Turn"                            #Turn on the spot

#CHANGE WAYPOINT LOGIC  (Based on Dist2Go or Hdg2Go)   ----------------------------
    #Note: Max dist travelled at 15MPH = 7mm/msec             
    ChangeWP = 0			            #Reset waypoint flag
    if LegMode == "Line" or LegMode == "Arc":	    #Straight Line or Arc
        if Dist2Go < 10:                            #Was 10                 ##
            ChangeWP = 1		            #Increment waypoint  
    if LegMode == "Turn":		            #Turn on Spot
        if Hdg2Go < 5:			            #Was 5
            ChangeWP = 1 		            #Increment waypoint
            
    if ChangeWP == 1:                               #Load next waypoint data
        WPNo = WPNo + 1         	            #Select next waypoint
        PrevWPDist = WPDist       	            #Store existing waypoint distance
        PrevWPHdg  = WPHdg        	            #Store existing waypoint heading
        WPDist = NextWPDist       	            #Transfer waypoint Dist                    
        WPHdg  = NextWPHdg		            #Transfer waypoint Hdg     
        NextWPDist = Waypoints[(WPNo+1)*2]          #Load Next Waypoint Dist 
        NextWPHdg  = Waypoints[(WPNo+1)*2+1]        #Load Next Waypoint Hdg        

#ARC GEOMETRIC CALCULATIONS     ---------------------------
    #Calculate following parameters once at start of leg:                        
    if ChangeWP ==1 and LegMode == "Arc":	    #Do Arc Calcs once at start of Arc leg				
        #Calc the ArcAngle between start and end of arc in degrees (Max 180 deg)
        if WPHdg >= PrevWPHdg:                      #Arc to Left
            ArcAng = WPHdg-PrevWPHdg                
            ArcDir = 0                              #Arc Left 
        else:                                       
            ArcAng = PrevWPHdg-WPHdg                #Arc to Right
            ArcDir = 1                              #Arc Right
        if ArcAng > 179:                            #Limit ArcAngle to prevent ambiguity (max179deg)
            ArcAng = 179

        #Calculate the Length of Arc 
        if WPDist >= PrevWPDist:                    #Forward ArcDist 
            ArcDist = WPDist-PrevWPDist             #Length of Arc Cord
        else:
            ArcDist = PrevWPDist-WPDist             #Length of Reverse Arc Cord
        if ArcDist < 10:
            ArcDist = 10                            #(Min ArcDist is 10 to prevent divide by zero)

#        Temp= ArcAng/2/57.3                         #/57.3 is deg to Radians
#        if Temp <= 0.1:
#            Temp=1                                  #Divide by zero trap
#        ArcRadius = (WPDist/2)/math.sin(Temp)       #Radius of Arc
#        ArcDist = 2*math.pi*ArcRadius*ArcAng/360    #Length of Arc

        #Calc Arc Feed Forward Turn Rate Demand.    Dist between wheels should be 234mm
        LtRtFeedFwd = 65*MaxSpd/10                  #=(310*ArcAngRadians*MaxSpd)/(ArcDist/1000) (note it was 27*)
        LtRtFeedFwd = LtRtFeedFwd*ArcAng/ArcDist    #=27*MaxSpdLimited/10*ArcAngDeg/ArcDist
#        LtRtFeedFwd = LtRtFeedFwd +10               #Fiddle Factor        

#LAST WAYPOINT and RESET LOGIC      --------------------------
    if WPNo > EndWpNo:				    #Finish
        FwdBk = 127                                 #Zero Longitudinal Speed Demand 
        LtRt  = 127                                 #Zero Lateral Turn Demand     
        ser.write(chr(FwdBk))	                    #Output to Motor Drive Board	    			
        ser.write(chr(LtRt))                        #Output to Motor Drive Board
        print "WPNo LMode Dis2Go Hdg2Go  DistL DistR Dist Hdg   FwdBk LtRt Ang  Rng LoopTime"
        print( WPNo,LegMode,int(Dist2Go),int(Hdg2Go),'  ',int(DistLt),int(DistRt),int(Dist),int(Hdg),FwdBk,LtRt,int(IRangle),int(IRrange),int(ElapsedTime*1000))
        print "Reached Last Waypoint - Switch STOP-GO switch to STOP"
        while GPIO.input(18) == 0:                  #GO Switch LIVE GO                
            GPIO.output(24, True)                   #Amber LED On - State of Stop/Go Switch                  
            time.sleep(0.2)
        print "Press Reset Button to Initialise program to Start conditions"
        while GPIO.input(23) == 1:                  #Reset Button OFF                
            GPIO.output(24, False)                  #Amber LED Off - State of Stop/Go Switch                  
            time.sleep(0.2)
        reset()

#LONGITUDINAL CONTROL ALGORITHMS    ---------------------------------									
    #Test if Fwd or Bk movement is required to reach waypoint and set max speed		
    #If stop at next Waypoint or Turn on Spot required, decelerate at set distance to go
    FwdBk = 128                      	            #Reached WPDist
    if LegMode =="Stop" or LegMode=="Line" or LegMode=="Arc" or LegMode=="Turn":  #
        #Calculate the Dist2Go                          #Always positive number
        if Dist < WPDist:                               #Go Forward towards Waypoint
            Dist2Go = WPDist - Dist              
            FwdBk = 128 + MaxSpd                        #Constant speed until deceleration required
            if LegMode == "Stop" or LegMode == "Turn":  #Stop or Turn on spot
                #Deceleration                           #DecelDist = MaxSpd*DecelLong 
                if Dist2Go < (MaxSpd*DecelLong):        #Distance from waypoint to start deceleration
                    FwdBk= 128+(Dist2Go/DecelLong)+5    #Decelerate & creep forward to waypoint
                   #FwdBk= 128+(Dist2Go/DecelLong)+5    #128+MaxSpd*Dist2Go/DecelDist                                   
  
        if Dist > WPDist:                               #Go Backwards as past waypoint           
            Dist2Go = Dist - WPDist              
            FwdBk = 128 - MaxSpd        
            if LegMode == "Stop" or LegMode == "Turn":  #Stop or Turn on Spot                    
                #Deceleration                           #DecelDist = MaxSpd*DecelLong 
                if Dist2Go < (MaxSpd*DecelLong):        #Decelerate
                    FwdBk= 128-(Dist2Go/DecelLong)-5    #Decelerate & creep forward to waypoint    
                   #FwdBk= 128-(Dist2Go/DecelLong)-5    #128-MaxSpd*Dist2Go/DecelDist                                   


#LATERAL CONTROL ALGORITHMS     -------------------------------------
    LtRt = 127                                      #Reached WPHdg (within error allowed) 
    #DecelLat=10                                    #Test
    #MaxTurnRate =20                                #Test
 
    #Calculate the Required Heading
    #   For Arc the ReqHdg is calculated as proportion of distance between waypoints
    #   and moves round the arc as Dist2Go decreases
    if LegMode=="Line" or LegMode=="Turn":          #Straight line or Turn on Spot
        ReqHdg = WPHdg                              #ReqHdg is the Waypoint Heading
        
    if LegMode=="Arc":                              #Arc
        if Dist2Go > ArcDist:                       #Not yet reached previous Waypoint
            Temp=0                                  #Trimming Factor
            if ArcDir == 0:
                ReqHdg = PrevWPHdg + Temp           #Left Arc
            if ArcDir == 1:
                ReqHdg = PrevWPHdg - Temp           #Right Arc            
        else:                                       #ReqHdg=(ArcDist-Dist2Go)/ArcDist*ArcAng
           Temp = (ArcDist-Dist2Go)/3*ArcAng/(ArcDist/3)     #/3 to avoid overflow
           Temp = Temp+0                            #Trimming Factor
           if ArcDir == 0:                          #Left Arc
              ReqHdg = PrevWPHdg + Temp              
           if ArcDir == 1:                          #Right Arc
              ReqHdg = PrevWPHdg - Temp             

    #Common Lateral Algorithm to acquire or correct ReqHdg   
    if LegMode=="Line" or LegMode=="Arc" or LegMode=="Turn":   
        if Hdg < ReqHdg:                            #Turn Left to required heading
            Hdg2Go = ReqHdg - Hdg                   #Hdg2Go is always +ve 
            LtRt = 128 - MaxTrnRate              
            #Deceleration                           #NB<DecelAng = MaxTrnRate*DecelLat
            if Hdg2Go < (MaxTrnRate*DecelLat/10):   #Decelerate Turn Rate
    #            LtRt = 128-(Hdg2Go*10/DecelLat)-5  #128-(MaxTrnRate*Hdg2Go/DecelLat)
                LtRt = 128-(Hdg2Go*2/DecelLat)-8    #
                if LegMode == "Arc":                #Arc
                    LtRt = 128-(Hdg2Go*10/DecelLat) #Overwrites previous LtRt
                    
    if Hdg > ReqHdg:                                #Turn Right to required heading
        Hdg2Go = Hdg - ReqHdg                       #Hdg2Go is always +ve 
        LtRt = 128 + MaxTrnRate     
        #Deceleration                               #NB<DecelAng=MaxTrnRate*DecelLat
        if Hdg2Go < (MaxTrnRate*DecelLat/10):       #Decelerate Turn Rate
    #        LtRt = 128+(Hdg2Go*10/DecelLat)+5      #128-MaxTrnRate*Hdg2Go/DecelLat
            LtRt = 128+(Hdg2Go*2/DecelLat)+8        #
            if LegMode == "Arc":                    #Arc
                LtRt = 128+(Hdg2Go*10/DecelLat)     #Overwrites previous LtRt 

    #Arc Feed Forward Code        
    if LegMode == "Arc":                            #Arc 
    #    LtRtFdFwd = 16                             #Test value about right for chicane
    #                                             If strays left decrease LtRtFdFwd and visa versa
        if ArcDir == 0:                             #Left Arc
            LtRt = LtRt - LtRtFdFwd
        if ArcDir == 1:                             #Right Arc
            LtRt = LtRt + LtRtFdFwd      
    

#OUTPUT TO MOTOR DRIVE BOARD    -----------------------------------
#    FwdBk = 128                    #Test +ve = Forward with invert below                                      
#    LtRt = 127                     #Test +ve = Right                                     
    if FwdBk > 128+MaxSpd:
        FwdBk = 128+MaxSpd
    if FwdBk < 128-MaxSpd:
        FwdBk = 128-MaxSpd
    if LtRt > 128+MaxTrnRate:
        LtRt = 128+MaxTrnRate
    if LtRt < 128-MaxTrnRate:        
        LtRt = 128-MaxTrnRate

    FwdBk= 127-(FwdBk-127)          #change Fwd to Reverse as Motor Drive Board inverted  ###
    
    FwdBk = int(FwdBk)
    LtRt  = int(LtRt)-2             #Trim factor for straight line
    ser.write(chr(FwdBk))	    #Output to Motor Drive Board	    			
    ser.write(chr(LtRt))            #Output to Motor Drive Board        

#    if (MainLoopCounter+4)%10 == 0:     #Modulus = 0 print every ten loops +4		
#        print "                                                           Fwd/Lt",(FwdBk), (LtRt)
#        print "                            Fwd/Lt",(FwdBk-128), (LtRt-128)


#READ INFRA_RED ANGLE AND RANGE     ---------------------------
    control = 116                           #Send command to I-R Sensor for Angle and Range
    Rxbytes = bus.read_i2c_block_data(address, control, numbytes) #Send I-R Sensor Angle and Range
    #print "Received Bytes>   ", Rxbytes[0], Rxbytes[1], Rxbytes[2], Rxbytes[3]
    IRangle = Rxbytes[0]*256 + Rxbytes[1]   #Extract high and low bytes of I-R Angle deg
    IRangle = IRangle/11.32 -135   	        #Correct for steps to I-R Angle 		
    IRrange = Rxbytes[2]*256 + Rxbytes[3]   #Extract high and low bytes of I-R Angle deg	
    #IRrange = IRrange*ScaleIRrange          #Correct for range = 		
    time.sleep(0.001)                       #Pause for 1ms

#DATALOGGING        ----------------------------------------- 
    logfile=open('/home/pi/datalogtst.csv','a')         #Open datalog file to append values
                                                        #& Write one line of time & data to file
    logfile.write(time.strftime("%H:%M:%S"))            #Write time to file
    logfile.write(", %d" % WPNo)                        #Write decimal data to file
    logfile.write(", %s" % LegMode)                     #Write string data to file
    logfile.write(", %d" % Dist2Go)                     #Write data to file
    logfile.write(", %d" % Hdg2Go)                      #Write data to file
    logfile.write(", %d" % DistLt)                      #Write data to file
    logfile.write(", %d" % DistRt)                      #Write data to file
    logfile.write(", %d" % Dist)                        #Write data to file
    logfile.write(", %d" % Hdg)                         #Write data to file
    logfile.write(", %d" % FwdBk)                       #Write data to file
    logfile.write(", %d" % LtRt)                        #Write data to file
    logfile.write(", %d" % IRangle)                     #Write data to file
    logfile.write(", %d" % IRrange)                     #Write data to file
    logfile.write(", %.2f" % ElapsedTime)               #Write floating point with 2 decimal places
    logfile.write(",\n")                                #New line
    logfile.close()                                     #Close file


#MAINLOOP EXECUTION TIME    ------------------------------------
    end_time = time.time()                      #Read end of loop time
    ElapsedTime = (end_time - start_time)*1000      #msec
    if ElapsedTime <0.016:                     #16ms (62 Hz)
        time.sleep(0.016 - ElapsedTime)        #Delay
    #if ElapsedTime > 0.18:                      #Test message if > 190 ms
    #    if (MainLoopCounter+4)%8 == 0:     #Modulus = 0 print every ten loops +4		
    #    print "ms per loop= ", int(ElapsedTime*1000)  #Print if >16ms
    #Test print to screen - Not included in loop time

    start_time = time.time()                    #Read start of loop time

while MainLoopCounter != 0:                     #Loop to start of mainloop
    mainloop()

