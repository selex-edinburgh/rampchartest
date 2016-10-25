import math, time, smbus, serial

try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error: importing RPi.GPIO")
          
ser= serial.Serial(                     #Set up Serial Interface				
    port="/dev/ttyAMA0",                #UART using Tx pin 8, Rx pin 10, Ground pin 6 		
    baudrate=9600,                      #bits/sec						
    bytesize=8, parity='N', stopbits=1, #8-N-1  protocol					
    timeout=1                           #1 sec							
)
GPIO.setmode(GPIO.BCM)		

GPIO.setup(24, GPIO.OUT)                #output gpio24 raspberry pin 18   AmberLED		##		
GPIO.setup(25, GPIO.OUT)		#output gpio25 raspberry pin 22   GreenLED		##
GPIO.setup(21, GPIO.IN)                 #input  gpio21 raspberry pin 40   ZeroButton		##
GPIO.setup(18, GPIO.IN)                 #input  gpio18 raspberry pin 12   StopGo Switch		##

GPIO.setwarnings(False)

#print "GPIO 7 output established"
#print "GPIO 8 output established"
#print "GPIO 4 input established"
#print "GPIO 8 input established"			 			

#Designate Course Coordinates
#First number is the distance
#Second number is the heading
waypoints = [(2048+0),(2048+0),							
			(2048+1000),(2048+0),
			(2048+1000),(2048+90),
			(2048+2000),(2048+90),
			(2048+2000),(2048+180),
			(2048+3000),(2048+180),
			(2048+3000),(2048+270),
			(2048+4000),(2048+270),
			(2048+4000),(2048+360)
		]
#print "Waypoints designated"


EndWpNo = 8								        
WaypointIterator = 0                #Waypoint iterator used to access to correct waypoint  (WPIndex)
WPNo = 0                            #Start Waypoint 
ChangeWP = 0		        #Flag
Dist = 2048                         #Zero (offset to avoid negative numbers) (resolution is ~0.5mm)
Hdg  = 2048                         #Zero degrees (offset to avoid negative numbers)
WPDist = 2048                       #Zero Distance to waypoint 0
WPHdg  = 2048                       #Zero Heading to waypoint 0
WPDistM1 = WPDist                   #Previous Waypoint(WP-1) Dist = waypoint 0
WPHdgM1 = WPHdg                     #Previous Waypoint(WP-1) Hdg = waypoint 0
WPDistP1 = waypoints[0]	        #Get EEPROM Distance to waypoint 1	aka	 1st waypoint number
WPHdgP1 = waypoints[1]              #Get EEPROM Heading to waypoint 1	aka 1st heading number
FwdBk = 128                         #Zero Longitudinal Speed Demand 
LtRt  = 128                         #Zero Lateral Turn Demand
LtRtFdFwd = 0                       #Zero Feed Forward Lateral Turn Demand
AmberLED = 0										
GreenLED = 1										
ZeroButton = 1										##
StopGo = 0											##
ReqHdg = 2048		        #Required Heading wrt 2048 						
Dist2Go = 0                         #mm
Hdg2Go = 0                          #deg								
LegMode = 0                         #Stop							
time.sleep(0.3)                     #Time for Motor Drive Bd to initialise	#300 milliseconds
MaxSpd = 30   		        #Max Speed round course					
DecelLong = 7		        #Deceleration Constant Lonitudinal				
DecelLat = 10		        #Deceleration Condtant Lateral Turn				
MaxTrnRate = 50		        #Max Turn Rate						
DistLt = 2048		        #Odometer Left Wheel Distance				
DistRt = 2048		        #Odometer Right Wheel Distance				
ScaleLtWheel = 0.58                 #Correct for left wheel diameter  (2.58mm/bit)		
ScaleRtWheel = 0.58                 #Correct for right wheel diameter  (2.58mm/bit)		
ScaleHdg = 4.08		        #Heading Scale factor (1deg hdg = differential dist mm/4.08)	
                                    #dist between wheels= 234mm					
Arc = 0			        #Flag							
ArcDist = 10		        #10 to stop Div by Zero?					
ArcAng = 2048		        #Finish angle - Start angle				
ArcDir = 0			        #Arc Left
Temp = 0				#Temp store				##
    
#restart()
def halt(FwdBk, LtRt, Dist2Go, Hdg2Go, WPNo):
    FwdBk = 128                                 #Zero Longitudinal Speed Demand 
    LtRt  = 128                                 #Zero Lateral Turn Demand     

#def continue():

#Output To Motor Drive Board 
    #print "FwdBk= ", FwdBk, "LtRt= ", LtRt, "Dist2Go= ", int(Dist2Go), "Hdg2Go= ", int(Hdg2Go), "WpNo= ", WPNo,  ##  

def mainloop():

    global EndWpNo							        
    global WaypointIterator 
    global WPNo 
    global ChangeWP 
    global Dist 
    global Hdg 
    global WPDist 
    global WPHdg 
    global WPDistM1
    global WPHdgM1 
    global WPDistP1 
    global WPHdgP1
    global FwdBk  
    global LtRt 
    global LtRtFdFwd 
    global AmberLED										
    global GreenLED										
    global ZeroButton 										##
    global StopGo 											##
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
    global Arc 							
    global ArcDist 					
    global ArcAng 			
    global ArcDir
    global Temp
    #Output LED Status and Input Switch Status							##

    GPIO.output(24, False)           #AmberLED Auto ON					##
    GPIO.output(25, True)           #GreenLED Power ON					##
    GPIO.input(21)              #Reset to start position				##
    GPIO.input(18)              #Starts Chariot Moving. Flag from Chariot Radio		##
		
    #Python I2C Code to Read Odometers
    #Start at WP0 is 2048 dist to ensure positive numbers
 
    bus = smbus.SMBus(1)     	   	#There are two SMbus available on the R-Pi
    address = 4       		       	#Seven bit Byte: as bit 8 is used for READ/WRITE designation.
    control = 144   		       	#Tells sensor board slave what sensor to address and actions to do
    numbytes = 4      		       	#Number of bytes to be received on a READ instruction

    #R-Pi  I2C READ 

    RxBytes = bus.read_i2c_block_data(address, control, numbytes)
    
    #print "Received Bytes>   ", RxBytes[0], RxBytes[1], RxBytes[2], RxBytes[3]          ###Test###

    # The R-Pi will send: address+0(for WRITE), control, restart(bus held high), address+1(for READ), 
    # The Sensor Interface then sends data to R-Pi   RxByte[1], RxByte[2], RxByte[3], RxByte[4]
    # The R-Pi will extend or shorten the RxByte array according to numbytes it designated above.

    DistLt = RxBytes[0]*256 + RxBytes[1]               #Extract high and low bytes of LtWheel Distance	
    DistLt = DistLt * ScaleLtWheel	               #correct for wheel diameter mm = 0.58*bits		
    DistRt = RxBytes[2]*256 + RxBytes[3]               #Extract high and low bytes of RtWheel Distance	
    DistRt = DistRt * ScaleRtWheel	               #correct for wheel diameter mm = 0.58*bits		
    Dist = DistLt/2 + DistRt/2		               #Dist travelled by center of chassis		
    Hdg = ((DistLt-DistRt)/ScaleHdg)+2048              #heading from differential wheel dist 		
    #print "Distance = " , int(Dist), "  Heading = ", int(Hdg)	    ###Test###

    #Initial Safety Code	

    
    if ZeroButton == 1:				       #Stop and reset route
        halt(FwdBk, LtRt, Dist2Go, Hdg2Go, WPNo)
#    AmberLED = StopGo				       #State of STOP/GO Switch input		##
    GPIO.output(24, StopGo)			       #Amber LED indicates state of STOP/GO Switch Input##

    if StopGo == 0:				       #Stop autonomous sequence 
        halt(FwdBk, LtRt, Dist2Go, Hdg2Go, WPNo)

    #Determine LegMode Codes 0 to 4
  
    if WPDist == WPDistP1:			       #Decelerate and stop
        LegMode = 0

    if WPDist == WPDistM1 and WPHdg != WPHdgM1:	       #Spin on the spot
        LegMode = 1

    if WPHdg == WPHdgM1:			       #Straight Line
        LegMode = 2

    if WPDist != WPDistM1 and WPHdg != WPHdgM1:	       #Arc
        LegMode = 3

    if WPNo > EndWpNo:				       #Finish
        LegMode = 4

#'Next Waypoint Logic              Code always actioned on first pass
#'Change waypoint when Dist2Go < +/-10mm unless this leg is a Spin
#'Max dist travelled at 15MPH = 107mm in 16ms Tx frame rate

    #Last waypoint logic
    if LegMode == 4:				       #Finish
        WPNo = EndWPNo + 1
        GreenLED = 0        
        GPIO.output(25, GreenLED)								##
        halt(FwdBk, LtRt, Dist2Go, Hdg2Go, WPNo) 
    
    ChangeWP = 0			               #Reset waypoint flag

    if LegMode == 1:			               #Spin on Spot
        if Hdg2Go < 5:			
            ChangeWP = 1 		               #Load next waypoint        
    if LegMode == 2 or LegMode == 3:	               #Straight Line or Arc
        if Dist2Go < 10:
            ChangeWP = 1		               #Load next waypoint  
            
    #Load next waypoint data
    if ChangeWP == 1:
        WPNo = WPNo + 1         	               #Select next waypoint
        WPDistM1 = WPDist       	               #Store existing waypoint distance
        WPHdgM1  = WPHdg        	               #Store existing waypoint heading
        WPDist = WPDistP1       	               #Transfer waypoint dist                    
        WPHdg  = WPHdgP1		               #Transfer waypoint heading
        
        
        #By iterating  " WaypointIterator" by 2 each time, it allows us to get the distance
        #and heading on each loop respectively
        
        WPDistP1 = waypoints[WaypointIterator - 2]  	#Load Next Waypoint Data     
        WPHdgP1  = waypoints[WaypointIterator - 1]    	#Load Next Waypoint Data  
        

	#Arc Code deleted for test
    if LegMode == 3:				#Arc				##
        print "Leg Mode3 = Arc selected. Not valid at present"			##
 					
    #Control Algorithms									##

    #Longitudinal Code
    #Test if Fwd or Bk movement is required to reach waypoint and set max speed		
    #If stop at next Waypoint or Turn on Spot required decelerate at set dist to go
	
    FwdBk = 128                      	            #Reached WPDist (within error allowed) 		

    #Calculate Dist2Go as always positive number
    if Dist < WPDist:                               #Go Forward towards Waypoint
        Dist2Go = WPDist - Dist              
        FwdBk = 128 + MaxSpd
        
        #Deceleration                               #NB: <DecelDist = MaxSpd*DecelLong 
        if LegMode == 0 or LegMode == 1:            #Stop or Turn on Spot               
            if Dist2Go < (MaxSpd*DecelLong):        #Decelerate & creep forward to waypt                                         
                FwdBk= 128+(Dist2Go/DecelLong)+5    #128-MaxSpd*Dist2Go/DecelDist    
  
    if Dist > WPDist:                         	    #Go Backwards as past waypoint           
        Dist2Go = Dist - WPDist              
        FwdBk = 128 - MaxSpd 
        #Deceleration                               #NB: <DecelDist = MaxSpd*DecelLong 
        if LegMode == 0 or LegMode == 1:            #Stop or Turn on Spot                    
            if Dist2Go < (MaxSpd*DecelLong):        #Decelerate
                FwdBk= 128-(Dist2Go/DecelLong)-5    #128-MaxSpd*Dist2Go/DecelDist    
       
        
    #Lateral Code
    LtRt = 128                                      #Reached WPHdg (within error allowed) 
 
    #For Straight or Spin the ReqHdg is the Waypoint Heading 
    ReqHdg = WPHdg     

    #For Arc the ReqHdg is calculated as proportion of distance between waypoints
    #   and moves round the arc as Dist2Go decreases
    #if LegMode == 3:                                #Arc 
     #   if Dist2Go > ArcDist:                       #Not yet reached previous Waypoint
      #      ReqHdg = WPHdgM1                    
       # else:     
      #      Temp = (ArcDist-Dist2Go)/3*ArcAng/(ArcDist/3)     #/3 to avoid overflow 
      #  if ArcDir == 0:
      #      ReqHdg = WPHdgM1 + Temp                 #Left Arc
      #  if ArcDir == 1:
      #      ReqHdg = WPHdgM1 - Temp                 #Right Arc
            
    #Common code for Straight line, turn on spot and Arc
    
    #if Hdg < ReqHdg:                            #Turn Left to required heading
     #   Hdg2Go = ReqHdg - Hdg                   #Hdg2Go is always +ve 
      #  LtRt = 128 - MaxTrnRate              
                                                #NB<DecelAng=MaxTrnRate*DecelLat
       # if Hdg2Go < (MaxTrnRate*DecelLat/10):       #Decelerate Turn Rate
       #     LtRt = 128-(Hdg2Go*10/DecelLat)-5       #128-MaxTrnRate*Hdg2Go/DecelAng
       #     if LegMode == 3:                            #Arc
       #         LtRt = 128-(Hdg2Go*10/DecelLat)         #Overwrites previous LtRt
        
   # if Hdg > ReqHdg:                            #Turn Right to required heading
   #     Hdg2Go = Hdg - ReqHdg                   #Hdg2Go is always +ve 
   #     LtRt = 128 + MaxTrnRate     
   #     if Hdg2Go < (MaxTrnRate*DecelLat/10):       #Decelerate Turn Rate
   #         LtRt = 128+(Hdg2Go*10/DecelLat)+5       #128-MaxTrnRate*Hdg2Go/DecelAng
   #         if LegMode == 3:                            #Arc
    #            LtRt = 128+(Hdg2Go*10/DecelLat)         #Overwrites previous LtRt 

  
    #Arc Feed Forward Code        
    if LegMode == 3:                            #Arc 
        if ArcDir == 1:
            LtRt = LtRt + LtRtFdFwd      
        if ArcDir == 0:
            LtRt = LtRt - LtRtFdFwd                  

    if StopGo == 0: 
        halt(FwdBk, LtRt, Dist2Go, Hdg2Go, WPNo)                            #Safety code
          

        #Jump over Halt logic


#Send cmd to drive chariot

#serial_write.py							

#fwdBk = 127-70				         ### For Test ###			
#LtRt = 127+0				         ### For Test ###			
    
    						

    
    if ZeroButton == 1:								        
        WaypointIterator = 0                #Waypoint iterator used to access to correct waypoint  (WPIndex)
        WPNo = 0                            #Start Waypoint 
        ChangeWP = 0		        #Flag
        Dist = 2048                         #Zero (offset to avoid negative numbers) (resolution is ~0.5mm)
        Hdg  = 2048                         #Zero degrees (offset to avoid negative numbers)
        WPDist = 2048                       #Zero Distance to waypoint 0
        WPHdg  = 2048                       #Zero Heading to waypoint 0
        WPDistM1 = WPDist                   #Previous Waypoint(WP-1) Dist = waypoint 0
        WPHdgM1 = WPHdg                     #Previous Waypoint(WP-1) Hdg = waypoint 0
        WPDistP1 = waypoints[0]	        #Get EEPROM Distance to waypoint 1	aka	 1st waypoint number
        WPHdgP1 = waypoints[1]              #Get EEPROM Heading to waypoint 1	aka 1st heading number
        FwdBk = 128                         #Zero Longitudinal Speed Demand 
        LtRt  = 128                         #Zero Lateral Turn Demand
        LtRtFdFwd = 0                       #Zero Feed Forward Lateral Turn Demand
        AmberLED = 0										
        GreenLED = 1										
        #    ZeroButton = 0										##
        #    StopGo = 0											##
        ReqHdg = 2048		        #Required Heading wrt 2048 						
        Dist2Go = 0                         #mm
        Hdg2Go = 0                          #deg								
        LegMode = 0                         #Stop							
        
        
        				
        				
        						
        DistLt = 2048		        #Odometer Left Wheel Distance				
        DistRt = 2048		        #Odometer Right Wheel Distance				
        ScaleLtWheel = 0.58                 #Correct for left wheel diameter  (2.58mm/bit)		
        ScaleRtWheel = 0.58                 #Correct for right wheel diameter  (2.58mm/bit)		
        ScaleHdg = 4.08		        #Heading Scale factor (1deg hdg = differential dist mm/4.08)	
                                            #dist between wheels= 234mm					
        Arc = 0			        #Flag							
        ArcDist = 10		        #10 to stop Div by Zero?					
        ArcAng = 2048		        #Finish angle - Start angle				
        ArcDir = 0			        #Arc Left
        Temp = 0
        ZeroButton = 0
        
    

    FwdBk = int(FwdBk)
    LtRt =int(LtRt)
    ser.write(chr(FwdBk))						
    ser.write(chr(LtRt))
    print (int(DistLt), int(DistRt))
    #time.sleep(0.020)
    mainloop()
      			                    #Reset Route       
mainloop()
