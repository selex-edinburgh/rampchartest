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
