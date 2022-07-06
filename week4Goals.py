#

# Imports
import pigpio
import sys
import time
import numpy as math

# Define the motor pins.
MTR1_LEGA = 7
MTR1_LEGB = 8

MTR2_LEGA = 5
MTR2_LEGB = 6

leftIR = 14
midIR = 15
rightIR = 18

IO = pigpio.pi()


class Motor:
    def __init__(self):
            # Prepare the GPIO connetion (to command the motors).
        print("Setting up the GPIO...")
        
        # Initialize the connection to the pigpio daemon (GPIO interface).
        self.io = pigpio.pi()
        if not self.io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)

        # Set up the four pins as output (commanding the motors).
        self.io.set_mode(MTR1_LEGA, pigpio.OUTPUT)
        self.io.set_mode(MTR1_LEGB, pigpio.OUTPUT)
        self.io.set_mode(MTR2_LEGA, pigpio.OUTPUT)
        self.io.set_mode(MTR2_LEGB, pigpio.OUTPUT)

        # Prepare the PWM.  The range gives the maximum value for 100%
        # duty cycle, using integer commands (1 up to max).
        self.io.set_PWM_range(MTR1_LEGA, 255)
        self.io.set_PWM_range(MTR1_LEGB, 255)
        self.io.set_PWM_range(MTR2_LEGA, 255)
        self.io.set_PWM_range(MTR2_LEGB, 255)
        
        # Set the PWM frequency to 1000Hz.  You could try 500Hz or 2000Hz
        # to see whether there is a difference?
        self.io.set_PWM_frequency(MTR1_LEGA, 1000)
        self.io.set_PWM_frequency(MTR1_LEGB, 1000)
        self.io.set_PWM_frequency(MTR2_LEGA, 1000)
        self.io.set_PWM_frequency(MTR2_LEGB, 1000)

        # Clear all pins, just in case.
        self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGB, 0)

        print("GPIO ready...")



    def shutdown(self):
        # Clear the pins
        print("Clearing pins...")
        self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGB, 0)

        # Disconnect the interface
        print("Turning off...")
        self.io.stop()

    def set(self, leftDutyCycle, rightDutyCycle):
            # left & right duty cycles are a float from -1.0 to +1.0
        if leftDutyCycle <= 1.0 and leftDutyCycle >= 0.0:
            #then motor needs to run forward
            leftSpeedA = leftDutyCycle*255
            leftSpeedB = 0
        elif leftDutyCycle >= -1.0 and leftDutyCycle < 0.0:
            #then motor needs to run backwards
            leftSpeedA = 0
            leftSpeedB = -1*leftDutyCycle*255
        else:
            #if it's an invalid input (greater than 1 or less than -1)
            #don't do anything
            leftSpeedA = 0
            leftSpeedB = 0            

        if rightDutyCycle <= 1.0 and rightDutyCycle >= 0.0:
            #then motor needs to run forward
            rightSpeedA = rightDutyCycle*255
            rightSpeedB = 0
        elif rightDutyCycle >= -1.0 and rightDutyCycle < 0.0:
            #then motor needs to run backwards
            rightSpeedA = 0
            rightSpeedB = -1*rightDutyCycle*255
        else:
            #if it's an invalid input (greater than 1 or less than -1)
            #don't do anything
            rightSpeedA = 0
            rightSpeedB = 0

        # Left Motor
        self.io.set_PWM_dutycycle(MTR1_LEGA, leftSpeedA)
        self.io.set_PWM_dutycycle(MTR1_LEGB,   leftSpeedB)
        # Right Motor
        self.io.set_PWM_dutycycle(MTR2_LEGA, rightSpeedA)
        self.io.set_PWM_dutycycle(MTR2_LEGB,   rightSpeedB)

    def setLinear(self, speed):
        # slope is originally 57.75
        # invert to get 0.017316
        pwmDuty = (math.abs(speed) + 14.17)*(1/56) 
        if pwmDuty >= 0 and pwmDuty < 1417/5600:
            pwmDuty = 0

        if speed < 0:
                pwmDuty = pwmDuty*-1
        
        self.set(pwmDuty, pwmDuty)

    def setSpin(self, speed):
        pwmDuty = (math.abs(speed) +257)*(1/620)
        if pwmDuty >= 0 and pwmDuty <257/600:
            pwmDuty = 0

        if speed < 0:
            pwmDuty = pwmDuty*-1
        
        self.set(pwmDuty, -1*pwmDuty)

    def setvel(self, linear, angSpeed):
        linearComp = (math.abs(linear) + 14.17)*(1/56) 
        if linearComp >= 0 and linearComp < 1417/5600:
            linearComp = 0

        angComp = (math.abs(angSpeed*180/math.pi) +257)*(1/620)
        if angComp >= 0 and angComp <257/600:
            angComp = 0
        weight = 1.75

        left = (weight*linearComp + angComp)/(1+weight)
        right = (weight*linearComp - angComp)/(1+weight)
        
        if angSpeed > 0:
            l = left
            r = right
            left = r
            right = l
        self.set(left, right)
        #print(left, right)

def readIRs():
    left = IO.read(leftIR)
    mid = IO.read(midIR)
    right = IO.read(rightIR)
    
    num = int(str(left) + str(mid) + str(right), 2)
    return num
    
def lost(lin, ang):
    searching = True
        
    while searching == True:
        print("i am lost :(((")
        lin += 0.0005
        ang += 0.0001
        if ang >= -0.01:
            ang = -2
            lin = 30
        motor.setvel(lin, ang)
        state = readIRs()
        if state != 0:
            if state == 2 or state == 5:
                last = 'C'
            elif state == 1 or state == 3 or state == 7:
                last = 'L'
            elif state == 4 or state == 6:
                last = 'R'
            searching = False
            print("i am not lost :)")
            motor.set(0, 0)



#
#   Main
#
if __name__ == "__main__":

    ############################################################
    

    ############################################################
    motor = Motor()
    
    try:
        lin = 30
        ang = -2
        
        #initialize last
        searching = True
        
        while searching == True:
            lin += 0.0005
            ang += 0.0001
            if ang >= -0.01:
                ang = -2
                lin = 30
            motor.setvel(lin, ang)
            state = readIRs()
            if state != 0:
                if state == 2 or state == 5:
                    last = 'C'
                elif state == 1 or state == 3 or state == 7:
                    last = 'L'
                elif state == 4 or state == 6:
                    last = 'R'
                searching = False
                
                motor.set(0, 0)

        count = 0
        while True:
            past = state
            #read the sensors
            state = readIRs() #a number 0 - 7
            
            
            #take appropriate action
            
            #if drifted left
            
            #change setvel to tank turns (0, ...)
            
            if state == 1:
                #hard turn right
                motor.setvel(35, -4)
                if last == 'C':
                    last = 'L'
            
            elif state == 3:
                #soft turn right
                motor.setvel(40, -0.5)
                if last == 'C':
                    last = 'L'
                
            #if drifted right
            
            elif state == 4:
                #hard turn left
                motor.setvel(35, 4)
                if last == 'C':
                    last = 'R'
                
                
            elif state == 6:
                #soft turn left
                motor.setvel(40, 0.5)
                if last == 'C':
                    last = 'R'
            
            #if centered
            elif state == 2:
                #drive straight
                motor.setvel(40, 0)
                if last == 'L' or last == 'R':
                    last = 'C'
                
            elif state == 7:
                #on tape
                if last == 'L':
                    motor.setvel(35, 1)
                    last = 'R'
                elif last == 'R':
                    motor.setvel(35, -1)
                    last = 'L'
                elif last == 'C':
                    motor.setvel(40, 0)
                
                
            #if branch
            elif state == 5:
                #choose direction; default to right
                if last == 'L':
                    motor.setvel(40, 0.5)
                    last = 'R'
                elif last == 'R' or last == 'C':
                    motor.setvel(40, -0.5)
                    last = 'L'
                
            #if completely off tape
            elif state == 0:
                if last == 'L':
                    #spin right
                    motor.setvel(0, -15)
                elif last == 'R' or last == 'C':
                    #spin left or turn around
                    motor.setvel(0, 15)
                    last = 'R'
                if past == state:
                    count += 1
                else:
                    count = 0
                
                if count > 5000:
                    lost(30, -2)
                    count = 0
                
                
            else:
                motor.setvel(0,0)  
                
            
            
            
            
            
            
            
    except BaseException as ex:
        print("Ending due to exception: %s" %repr(ex))

    motor.set(0, 0)        
    motor.shutdown()






