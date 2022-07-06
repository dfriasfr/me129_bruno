# Imports
from queue import Queue
from re import L
from tracemalloc import stop
import pigpio
import sys
import time
import numpy as math
import random
import traceback
import threading

IO = pigpio.pi()

###########################
#MOTOR SETUP
###########################

# define the motor pins
MTR1_LEGA = 7
MTR1_LEGB = 8

MTR2_LEGA = 5
MTR2_LEGB = 6

leftIR = 14
midIR = 15
rightIR = 18

class Motor:
    def __init__(self, io):
        # Prepare the GPIO connetion (to command the motors).
        print("Setting up the GPIO...")
        
        # Initialize the connection to the pigpio daemon (GPIO interface).
        self.io = io
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
        #stopping motors
        self.set(0,0)

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






#############################
#ULTRASOUND SENSORS
#############################


# define the ultrasound pins
ULTRA1_TRIGGER = 13
ULTRA1_ECHO = 16

ULTRA2_TRIGGER = 19
ULTRA2_ECHO = 20

ULTRA3_TRIGGER = 26
ULTRA3_ECHO = 21

EXPECT_TRIGGER = 'expect_trigger'
EXPECT_RISING = 'expect_rising'
EXPECT_FALLING = 'expect_falling'


class Ultrasonic:

    def __init__(self, trigger, echo, io):
        # prepare setting up the ultrasound sensors
        print("Setting up the ultrasound sensors...")
        # set up the trigger pins as output, echo pins as input
        io.set_mode(trigger, pigpio.OUTPUT)
        io.set_mode(echo, pigpio.INPUT)

        self.trigger = trigger
        self.echo = echo
        self.risetick = 0
        self.distance = 0

        self.lastFive = [1000, 1000, 1000, 1000, 1000]

        # previous state: trigger, rising, falling
        self.state = EXPECT_TRIGGER
        self.cbr = IO.callback(self.echo, pigpio.RISING_EDGE, self.rising)
        self.cbf = IO.callback(self.echo, pigpio.FALLING_EDGE, self.falling)

    def rising(self, gpio, level, tick):
        #print("rising")
        if self.state == EXPECT_RISING:
            # catching the rising edge
            self.state = EXPECT_FALLING
            self.risetick = tick

    def falling(self, gpio, level, tick):
        #print("falling")
        if self.state == EXPECT_FALLING:
            # catching the falling edge
            self.state = EXPECT_TRIGGER

            dt = tick - self.risetick
            self.distance = (343/2)* 100 * dt/1000000
            self.lastFive.pop(0)
            self.lastFive.append(self.distance)
            #print("There is something " + str(self.distance) + " cm away.")

    def trigger_pins(self):
        # pull trigger pins HIGH
        IO.write(self.trigger, 1)

        # hold for 10 microseconds
        time.sleep(0.000010)

        # pull the pins LOW again
        IO.write(self.trigger, 0)

        self.state = EXPECT_RISING




