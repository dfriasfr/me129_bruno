#

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

# define the motor pins
MTR1_LEGA = 7
MTR1_LEGB = 8

MTR2_LEGA = 5
MTR2_LEGB = 6

leftIR = 14
midIR = 15
rightIR = 18

IO = pigpio.pi()

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

    def __init__(self, trigger, echo):
        # prepare setting up the ultrasound sensors
        print("Setting up the ultrasound sensors...")
        # set up the trigger pins as output, echo pins as input
        IO.set_mode(trigger, pigpio.OUTPUT)
        IO.set_mode(echo, pigpio.INPUT)

        self.trigger = trigger
        self.echo = echo
        self.risetick = 0
        self.distance = 0

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
            #print("There is something " + str(self.distance) + " cm away.")

    def trigger_pins(self):
        # pull trigger pins HIGH
        IO.write(self.trigger, 1)

        # hold for 10 microseconds
        time.sleep(0.000010)

        # pull the pins LOW again
        IO.write(self.trigger, 0)

        self.state = EXPECT_RISING

sensors = []

#global variables for ultrasonic sensors
stopflag = False

#multithreading functions
def stopcontinual():
    global stopflag
    stopflag = True


def runcontinual():
    global stopflag
    global sensors

    stopFlag = False
    while not stopflag:
        #trigger the pins
        for i in sensors:
            i.trigger_pins()
        time.sleep(0.08 + 0.04 * random.random())

            



#global variables
last = 'C'
state = 0
count = 0
turnInPlace = 17
straight = 50

# Global Constants:
# Headings
NORTH = 0
WEST = 1
SOUTH = 2
EAST = 3
HEADING = {NORTH:'North', WEST:'West', SOUTH:'South',
    EAST:'East', None:'None'} # For printing
# Street status
UNKNOWN = 'Unknown'
NOSTREET = 'NoStreet'
UNEXPLORED = 'Unexplored'
CONNECTED = 'Connected'
# Global Variables:
intersections = [] # List of intersections
lastintersection = None # Last intersection visited
long = 0 # Current east/west coordinate
lat = -1 # Current north/south coordinate
heading = NORTH # Current heading

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
    global last
    global state

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



def drive():
    global last
    global state
    global count
    global motor
    global long
    global lat
    global heading
    global intersections
    global lastintersection

    while True:
        #read the sensors
        state = readIRs() #a number 0 - 7
        
        
        #take appropriate action
        
        #if drifted left
        
        #change setvel to tank turns (0, ...)
        
        if state == 1:
            #hard turn right
            motor.setvel(40, -4)
            if last == 'C':
                last = 'L'
            count = 0
        
        elif state == 3:
            #soft turn right
            motor.setvel(45, -0.5)
            if last == 'C':
                last = 'L'
            count = 0
            
        #if drifted right
        
        elif state == 4:
            #hard turn left
            motor.setvel(40, 4)
            if last == 'C':
                last = 'R'
            count = 0
            
            
        elif state == 6:
            #soft turn left
            motor.setvel(45, 0.5)
            if last == 'C':
                last = 'R'
            count = 0
        
        #if centered
        elif state == 2:
            #drive straight
            motor.setvel(straight, 0)
            if last == 'L' or last == 'R':
                last = 'C'
            count = 0
            
        elif state == 7:
            #on tape
            motor.setvel(straight, 0)
            time.sleep(0.35)
            motor.setvel(0,0)
            count = 0
            break
            
            
        #if branch
        elif state == 5:
            #choose direction; default to right
            if last == 'L':
                motor.setvel(40, 0.5)
                last = 'R'
            elif last == 'R' or last == 'C':
                motor.setvel(40, -0.5)
                last = 'L'
            count = 0
            
        #if completely off tape
        elif state == 0:
            count += 1
            if count > 20000:
                motor.setvel(0, 0)
                raise Exception("No intersection found.")
            time.sleep(0.35)
            motor.setvel(0, 0)
            break
            
        else:
            motor.setvel(0,0)

    #update long and lat
    new_coords = shift(long, lat, heading)
    long = new_coords[0]
    lat = new_coords[1]
    print(long, lat, heading)
    #instantiate intersection if it doesn't exist
    try:
        curr_inter = Intersection(long, lat)
        prevHeading = heading
        rel_dir = check()
        
        for j in range(4):
            if curr_inter.streets[j] == UNKNOWN:
                if rel_dir[(j - prevHeading) %4] == True:
                    curr_inter.streets[j] = UNEXPLORED
                else:
                    curr_inter.streets[j] = NOSTREET
        
        
    except:
        print("I've been here before!")
        curr_inter = intersection(long, lat)

    if lastintersection != None:
        lastintersection.streets[prevHeading] = CONNECTED
        curr_inter.streets[(prevHeading + 2) %4] = CONNECTED
        curr_inter.headingToTarget = (prevHeading + 2) %4
    
    lastintersection = curr_inter
    unexplored_list = []
    connected_list = []

    for k in range(4):
        if curr_inter.streets[k] != NOSTREET:
            if curr_inter.streets[k] == UNEXPLORED:
                #if the street on the opposite end is also unexplored, connect them
                oppIndex = shift(curr_inter.long, curr_inter.lat, k)
                oppNode = intersection(oppIndex[0], oppIndex[1])
                if oppNode != None and oppNode.streets[(k + 2) %4] == UNEXPLORED:
                    curr_inter.streets[k] = CONNECTED
                    oppNode.streets[(k + 2) %4] = CONNECTED
                    connected_list.append(k)
                else:
                    unexplored_list.append(k)
            elif curr_inter.streets[k] == CONNECTED:
                connected_list.append(k)

    time.sleep(0.3)
    print(curr_inter.streets)
    motor.setvel(0, 0)

def justDrive():
    global last
    global state
    global count
    global motor
    global long
    global lat
    global heading
    global intersections
    global lastintersection

    while True:
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
            count = 0
        
        elif state == 3:
            #soft turn right
            motor.setvel(40, -0.5)
            if last == 'C':
                last = 'L'
            count = 0
            
        #if drifted right
        
        elif state == 4:
            #hard turn left
            motor.setvel(35, 4)
            if last == 'C':
                last = 'R'
            count = 0
            
            
        elif state == 6:
            #soft turn left
            motor.setvel(43, 0.5)
            if last == 'C':
                last = 'R'
            count = 0
        
        #if centered
        elif state == 2:
            #drive straight
            motor.setvel(43, 0)
            if last == 'L' or last == 'R':
                last = 'C'
            count = 0
            
        elif state == 7:
            #on tape
            motor.setvel(40, 0)
            time.sleep(0.5)
            motor.setvel(0,0)
            count = 0
            break
            
            
        #if branch
        elif state == 5:
            #choose direction; default to right
            if last == 'L':
                motor.setvel(40, 0.5)
                last = 'R'
            elif last == 'R' or last == 'C':
                motor.setvel(40, -0.5)
                last = 'L'
            count = 0
            
        #if completely off tape
        elif state == 0:
            count += 1
            if count > 20000:
                motor.setvel(0, 0)
                raise Exception("No intersection found.")
            time.sleep(0.5)
            motor.setvel(0, 0)
            break
            
        else:
            motor.setvel(0,0)

    #update long and lat
    new_coords = shift(long, lat, heading)
    long = new_coords[0]
    lat = new_coords[1]
    
    curr_inter = intersection(long, lat)
    lastintersection = curr_inter
    print(long, lat, heading)
    motor.setvel(0, 0)


def turn(spin):
    global motor
    global heading

    if spin == -1 or spin == 3:
        motor.set(1, -1)
        time.sleep(0.030)
        motor.setvel(0, -16)
        time.sleep(0.8)

    elif spin == 2:
        motor.set(-1, 1)
        time.sleep(0.050)
        motor.setvel(0, 16)
        time.sleep(1.65)

    elif spin == -2:
        motor.set(1, -1)
        time.sleep(0.030)
        motor.setvel(0, -16)
        time.sleep(1.35)

    elif spin == 1 or spin == -3:
        motor.set(-1, 1)
        time.sleep(0.030)
        motor.setvel(0, 16)
        time.sleep(0.65)

    if spin > 0:
        last = 'R'
    elif spin < 0:
        last = 'L'
    elif spin == 0:
        last = 'C'


    #and if spin == 0, don't do anything, keep driving straight
    motor.setvel(0, 0)
    time.sleep(0.3)

    heading = (heading + spin) %4
    #print(heading)


def better_turn(spin):

    global motor
    global heading

    t0 = time.time()
    t1 = time.time()

    #right turns
    if spin == -1 or spin == 3:
        #if we're on the tape, get off the tape
        while readIRs() != 0:
            #kickstart
            motor.set(1, -1)
            time.sleep(0.050)
            motor.setvel(0, -1*turnInPlace)
        motor.set(0,0)

        #kickstart
        motor.set(1, -1)
        time.sleep(0.050)

        while readIRs() == 0:
            motor.setvel(0, -1*turnInPlace)
            t1 = time.time()

        time.sleep(0.05)
        motor.set(0, 0)
        time.sleep(0.3)

    #left turns
    if spin == 1 or spin == -3:
        #if we're on the tape, get off the tape
        while readIRs() != 0:
            #kickstart
            motor.set(-1, 1)
            time.sleep(0.050)
            motor.setvel(0, turnInPlace)
        motor.set(0,0)

        #kickstart
        motor.set(-1, 1)
        time.sleep(0.050)
        while readIRs() == 0:
            motor.setvel(0, turnInPlace)
            t1 = time.time()
        time.sleep(0.05)
        motor.set(0, 0)
        time.sleep(0.3)

    #180 deg turns, default turn to left
    if spin == 2 or spin == -2:

        #if we're on the tape, get off the tape
        while readIRs() != 0:
            #kickstart
            motor.set(-1, 1)
            time.sleep(0.050)
            motor.setvel(0, turnInPlace)
        motor.set(0,0)

        #kickstart
        motor.set(-1, 1)
        time.sleep(0.050)
        while readIRs() == 0:
            motor.setvel(0, turnInPlace)
            t1 = time.time()
        time.sleep(0.05)
        motor.set(0, 0)
        time.sleep(0.3)

        #if we're on the tape, get off the tape
        while readIRs() != 0:
            #kickstart
            motor.set(-1, 1)
            time.sleep(0.050)
            motor.setvel(0, turnInPlace)
        motor.set(0,0)

        #kickstart
        motor.set(-1, 1)
        time.sleep(0.050)
        while readIRs() == 0:
            motor.setvel(0, turnInPlace)
            t1 = time.time()
        time.sleep(0.05)
        motor.set(0, 0)
        time.sleep(0.3)

    heading = (heading + spin) %4


def TurnToNextStreet(direction):
    #direction should be either 1 or -1

    global heading
    global motor

    #if we're on the tape, get off the tape
    t0 = time.time()
    motor.set(-1*direction, 1*direction)
    time.sleep(0.040)
    while readIRs() != 0:
        motor.setvel(0, turnInPlace*direction)
    motor.set(0,0)

    t1 = time.time()
    #turn until a street is detected

    #kickstart

    motor.set(-1*direction, 1*direction)
    time.sleep(0.040)

    while readIRs() == 0:
        motor.setvel(0, turnInPlace*direction)
        t1 = time.time()
    time.sleep(0.05)
    motor.set(0, 0)
    time.sleep(0.1)
    


    

    #categorize as 90 deg or 180 deg turn
    if t1 - t0 < 0.95:
        #heading = (heading + 1*direction) %4
        turn = 90
    else:
        #heading = (heading + 2*direction) %4
        turn = 180
    return turn


def check():
    global heading

    check_list = ['forward', 'left', 'backward', 'right']

    #assume backward direction exists
    check_list[2] = True

    #check forward sensors
    time.sleep(0.3)
    check_list[0] = (readIRs() != 0)

    #check left side first
    outcome = TurnToNextStreet(1)
    if outcome == 90:
        check_list[1] = True
        heading = (heading + 1) %4

        TurnToNextStreet(1) #turn to the back line
        heading = (heading + 1) %4

        outcome = TurnToNextStreet(1)
        if outcome == 90:
            check_list[3] = True
            heading = (heading+1) %4
        else:
            check_list[3] = False
            if check_list[0] == True:
                heading = (heading+2) %4
            else:
                heading = (heading+3) %4



    elif outcome == 180: #left doesn't exist
        check_list[1] = False
        heading = (heading+2) %4

        outcome = TurnToNextStreet(1)
        if outcome == 90:
            check_list[3] = True
            heading = (heading+1) %4
        else:
            check_list[3] = False
            if check_list[0] == True:
                heading = (heading+2) %4
            else:
                heading = (heading+0) %4

    print("heading:")
    print(heading)

    print(check_list)
    return check_list

# New longitude/latitude value after a step in the given heading.
def shift(long, lat, heading):
    if heading % 4 == NORTH:
        return (long, lat+1)
    elif heading % 4 == WEST:
        return (long-1, lat)
    elif heading % 4 == SOUTH:
        return (long, lat-1)
    elif heading % 4 == EAST:
        return (long+1, lat)
    else:
        raise Exception("This can't be")

# Find the intersection
def intersection(long, lat):
    list = [i for i in intersections if i.long == long and i.lat == lat]
    if len(list) == 0:
        return None
    if len(list) > 1:
        raise Exception("Multiple intersections at (%2d,%2d)" % (long, lat))
    return list[0]


class Intersection:
    # Initialize - create new intersection at (long, let)
    def __init__(self, long, lat):

        # Save the parameters.
        self.long = long
        self.lat = lat

        # Status of streets at the intersection, in NWSE directions.
        self.streets = [UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN]

        # Direction to head from this intersection in planned move.
        self.headingToTarget = None

        # You are welcome to implement an arbitrary number of
        # "neighbors" to more directly match a general graph.
        # But the above should be sufficient for a regular grid.
        # Add this to the global list of intersections to make it searchable.
        global intersections
        if intersection(long, lat) is not None:
            raise Exception("Duplicate intersection at (%2d,%2d)" % (long,lat))
        intersections.append(self)

    # Print format.
    def __repr__(self):
        return ("(%2d, %2d) N:%s W:%s S:%s E:%s - head %s\n" %
            (self.long, self.lat, self.streets[0],
            self.streets[1], self.streets[2], self.streets[3],
            HEADING[self.headingToTarget]))

def backToHome():
    global heading
    global motor
    global lastintersection
    global long
    global lat
    # turn in the direction the bot came from, then drive
    while long != 0 or lat != 0:
        print("heading is:" + str(lastintersection.headingToTarget))
        return_dir = lastintersection.headingToTarget - heading
        print("return direction is:" + str(return_dir))
        motor.setvel(0,0)
        
        better_turn(return_dir)
        #print(long, lat, heading)
        while readIRs() == 0:
            #assumes underrotation in either direction
            if return_dir>0:
                motor.setvel(0, turnInPlace)
            elif return_dir<0:
                motor.setvel(0, -turnInPlace)
            else:
                break
        motor.setvel(0, 0)
        justDrive()
        
        
    print("Honey, I'm home!")

def isFullyExplored():
    isExplored = True
    for i in range(len(intersections)):
        if isIntersectionExplored(intersections[i]) == False:
            isExplored = False
            break

    return isExplored

def isIntersectionExplored(intersection):
    isExplored = True
    for j in range(len(intersection.streets)):
        if intersection.streets[j] == UNEXPLORED:
            isExplored = False
            break
    return isExplored

def goToTarget(x, y):
    global intersections
    global long
    global lat
    global heading
    global motor
    global lastintersection

    clearHeadings()
    print("All headings cleared..")
    #make a list of to-be-processed intersections
    toProcess = []
    currInter = intersection(x, y)
    toProcess.append(currInter)
    
    while currInter.long != long or currInter.lat != lat:
        for i in range(4):
            #looks at connecting nodes and assigns headingToTarget
            if currInter.streets[i] == CONNECTED:
                nextNode = shift(currInter.long, currInter.lat, i)
                nextInter = intersection(nextNode[0], nextNode[1])
                print(nextNode)
            
                if nextInter.headingToTarget == None:
                    nextInter.headingToTarget = (i + 2) %4
                    if nextInter not in toProcess:
                        toProcess.append(nextInter)
        
                                
        currInter = toProcess.pop(0)
    #if it breaks out of the loop, we are ready to go to the Target 
    while long != x or lat != y:
        return_dir = lastintersection.headingToTarget - heading
        motor.setvel(0,0)
        better_turn(return_dir)
        motor.setvel(0, 0)
        justDrive()



def clearHeadings():
    global intersections

    for i in range(len(intersections)):
        intersections[i].headingToTarget = None

def unexplored():
    return [i for i in intersections if any([s==UNEXPLORED for s in i.streets])]

def victoryDance():
    motor.set(0.9,0.9)
    time.sleep(0.2)
    motor.set(-0.9, -0.9)
    time.sleep(0.6)
    motor.set(0.9, 0.9)
    time.sleep(0.3)
    motor.set(0,0)

def softTurn(dir):
    motor.setvel(45, dir*0.45)

def spinTurn(dir):
    motor.setvel(0, dir*turnInPlace)


def driveStraight(dir):
    motor.setvel(dir*straight, 0)


def herd(allSensors):
    left = allSensors[0]
    front = allSensors[1]
    right = allSensors[2]

    obstable = isObstable(left, front, right)

    print(obstable)

    # try 8 different cases
    if obstable == 5 or obstable == 0:
        driveStraight(1)

    elif obstable == 1:
        #soft turn left
        softTurn(1)

    elif obstable == 4:
        #soft turn right
        softTurn(-1)

    elif obstable == 3:
        #spin left
        spinTurn(1)

    elif obstable == 6:
        #spin right
        spinTurn(-1)

    elif obstable == 2:
        #back up
        motor.set(-0.6, -0.6)
        time.sleep(0.5)
        spinTurn(1)
        time.sleep(0.2)

    elif obstable == 7:
        motor.set(0,0)



def isObstable(left, front, right):
    l = int(left.distance < 30)
    f = int(front.distance < 30)
    r = int(right.distance < 30)
    num = int(str(l) + str(f) + str(r), 2)
    return num 

def wallFollow(mode, left, front, right):#assumes wall is on left side
    #dd is our desired distance from the wall
    dd = 35

    #calculate the error
    if mode == 'left':
        e = left.distance - dd

        #scale the error proportionally using gain, k
        k = 0.05
        u = k*e
        #adjust the speed of the motors
        if front.distance < 25 or right.distance < 25:
            motor.set(0,0)
            time.sleep(60)
        else:
            PWM_left = max(0.5, min(0.9, 0.7 - u))
            PWM_right = max(0.5, min(0.9, 0.7 + u))
            motor.set(PWM_left, PWM_right)
        
    elif mode == 'right':
        e = right.distance - dd

        #scale the error proportionally using gain, k
        k = 0.05
        u = -k*e
        #adjust the speed of the motors
        if front.distance < 25 or left.distance < 25:
            motor.set(0,0)
            time.sleep(60)
        else:
            PWM_left = max(0.5, min(0.9, 0.7 - u))
            PWM_right = max(0.5, min(0.9, 0.7 + u))
            motor.set(PWM_left, PWM_right)




#
#   Main
#
if __name__ == "__main__":

    ############################################################
    

    ############################################################

    #initialize objects
    motor = Motor()

    ULTRA1 = Ultrasonic(ULTRA1_TRIGGER, ULTRA1_ECHO)
    ULTRA2 = Ultrasonic(ULTRA2_TRIGGER, ULTRA2_ECHO)
    ULTRA3 = Ultrasonic(ULTRA3_TRIGGER, ULTRA3_ECHO)
    #sensors = [ULTRA1, ULTRA2, ULTRA3]
    sensors = [ULTRA1, ULTRA2, ULTRA3]

    #start new thread
    thread = threading.Thread(target=runcontinual)
    thread.start()
    time.sleep(0.5)

    #run the main code in the exception handler
    try:
        while True:
            #herd(sensors)
            wallFollow('left', sensors[0], sensors[1], sensors[2])
            print(ULTRA1.distance)
            
    except BaseException as ex:
        print("Ending due to exception: %s" %repr(ex))

        traceback.print_exc()

    #wait for the triggering thread to be done (re-joined)
    stopcontinual()
    thread.join()

    #shutdown sensors
    for i in sensors:
        i.cbr.cancel()
        i.cbf.cancel()

    #shutdown motors
    motor.set(0, 0)        
    motor.shutdown()





