#

# Imports
import pigpio
import sys
import time
import numpy as math
import random

# Define the motor pins.
MTR1_LEGA = 7
MTR1_LEGB = 8

MTR2_LEGA = 5
MTR2_LEGB = 6

leftIR = 14
midIR = 15
rightIR = 18

IO = pigpio.pi()

#global variables
last = 'C'
state = 0
count = 0

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
            time.sleep(0.3)
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
        
        rel_dir = check()
        
        for j in range(4):
            if curr_inter.streets[j] == UNKNOWN:
                if rel_dir[(j - heading) %4] == True:
                    curr_inter.streets[j] = UNEXPLORED
                else:
                    curr_inter.streets[j] = NOSTREET
        if lastintersection != None:
            lastintersection.streets[heading] = CONNECTED
            curr_inter.streets[(heading - 2) %4] = CONNECTED
            curr_inter.headingToTarget = (heading - 2) %4
        
    except:
        print("I've been here before!")

    curr_inter = intersection(long, lat)
    lastintersection = curr_inter
    dir_list = []
    for k in range(4):
        if curr_inter.streets[k] != NOSTREET:
            dir_list.append(k)
    time.sleep(0.2)
    spinDir = ((random.choice(dir_list) + heading) %4)
    turn(spinDir)
    while readIRs() == 0:
        #assumes underrotation in either direction
        if spinDir>=0:
            motor.setvel(0, 12)
        elif spinDir<0:
            motor.setvel(0, -12)
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
            time.sleep(0.3)
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
            motor.setvel(0, 0)
            break
            
        else:
            motor.setvel(0,0)

    #update long and lat
    new_coords = shift(long, lat, heading)
    long = new_coords[0]
    lat = new_coords[1]
    print(long, lat, heading)
    motor.setvel(0, 0)


def turn(spin):
    global motor
    global heading

    if spin == -1 or spin == 3:
        motor.setvel(0, -13)
        time.sleep(0.9)

    elif spin == 2 or spin == -2:
        motor.setvel(0, 13)
        time.sleep(1.85)

    elif spin == 1 or spin == -3:
        motor.setvel(0, 13)
        time.sleep(0.85)

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

def check():

    global state
    global motor

    check_list = ['forward', 'left', 'backward', 'right']

    #sample left
    time.sleep(0.2)
    startTime = millis()
    
    leftState = False
    while (millis() - startTime) < 1.2*1000:
        #turn left and hope we see something
        motor.setvel(0, 13)
        s = readIRs()
        if s != 0:
            #we found a piece of tape
            leftState = True
            motor.setvel(0,0)
            break
    motor.setvel(0,0)
    check_list[1] = (leftState)

    
    #sample forward
    startTime = millis()
    forwardState = False
    while (millis() - startTime) < 1.2*1000:
        #turn left and hope we see something
        motor.setvel(0, -13)
        s = readIRs()
        if s != 0:
            #we found a piece of tape
            forwardState = True
            motor.setvel(0,0)
            break
    motor.setvel(0,0)
    check_list[0] = (forwardState)

    #sample right
    startTime = millis()
    rightState = False
    while (millis() - startTime) < 1.2*1000:
        #turn left and hope we see something
        motor.setvel(0, -13)
        s = readIRs()
        if s != 0:
            #we found a piece of tape
            rightState = True
            motor.setvel(0,0)
            break
    motor.setvel(0,0)
    check_list[3] = (rightState)

    #assume backward direction exists
    check_list[2] = True
    

    #turn back to center
    startTime = millis()
    while (millis() - startTime) < 1.2*1000:
        motor.setvel(0, 13)
    motor.setvel(0,0)

    print(check_list)
    time.sleep(2)
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
        return_dir = lastintersection.headingToTarget - heading
        motor.setvel(0,0)
        turn(return_dir)
        #print(long, lat, heading)
        while readIRs() == 0:
            #assumes underrotation in either direction
            if return_dir>0:
                motor.setvel(0, 12)
            elif return_dir<0:
                motor.setvel(0, -12)
            else:
                break
        motor.setvel(0, 0)
        justDrive()        
    print("Honey, I'm home!")

def millis():
    return int(round(time.time() * 1000))

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

        for i in range(4):
            #check()
            #drive and find long, lat in drive
            drive()

            #pick a random direction and turn
            #print(intersections)
            

            #time.sleep(2)

        backToHome()

    except BaseException as ex:
        print("Ending due to exception: %s" %repr(ex))

    motor.set(0, 0)        
    motor.shutdown()






