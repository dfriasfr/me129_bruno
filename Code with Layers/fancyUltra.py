import lowLevel as ll
import moveLevel as ml

#global obstable

def herd(allSensors, motor):

    global obstable
    obstable = isObstable(allSensors, motor)

    print(obstable)

    # try 8 different cases
    if obstable == 5 or obstable == 0:
        ml.driveStraight(1, motor)

    elif obstable == 1:
        #soft turn left
        ml.softTurn(1, motor)

    elif obstable == 4:
        #soft turn right
        ml.softTurn(-1, motor)

    elif obstable == 3:
        #spin left
        ml.spinTurn(1, motor)

    elif obstable == 6:
        #spin right
        ml.spinTurn(-1, motor)

    elif obstable == 2:
        # turn around
        state = ll.readIRs()
        ml.spinTurn(1, motor)
        ll.time.sleep(1)
        while state == 0:
            ml.spinTurn(1, motor)

        #back up
        #motor.set(-0.6, -0.6)
        #ll.time.sleep(0.5)
        #ml.spinTurn(1, motor)
        #ll.time.sleep(0.2)

    elif obstable == 7:
        motor.set(0,0)

    return obstable



def isObstable(allSensors):
    left = allSensors[0]
    front = allSensors[1]
    right = allSensors[2]
    #30 cm is a good threshold value for us when checking intersections
    #because otherwise, Bruno could get confused with the sides of the tunnels otherwise

    l = int(left.distance < 30)
    f = int(front.distance < 30)
    r = int(right.distance < 30)
    num = int(str(l) + str(f) + str(r), 2)
    return num 

def wallFollow(mode, allSensors, motor):#assumes wall is on left side
    left = allSensors[0]
    front = allSensors[1]
    right = allSensors[2]
    print(left.distance, right.distance, front.distance)
    #dd is our desired distance from the wall
    dd = 8

    #calculate the error
    if mode == 'left':
        e = left.distance - dd

        #scale the error proportionally using gain, k
        k = 0.05
        u = k*e
        #adjust the speed of the motors
        if front.distance < 25:
            motor.set(0,0)
            ll.time.sleep(60)
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
        if front.distance < 25:
            motor.set(0,0)
            ll.time.sleep(60)
        else:
            PWM_left = max(0.5, min(0.9, 0.7 - u))
            PWM_right = max(0.5, min(0.9, 0.7 + u))
            motor.set(PWM_left, PWM_right)

def tunnelFollow(allSensors, motor):
    left = allSensors[0]
    front = allSensors[1]
    right = allSensors[2]

    print(left.distance, right.distance, front.distance)
    #we want to balance the distances on both sides
    totalDist = (left.distance + right.distance)

    dd = totalDist/2

    e = left.distance - dd

    #scale the error proportionally using gain, k
    k = 0.05
    u = k*e
    #adjust the speed of the motors
    PWM_left = max(0.5, min(0.9, 0.7 - u))
    PWM_right = max(0.5, min(0.9, 0.7 + u))
    
    motor.set(PWM_left, PWM_right)

def getDistances(allSensors):
    #return (allSensors[0].distance, allSensors[1].distance, allSensors[2].distance)
    return (allSensors[0].lastFive, allSensors[1].lastFive, allSensors[2].lastFive)

def wasObstacle(allSensors):
    outcome = [False, False, False]
    for i in range(3):
        distances = allSensors[i].lastFive
        #print(distances)
        #now check if any of the values in the list is less than 30
        count = sum(j < 30 for j in distances)
        outcome[i] = (count > 0)
    outcome = [outcome[1], outcome[0], False, outcome[2]]
    return outcome

def frontObstacle(allSensors):
    return allSensors[1].distance < 15

    
