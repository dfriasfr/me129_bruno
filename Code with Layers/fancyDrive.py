from numpy import block
import lowLevel as ll
import moveLevel as ml
import fancyUltra as fU
from collections import deque

#global variables
last = 'C'
state = 0
count = 0
ONROAD = 'onRoad'
INTUNNEL = 'inTunnel'
BLOCKED = 'Blocked'
surroundings = ONROAD
#global obstable

blockStatus = False
exploreCount = 0

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

        self.isBlocked = False

        self.seenBefore = [False, False, False, False]
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


def justDrive(motor, ultraSensors):
    global last
    global state
    global count
    global long
    global lat
    global heading
    global intersections
    global lastintersection
    global surroundings
    global blockStatus

    while True:
        #read the sensors
        state = ll.readIRs() #a number 0 - 7
        
        # if obstacle is detected in front, mark street as BLOCKED
        # turning around is in fU
        #if fU.obstable == 2:
            #lastintersection.streets[heading - 2] = BLOCKED
        
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
            ll.time.sleep(0.5)
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
            motor.setvel(0, 0)
            
             #while either side sensor senses something, follow the wall
            while ll.readIRs() == 0:
                #run wallFollow
                fU.tunnelFollow(ultraSensors, motor)
                surroundings = INTUNNEL
                        
        else:
            motor.setvel(0,0)

    if not blockStatus:
        #update long and lat
        new_coords = shift(long, lat, heading)
        long = new_coords[0]
        lat = new_coords[1]
        
        curr_inter = intersection(long, lat)
        curr_inter.blockStatus = False
        lastintersection = curr_inter
        print(long, lat, heading)
        motor.setvel(0, 0)
        surroundings = ONROAD
        updateMap(ultraSensors)



def drive(motor, ultraSensors):
    global last
    global state
    global count
    global long
    global lat
    global heading
    global intersections
    global lastintersection
    global surroundings
    global blockStatus

    while True:
        #read the sensors
        state = ll.readIRs() #a number 0 - 7

        if fU.frontObstacle(ultraSensors):
            print("Oh no the intersection is blocked")
            blockStatus = True
            flipAround(motor, ultraSensors)
            break

        # if obstacle is detected in front, mark street as BLOCKED
        # turning around is in fU
        #if fU.obstable == 2:
            #lastintersection.streets[heading - 2] = BLOCKED
        
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
            motor.setvel(ml.straight, 0)
            if last == 'L' or last == 'R':
                last = 'C'
            count = 0
            
        elif state == 7:
            #on tape
            motor.setvel(ml.straight, 0)
            ll.time.sleep(0.35)
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
            motor.setvel(0, 0)
            
            #while either side sensor senses something, follow the wall
            while ll.readIRs() == 0:
                              
                #run wallFollow
                print('tunnel follow mode')
                fU.tunnelFollow(ultraSensors, motor)
                surroundings = INTUNNEL

            #if we've found somethings, we have to determine whether we're in the tunnel or on the road
            #drive a little further and peek to see if we're out of the tunnel
            fU.tunnelFollow(ultraSensors, motor)
            ll.time.sleep(0.35)
            motor.set(0,0)
            if ll.readIRs() != 7 and ll.readIRs() != 0:
                print("i am out of tunnel")
                surroundings = ONROAD
            else:
                print("i am in tunnel")
                surroundings = INTUNNEL
                motor.set(0,0)
                ll.time.sleep(0.5)
                break            
        else:
            motor.setvel(0,0)

    if not blockStatus:
        #update long and lat
        new_coords = shift(long, lat, heading)
        long = new_coords[0]
        lat = new_coords[1]
        print(long, lat, heading)
        #instantiate intersection if it doesn't exist
        prevHeading = heading
        try:
            curr_inter = Intersection(long, lat)
            prevHeading = heading
            if surroundings == ONROAD:
                print("new intersection on road")
                results = check(motor, ultraSensors)
                rel_dir = results[0]
                dist_list = results[1]
                
                for j in range(4):
                    if curr_inter.streets[j] == UNKNOWN:
                        if rel_dir[(j - prevHeading) %4] == True:
                            curr_inter.streets[j] = UNEXPLORED
                            if dist_list[(j - prevHeading) %4] == True:
                                #that means, the intersection exists, but the street is blocked
                                curr_inter.streets[j] = BLOCKED
                        else:
                            curr_inter.streets[j] = NOSTREET

            else:
                print("new intersection in tunnel")
                curr_inter.streets[(0 - prevHeading)] = UNEXPLORED
                curr_inter.streets[(1 - prevHeading)] = NOSTREET
                curr_inter.streets[(2 - prevHeading)] = UNEXPLORED
                curr_inter.streets[(3 - prevHeading)] = NOSTREET
                
            
        except:
            print("I've been here before!")
            curr_inter = intersection(long, lat)
            curr_inter.blockStatus = False
            #then we need to double check the intersection to see if there are
            #any new/old obstacles that need to be updated
            updateMap(ultraSensors)

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

        ll.time.sleep(0.3)
        print(curr_inter.streets)
        motor.setvel(0, 0)
    
    else:
        #we need to mark the intersection as blocked
        #instantiate intersection if it doesn't exist
        new_coords = shift(long, lat, (2 + heading) %4)
        new_long = new_coords[0]
        new_lat = new_coords[1]
        try:
            new_inter = Intersection(new_long, new_lat)
            #prevHeading = heading
            print('Intersection does not exist yet')
            print('Making new intersection')
            print('Marking this intersection as blocked')
            
            #mark streets appropriately
            #all of them are unknown to begin with except the street we just came down which is connected
            lastintersection = intersection(long, lat)
            lastintersection.streets[(heading + 2) %4] = CONNECTED
            new_inter.streets[(heading) %4] = CONNECTED
            #then mark the overall intersection as blocked
            new_inter.isBlocked = True
            print(curr_inter.streets)
        except:
            print('Intersection already exists')
            blk_inter = intersection(new_long, new_lat)
            print('Marking this intersection as blocked')
            #also mark the current heading as CONNECTED
            lastintersection = intersection(long, lat)
            lastintersection.streets[(heading + 2) %4] = CONNECTED
            blk_inter.streets[(heading) %4] = CONNECTED
            blk_inter.isBlocked =  True
            print(blk_inter.streets)

    #now reset the blockStatus
    blockStatus = False


def better_turn(spin, motor):
    global heading

    t0 = ll.time.time()
    t1 = ll.time.time()

    #right turns
    if spin == -1 or spin == 3:
        #if we're on the tape, get off the tape
        while ll.readIRs() != 0:
            #kickstart
            motor.set(1, -1)
            ll.time.sleep(0.050)
            motor.setvel(0, -1*ml.turnInPlace)
        motor.set(0,0)

        #kickstart
        motor.set(1, -1)
        ll.time.sleep(0.050)

        while ll.readIRs() == 0:
            motor.setvel(0, -1*ml.turnInPlace)
            t1 = ll.time.time()

        ll.time.sleep(0.05)
        motor.set(0, 0)
        ll.time.sleep(0.3)

    #left turns
    if spin == 1 or spin == -3:
        #if we're on the tape, get off the tape
        while ll.readIRs() != 0:
            #kickstart
            motor.set(-1, 1)
            ll.time.sleep(0.050)
            motor.setvel(0, ml.turnInPlace)
        motor.set(0,0)

        #kickstart
        motor.set(-1, 1)
        ll.time.sleep(0.050)
        while ll.readIRs() == 0:
            motor.setvel(0, ml.turnInPlace)
            t1 = ll.time.time()
        ll.time.sleep(0.05)
        motor.set(0, 0)
        ll.time.sleep(0.3)

    #180 deg turns, default turn to left
    if spin == 2 or spin == -2:
        #if left intersections don't exist, just go to the next intersection
        curr_inter = intersection(long, lat)
        
        if ll.readIRs() != 0 and (curr_inter == None or curr_inter.streets[(heading - 1)%4] == BLOCKED or curr_inter.streets[(1 + heading)%4] == NOSTREET):
            #if we're on the tape, get off the tape
            while ll.readIRs() != 0:
                #kickstart
                motor.set(-1, 1)
                ll.time.sleep(0.050)
                motor.setvel(0, ml.turnInPlace)
            motor.set(0,0)

            #kickstart
            motor.set(-1, 1)
            ll.time.sleep(0.050)
            while ll.readIRs() == 0:
                motor.setvel(0, ml.turnInPlace)
                t1 = ll.time.time()
            ll.time.sleep(0.05)
            motor.set(0, 0)
            ll.time.sleep(0.3)
            

        #else, take two turns
        else:
            #if we're on the tape, get off the tape
            while ll.readIRs() != 0:
                #kickstart
                motor.set(-1, 1)
                ll.time.sleep(0.050)
                motor.setvel(0, ml.turnInPlace)
            motor.set(0,0)

            #kickstart
            motor.set(-1, 1)
            ll.time.sleep(0.050)
            while ll.readIRs() == 0:
                motor.setvel(0, ml.turnInPlace)
                t1 = ll.time.time()
            ll.time.sleep(0.05)
            motor.set(0, 0)
            ll.time.sleep(0.3)

            #if we're on the tape, get off the tape
            while ll.readIRs() != 0:
                #kickstart
                motor.set(-1, 1)
                ll.time.sleep(0.050)
                motor.setvel(0, ml.turnInPlace)
            motor.set(0,0)

            #kickstart
            motor.set(-1, 1)
            ll.time.sleep(0.050)
            while ll.readIRs() == 0:
                motor.setvel(0, ml.turnInPlace)
                t1 = ll.time.time()
            ll.time.sleep(0.05)
            motor.set(0, 0)
            ll.time.sleep(0.3)

    heading = (heading + spin) %4

def TurnToNextStreet(direction, motor):
    #direction should be either 1 or -1

    global heading

    #if we're on the tape, get off the tape
    t0 = ll.time.time()
    motor.set(-1*direction, 1*direction)
    ll.time.sleep(0.040)
    while ll.readIRs() != 0:
        motor.setvel(0, ml.turnInPlace*direction)
    motor.set(0,0)

    t1 = ll.time.time()
    #turn until a street is detected

    #kickstart

    motor.set(-1*direction, 1*direction)
    ll.time.sleep(0.040)

    while ll.readIRs() == 0:
        motor.setvel(0, ml.turnInPlace*direction)
        t1 = ll.time.time()
    ll.time.sleep(0.05)
    motor.set(0, 0)
    ll.time.sleep(0.1)

    #categorize as 90 deg or 180 deg turn
    if t1 - t0 < 0.95:
        #heading = (heading + 1*direction) %4
        turn = 90
    else:
        #heading = (heading + 2*direction) %4
        turn = 180
    return turn

def check(motor, ultraSensors):
    global heading
    print("checking")

    check_list = ['forward', 'left', 'backward', 'right']
    dist_list = fU.wasObstacle(ultraSensors)

    #assume backward direction exists
    check_list[2] = True

    #check forward sensors
    ll.time.sleep(0.3)
    check_list[0] = (ll.readIRs() != 0)

    #check left side first
    outcome = TurnToNextStreet(1, motor)
    if outcome == 90:
        check_list[1] = True
        heading = (heading + 1) %4

        TurnToNextStreet(1, motor) #turn to the back line
        heading = (heading + 1) %4

        outcome = TurnToNextStreet(1, motor)
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

        outcome = TurnToNextStreet(1, motor)
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
    return [check_list, dist_list]

def isIntersectionExplored(intersection):
    isExplored = True
    for j in range(len(intersection.streets)):
        if intersection.streets[j] == UNEXPLORED:
            isExplored = False
            break
    return isExplored

def clearHeadings():
    global intersections

    for i in range(len(intersections)):
        intersections[i].headingToTarget = None

def unexplored():
    return [i for i in intersections if any([s==UNEXPLORED for s in i.streets])]

def victoryDance(motor):
    motor.set(0.9,0.9)
    ll.time.sleep(0.2)
    motor.set(-0.9, -0.9)
    ll.time.sleep(0.6)
    motor.set(0.9, 0.9)
    ll.time.sleep(0.3)
    motor.set(0,0)

def goToTarget(x, y, motor, allSensors):
    global intersections
    global long
    global lat
    global heading
    global lastintersection

    clearHeadings()
    print("All headings cleared..")
    #make a list of to-be-processed intersections
    toProcess = []
    currInter = intersection(x, y)
    if currInter != None:
        toProcess.append(currInter)
        
        while currInter.long != long or currInter.lat != lat:
            for i in range(4):
                #looks at connecting nodes and assigns headingToTarget
                if currInter.streets[i] == CONNECTED:
                    nextNode = shift(currInter.long, currInter.lat, i)
                    nextInter = intersection(nextNode[0], nextNode[1])
                    #print(nextNode)
                
                    if nextInter.headingToTarget == None:
                        nextInter.headingToTarget = (i + 2) %4
                        if nextInter not in toProcess:
                            toProcess.append(nextInter)
            
                                    
            currInter = toProcess.pop(0)

        #if it breaks out of the loop, we are ready to go to the Target 
        while long != x or lat != y:
            return_dir = lastintersection.headingToTarget - heading
            motor.setvel(0,0)
            better_turn(return_dir, motor)
            motor.setvel(0, 0)
            justDrive(motor, allSensors)
    else:
        print("Intersection does not exist")

def goToIntersection(x, y, motor, allSensors):
    global exploreCount
    print('finding a path to (' + str(x)+', ' + str(y)+')')
    clearHeadings()
    print("All headings cleared..")
    #make a list of to-be-processed intersections
    toProcess = []
    currInter = intersection(x, y)
    #try:
    if long == x and lat == y:
        print("i'm already there")
        ll.time.sleep(5)
    elif currInter != None:
        toProcess.append(currInter)
        while currInter.long != long or currInter.lat != lat:
            for i in range(4):
                #looks at connecting nodes and assigns headingToTarget
                if currInter.streets[i] == CONNECTED:
                    nextNode = shift(currInter.long, currInter.lat, i)
                    nextInter = intersection(nextNode[0], nextNode[1])
                    #print(nextNode)
                    if nextInter.headingToTarget == None:
                        nextInter.headingToTarget = (i + 2) %4
                        if nextInter not in toProcess:
                            toProcess.append(nextInter)
            
                                    
            currInter = toProcess.pop(0)

        #if it breaks out of the loop, we are ready to go to the Target 
        while long != x or lat != y:
            return_dir = lastintersection.headingToTarget - heading
            motor.setvel(0,0)
            better_turn(return_dir, motor)
            motor.setvel(0, 0)
            justDrive(motor, allSensors)
            break
    else:
        print("Intersection does not exist yet")
        if exploreCount > 100000:
            print("I'm sorry, we can't get to that intersection")
            print("Please choose another command")
            ll.time.sleep(5)
        else:

            exploreCount += 1
            #find nearest intersection
            minDist = 100
            minInter = None
            for inter in intersections:
                if inter.isBlocked != True:
                    manDist = manhattan(x, y, inter.long, inter.lat)
                    if manDist < minDist:
                        minDist = manDist
                        minInter = inter

            #now that we've found the nearest intersection go to it
            goToTarget(minInter.long, minInter.lat, motor, allSensors)
            #then face direction towards target
            currInter = intersection(long, lat)
            minDist = 1000
            minStreet = 0
            bestCoords = [0, 0]
            for i in range(4):
                possibleInter = shift(long, lat, (heading + i)%4)
                possibleLong = possibleInter[0]
                possibleLat = possibleInter[1]
                futureInter = intersection(possibleLong, possibleLat)

                if (currInter.streets[i] == CONNECTED or currInter.streets[i] == UNEXPLORED) and (futureInter == None or futureInter.isBlocked != True):
                    dist = manhattan(possibleLong, possibleLat, long, lat)
                    if dist < minDist:
                        #then this is the intersection to go to
                        minStreet = i
                        minDist = dist
                        bestCoords[0] = possibleLong
                        bestCoords[1] = possibleLat

            print('attempting to go to' + str(bestCoords[0]) + ', ' + str(bestCoords[1]))
            #turn towards correct street
            better_turn((minStreet - heading), motor)

            drive(motor, allSensors)

            #explore(motor, allSensors)




    #except:
    #    print("No possible path to the intersection")
    #    ll.time.sleep(5)



def explore(motor, allSensors):
    if lastintersection == None:
        drive(motor, allSensors)
        # if obstacle is detected in front, mark street as BLOCKED
        # turning around is in fU
        # this gives the error AttributeError: module 'fancyUltra' has no attribute 'obstable'
        '''if fU.isObstable(allSensors) == 2:
            lastintersection.streets[heading - 2] = BLOCKED
            print("Obstacle detected, turning around.")
            '''

    else:
        toExplore = unexplored()
        if toExplore:
            target = toExplore[-1]
            
            goToTarget(target.long, target.lat, motor, allSensors)
            
            #face unexplored street
            for i in range(4):
                #go down the unexplored street
                if target.streets[(i + heading) %4] == UNEXPLORED:
                    #only go down it if the next intersection isn't blocked:
                    new_coords = shift(long, lat, (i + heading) %4)
                    new_long = new_coords[0]
                    new_lat = new_coords[1]
                    if intersection(new_long, new_lat) == None or intersection(new_long, new_lat).isBlocked == False:
                        print('facing by turning:')
                        print(i)
                        better_turn(i, motor)
                        break
            #drive down it
            drive(motor, allSensors)
            # if obstacle is detected in front, mark street as BLOCKED

def home():
    global lastintersection
    global long
    global lat
    global heading

    lastintersection = intersection(0,0)
    long = 0
    lat = 0
    heading = NORTH

def flipAround(motor, ultraSensors):
    global long
    global lat
    #first robot needs to stop
    motor.set(0, 0)
    #then robot needs to turn 180 and find the line again
    old_coords = [long, lat]
    new_coords = shift(long, lat, heading)
    long = new_coords[0]
    lat = new_coords[1]
    better_turn(2, motor)
    long = old_coords[0]
    lat = old_coords[1]
    #then go back to the og intersection
    justDrive(motor, ultraSensors)

def updateMap(allSensors):
    #get recent ultra data
    print('updating map with new data')

    #updating blocked street data
    data = fU.wasObstacle(allSensors) #local position
    globalData = deque(data) #global reference frame
    globalData.rotate(heading)
    globalData = list(globalData)
    curr_inter = intersection(long, lat)
    for i in range(4):
        if curr_inter.streets[i] == BLOCKED and globalData[i] == False:
            print('marking blocked street as unblocked')
            curr_inter.streets[i] = UNEXPLORED
        elif (curr_inter.streets[i] == CONNECTED or curr_inter.streets[i] == UNEXPLORED) and globalData[i] == True:
            curr_inter.streets[i] = BLOCKED
            print('marking street as blocked')

    #updating blocked intersection data
    #if any of the streets at the current intersection lead to a an intersection, 

def manhattan(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)
        

