#
import lowLevel as ll
          
import moveLevel as ml

import fancyDrive as fD

import fancyUltra as fU

import pickle

import threading

import ctypes


#initialize objects
motor = ll.Motor(ll.IO)

ULTRA1 = ll.Ultrasonic(ll.ULTRA1_TRIGGER, ll.ULTRA1_ECHO, ll.IO)
ULTRA2 = ll.Ultrasonic(ll.ULTRA2_TRIGGER, ll.ULTRA2_ECHO, ll.IO)
ULTRA3 = ll.Ultrasonic(ll.ULTRA3_TRIGGER, ll.ULTRA3_ECHO, ll.IO)
sensors = [ULTRA1, ULTRA2, ULTRA3]

#############################
#TRIGGERING
#############################
#global variables for ultrasonic sensors
triggering_stopflag = False
driving_stopflag = False
pausedriving = True

#multithreading functions
def triggering_stop():
    global triggering_stopflag
    triggering_stopflag = True

def triggering_loop():
    global triggering_stopflag
    global sensors

    triggering_stopflag = False
    while not triggering_stopflag:
        #trigger the pins
        for i in sensors:
            i.trigger_pins()
        ll.time.sleep(0.08 + 0.04 * ll.random.random())

def driving_stop():
    global driving_stopflag
    driving_stopflag = True

target_long = 0
target_lat = 0

EXPLORE = 'explore'
GOTO = 'goto'
STOP = 'stop'

state = EXPLORE


def driving_loop():
    global driving_stopflag
    global sensors
    global state

    driving_stopflag = False
    while not driving_stopflag:
        # Pause at this intersection, if requested
        if pausedriving:
            #fD.justDrive(motor)
            continue
        # Move from this intersection to the next intersection
        else:
            if state == EXPLORE:
                fD.explore(motor, sensors)
            elif state == GOTO:
                fD.goToIntersection(target_long, target_lat, motor, sensors)
                #state = STOP
            elif state == STOP:
                ml.stopMotor(motor)

def userinput():
    global pausedriving
    global target_long
    global target_lat
    global state

    while True:

        # Grab a command
        command = input("Command? ")

        # Compare against possible commands.
        if (command == 'pause'):
            print("Pausing at the next intersection")
            state = STOP
            pausedriving = True

        elif (command == 'explore'):
            print("Exploring without a target")
            state = EXPLORE
            pausedriving = False

        elif (command == 'goto'):
            fD.exploreCount = 0
            print("Driving to a target")
            target_command_x = input("x coordinate? ")
            target_command_y = input("y coordinate? ")
            target_long = int(target_command_x)
            target_lat = int(target_command_y)
            if target_long != fD.long or target_lat != fD.lat:
                state = GOTO
                pausedriving = False
            else:
                print("you're already there!")

        elif (command == 'print'):
            print(fD.intersections)
            #... and/or useful debug values?

        elif (command == 'save'):
            print("Saving the map...")
            with open('map.pickle', 'wb') as file:
                pickle.dump(fD.intersections, file)

        elif (command == 'load'):
            print("Loading the map...")
            with open('map.pickle', 'rb') as file:
                fD.intersections = pickle.load(file)

        elif (command == 'quit'):
            print("Quitting...")
            break

        elif (command == 'home'):
            # set the current intersection as (0, 0)
            print("Restarting the robot at the home position")
            fD.home()
            pausedriving = True

        elif (command == 'distances'):
            #return distances
            #print(fU.getDistances(sensors))
            print(fU.wasObstacle(sensors))
        elif (command == 'heading'):
            print(fD.heading)
        elif (command == 'position'):
            print(fD.long, fD.lat)
        else:
            print("Unknown command: '%s'" % command)

#
#   Main
#
if __name__ == "__main__":

    ############################################################
    

    ############################################################
    


    #start new thread
    triggering_thread = ll.threading.Thread(target=triggering_loop, name='triggering')
    triggering_thread.start()

    driving_thread = ll.threading.Thread(target=driving_loop, name='driving')
    driving_thread.start()
    ll.time.sleep(0.5)

    #run the main code in the exception handler
    try:
        userinput()
        #while True:
            #fU.tunnelFollow(sensors, motor)
            #print(sensors[0].distance, sensors[1].distance, sensors[2].distance)
            #ll.time.sleep(1)
        #fD.justDrive(motor, sensors)
        #fD.check(motor)

    except KeyboardInterrupt:
            #wait for the triggering thread to be done (re-joined)
            print('keyboard interrupt')
            ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(triggering_thread.ident),ctypes.py_object(KeyboardInterrupt))
            ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(driving_thread.ident),ctypes.py_object(KeyboardInterrupt))
    except BaseException as ex:
        print("Ending due to exception: %s" %repr(ex))

        ll.traceback.print_exc()

    triggering_stop()
    triggering_thread.join()

    driving_stop()
    driving_thread.join()

    
    #shutdown sensors
    for i in sensors:
        i.cbr.cancel()
        i.cbf.cancel()

    motor.set(0, 0)        
    motor.shutdown()


    for thread in threading.enumerate():
        print(thread.name)

    




