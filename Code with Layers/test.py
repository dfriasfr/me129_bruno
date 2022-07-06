#
import lowLevel as ll
          
import moveLevel as ml

#
#   Main
#
if __name__ == "__main__":

    ############################################################
    

    ############################################################

    motor = ll.Motor()

    #run the main code in the exception handler
    try:
        ml.justDrive(motor)
    except BaseException as ex:
        print("Ending due to exception: %s" %repr(ex))

        ll.traceback.print_exc()

    #shutdown motors
    motor.set(0, 0)        
    motor.shutdown()

    #wait for the triggering thread to be done (re-joined)
    ll.stopcontinual()
    ll.thread.join()
