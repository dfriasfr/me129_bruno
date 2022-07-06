import lowLevel as ll
turnInPlace = 17
straight = 50

def softTurn(dir, motor):
    motor.setvel(45, dir*0.45)

def spinTurn(dir, motor):
    motor.setvel(0, dir*turnInPlace)


def driveStraight(dir, motor):
    motor.setvel(dir*straight, 0)

def stopMotor(motor):
    motor.setvel(0,0)