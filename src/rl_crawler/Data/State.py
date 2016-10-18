

# state.py holds state representations for the Curl bot

# abstract state interface:
#   get state id
#   get num states
#   get raw state values
# 
#   states must be able to initialize themselves from ServoState


# number of ticks between the smallest and largest servo positions for one servo
NUM_SERVO_TICKS = 255


# represents raw servo tick values
class ServoState(object):
    def __init__(self, near=0, far=0):
        self.nearServo = near
        self.farServo = far



class State(object):
    def __init__(self, servoState):
        pass

    # get the unique id of the state
    def getStateId(self):
        pass

    # get the number of unique states
    def getNumStates(self):
        pass

    # convert state to ServoState object
    def convertToServoState(self):
        pass


class RawState(State):
    def __init__(self, servoState = ServoState()):
        self.nearServo = servoState.nearServo
        self.farServo = servoState.farServo

    def getStateId(self):
        weighty = self.nearServo * NUM_SERVO_TICKS
        light = self.farServo

        return weighty + light

    def getNumStates(self):
        return NUM_SERVO_TICKS * NUM_SERVO_TICKS

    def convertToServoState(self):
        return ServoState(self.nearServo, self.farServo)
        
# TODO add simplified state class (IE one that just has far, middle, or close for each servo

    










