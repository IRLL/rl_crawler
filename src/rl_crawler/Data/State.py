

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
	
	# get the list of features that make up the state
	def getStateFeatures(self):
		pass

    # get the number of unique states
	# -1 is infinity
	@staticmethod
	def getNumStates():
		pass

    # convert state to ServoState object
    def convertToServoState(self):
        pass


class RawState(object):
	def __init__(self, servoState = ServoState()):
		self.nearServo = servoState.nearServo
		self.farServo = servoState.farServo
	
	def getStateId(self):
		weighty = self.nearServo * NUM_SERVO_TICKS
		light = self.farServo
		return weighty + light

	def getstatefeatures(self):
		# todo output state feature list
		pass
		
	@staticmethod
	def getNumStates():
		return NUM_SERVO_TICKS * NUM_SERVO_TICKS

	def convertToServoState(self):
		return ServoState(self.nearServo, self.farServo)

class DoubleState(object):
    def __init__(self, servoState1 = ServoState(), servoState2 = ServoState()):
        self.state1 = RawState(servoState1)
        self.state2 = RawState(servoState2)

    def getStateId(self):
        multiplier = RawState.getNumStates()

        weighty = self.state1.getStateId * multiplier
        light = self.state2.getStateId

        return weighty + light

    @staticmethod
    def getNumStates():
        multiplier = RawState.getNumStates()
        return multiplier * multiplier

    def convertToServoState(self):
        return self.state2.convertToServoState()

        
# TODO add simplified state class (IE one that just has far, middle, or close for each servo









