


# action.py holds action representations for the curl bot

# abstract action interface:
#    get action id
#    get num actions
#    get raw action values

from State import ServoState


# represents position to move to for both servos
class ServoAction(object):
    def __init__(self, near=0, far=0):
        self.nearServo = near
        self.farServo = far


# abstract action representation
class Action(object):
    def __init__(self, rawState):
        pass

    # get the unique id of the action
    def getActionId(self):
        pass

    # get the number of unique action
    def getNumActions():
        pass

	# get the list of possible actions
	def getPossibleActions():
		pass

    # convert state to ServoAction object
    def convertToServoAction(self):
        pass


# action representing a short distance displacement change
class SimpleDisplacementAction(object):
    DISPLACEMENT = 5

    def __init__(self, rawState):
        self.rawState = rawState
        self.nearServo = 0
        self.farServo = 0

    def getActionId(self):
        weighty = (self.nearServo + 1) * 3
        light = self.farServo + 1

        return weighty + light

    def getNumActions(self):
        # 3 for number of possible actions that the near servo can take
        # 3 for the number of possible actions that the far servo can take
        return 3 * 3

    def convertToServoAction(self):
        servoAction = ServoAction()
        servoAction.nearServo = rawState.nearServo
        servoAction.farServo = rawState.farServo

        servoAction.nearServo += self.nearServo * DISPLACEMENT
        servoAction.farServo += self.farServo * DISPLACEMENT

        return servoAction

# TODO add simplified action class (IE one that just moves each servo to far, middle, or close for each servo)





