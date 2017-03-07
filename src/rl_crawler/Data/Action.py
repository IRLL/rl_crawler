


# action.py holds action representations for the curl bot

# abstract action interface:
#    get action id
#    get num actions
#    get raw action values

import random

from State import ServoState
import State



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
    DISPLACEMENT = 20

    def __init__(self, rawState=ServoState()):
        self.rawState = rawState
        self.nearServo = 0
        self.farServo = 0

    def getActionId(self):
        weighty = (self.nearServo + 1) * 3
        light = self.farServo + 1
        return weighty + light

    @classmethod
    def IdToAction(self, Id, servoState):
        weighty = (Id // 3) - 1
        light = (Id % 3) - 1

        displacementAction = SimpleDisplacementAction(servoState)
        displacementAction.nearServo = weighty
        displacementAction.farServo = light

        return displacementAction
    
    @classmethod
    def getRandomAction(self, state):
        action = SimpleDisplacementAction(state)
        action.nearServo = random.randint(-1, 1)
        action.farServo = random.randint(-1, 1)

        return action
        
    
    @classmethod
    def getNumActions(self):
        # 3 for number of possible actions that the near servo can take
        # 3 for the number of possible actions that the far servo can take
        return 3 * 3

    @classmethod
    def getPossibleActions(self):
        pass

    def convertToServoState(self):
        state = ServoState()
        state.nearServo = self.rawState.nearServo
        state.farServo = self.rawState.farServo

        state.nearServo += self.nearServo * self.DISPLACEMENT
        state.farServo += self.farServo * self.DISPLACEMENT

        return state


"""
class SingleMovementAction(object):
    DISPLACEMENT = 20

    def __init__(self, rawState=SevoState()):
        pass

    def getActionId(self):
        pass

    @classmethod
    def IdToAction(self, Id, servoState):
        pass

    @classmethod
    def getRandomAction(self, state):
        pass

    @classmethod
    def getNumActions(self):
        return 4

    def convertToServoState(self):
        pass

"""


# TODO add simplified action class (IE one that just moves each servo to far, middle, or close for each servo)





