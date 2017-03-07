
"""
    File for holding classes related to storing and updating Q-values

"""

import random

from Data.State import DoubleState as State
from Data.Action import SimpleDisplacementAction as Action

# Abstract QValues class
class QValues(object):
    # update q value with new information
    def updateQValue(self, state, action, reward, rState):
        pass

    # retrieve the action to take given the current state
    def getPiAction(self, state):
        pass


class TabularQValues(QValues):
    def __init__(self, initValue=0):
        self.qValues = []
        n = State.getNumStates()
        m = Action.getNumActions()

        for i in range(n):
            # create state array of actions
            self.qValues.append([])

            # populate with default values
            for j in range(m):
                self.qValues[i].append(initValue)

    # update given q value with new information
    def updateQValue(self, state, action, reward, rState, alpha=0.1, gamma=1):
        # find max of all possible actions
        _, maxValue = self._getMaxActionValue(rState)

        # get previous q
        previousQ = self._getQValue(state, action.getActionId())

        self._setQValue(state, action, (1 - alpha)*previousQ + alpha * (reward + gamma*maxValue))


    # retrieve the action to take given the current state
    def getPiAction(self, state):
        maxActionId, _ = self._getMaxActionValue(state)
        maxAction = Action.IdToAction(maxActionId, state.convertToServoState())
        return maxAction

    def saveQValues(self, filepath):
        saveFile = open(filepath, 'w')

        n = State.getNumStates()
        m = Action.getNumActions()

        for i in range(n):
            for j in range(m):
                saveFile.write(str(self.qValues[i][j]) + "\n")

        saveFile.close()

    def loadQValues(self, filepath):
        loadFile = open(filepath, 'r')

        n = State.getNumStates()
        m = Action.getNumActions()

        for i in range(n):
            for j in range(m):
                self.qValues[i][j] = float(loadFile.readline())

        loadFile.close()

    # finds the maximum value action and returns its value and the action itself
    def _getMaxActionValue(self, state):
        numActions = Action.getNumActions()
        stateTable = self.qValues[state.getStateId()]

        maxActionId = 0
        maxValue = self._getQValue(state, maxActionId)

        for actionId in range(numActions):
            currentValue = self._getQValue(state, actionId)
            if currentValue > maxValue:
                maxActionId = actionId
                maxValue = currentValue

        return maxActionId, maxValue


    def _getQValue(self, state, action):
        actionID = action.getActionId()
        stateID = state.getStateId()
        return self.qValues[stateID][actionId]

    def _setQValue(self, state, action, value):
        stateID = state.getStateId()
        actionID = action.getActionId()

        self.qValues[stateID][actionID] = value


class DictionaryQValues(QValues):
    def __init__(self, initValue=0):
        self.initValue = initValue
        self.qValues = dict()


    # update given q value with new information
    def updateQValue(self, state, action, reward, rState, alpha=0.1, gamma=1):
        # find max of all possible actions
        _, maxValue = self._getMaxActionValue(rState)

        # get previous q
        previousQ = self._getQValue(state, action)

        self._setQValue(state, action, (1-alpha) * previousQ + alpha * (reward + gamma*maxValue))

    # retrieve the action to take given the current state
    def getPiAction(self, state):
        maxActionId, _ = self._getMaxActionValue(state)
        maxAction = Action.IdToAction(maxActionId, state.convertToServoState())
        return maxAction

    def saveQValues(self, filepath):
        # open the fiel for write
        savefile = open(filepath, 'w')

        # save key, value pairs from the dictionary
        for key, value in self.qValues.iteritems():
            if key in self.qValues:
                savefile.write(str(key) + "," + str(value) + "\n")

        savefile.close()

    def loadQValues(self, filepath):
        with open(filepath, 'r') as loadfile:
            self.qValues = dict()

            for line in loadfile:
                if line != '\n':
                    # split the line into key and value
                    split = line.split(',', 2)
                    key = float(split[0])
                    value = float(split[1])

                    # load into our qValues
                    self.qValues[key] = value

    # finds the maximum value action and returns its value and the action itself
    def _getMaxActionValue(self, state):
        maxActionId = 0
        idAction = Action.IdToAction(maxActionId, state)
        maxValue = self._getQValue(state, idAction)
        
        for i in range(1, Action.getNumActions()):
            idAction = Action.IdToAction(i, state)
            currentValue = self._getQValue(state, idAction)

            if currentValue > maxValue:
                maxValue = currentValue
                maxActionId = i

        return maxActionId, maxValue

    def _getQValue(self, state, action):
        qValueId = self._getQValueId(state, action)

        if qValueId in self.qValues:
            return self.qValues[qValueId]
        else:
            return self.initValue

    def _setQValue(self, state, action, value):
        qValueId = self._getQValueId(state, action)
        self.qValues[qValueId] = value

    def _getQValueId(self, state, action):
        stateId = state.getStateId()
        actionId = action.getActionId()
        qValueId = stateId * action.getNumActions() + actionId
        return qValueId




