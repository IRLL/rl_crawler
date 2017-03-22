
"""
    Agent.py
    
    Abstract interface for all agents.
    All agents are derived from agent
"""

import random

from rl_crawler.RLI.QValues import DictionaryQValues as QValues


import Data.State
import Data.Action


class CurlBotAgent(object):
    # intialize our curl bot agent
    def __init__(self, arguments = ""):
        # initialize QValues
        self.QValues = QValues(0.1)
        self.explorationRate = 0.1
        self.learningRate = 0.8
        self.gamma = 0.9

    def saveAgent(self):
        # save QValues
        self.QValues.saveQValues("savedQValues")

        # save agent
        saveFile = open("agentState", 'w')

        saveFile.write(str(self.explorationRate) + "\n")
        saveFile.write(str(self.learningRate) + "\n")

        saveFile.close()

    def loadAgent(self):
        # load QValues
        self.QValues.loadQValues("savedQValues")

        # load agent
        loadFile = open("agentState", 'r')

        self.explorationRate = float(loadFile.readline())
        self.learningRate = float(loadFile.readline())

        loadFile.close()
        

    # set the agent to the first sensation in the trial.
    def StartTrial(self, sensation):
        return self.QValues.getPiAction(sensation)

    # take a simulated step in the simulation
    def Step(self, prevState, prevAction, reward, currentState):

        # update Q values
        self.QValues.updateQValue(prevState, prevAction, reward, currentState, self.learningRate, self.gamma)

        # update learning rate
        self.learningRate /= 1.0003

        # check for random action
        randNum = random.random()
        self.explorationRate -= 0.0002
        if self.explorationRate < 0.1: self.explorationRate = 0.1

        if randNum < self.explorationRate:
            # take random action
            print(self.explorationRate)
            return Data.Action.SimpleDisplacementAction.getRandomAction(currentState.convertToServoState())
        else:
            # take greedy action
            action = self.QValues.getPiAction(currentState)
            print("learning rate: ", self.learningRate, "  qValue: ", self.QValues._getQValue(currentState, action))
            return action
    
    # return the greedy action for the current timestep
    def getAction(self, currentSensation):
        state = Data.State.RawState(currentSensation)
        action = self.QValues.getPiAction(state)
        return action



       


