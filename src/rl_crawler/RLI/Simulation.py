


import time
import sys
import termios
import fcntl
import os

import Agent
import Environment


# setup terminal nonblocking
fd = sys.stdin.fileno()

oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)

oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)


# Manages the interaction between the agent and the environment.
class Simulation(object):
    # init the whole simulation
    def __init__(self, arguments = ""):
        self.agent = Agent.CurlBotAgent(arguments)
        self.environment = Environment.CurlBotEnvironment(arguments)


        # let user decide whether to reload or not
        while 1:
            try:
                c = sys.stdin.read(1)
                if c != 'n':
                    # reload agent
                    self.agent.loadAgent()
                break
            except IOError: pass

        # result information
        self.totalTimesteps = 0
        self.currentTrial = 0
        self.currentTrackingReward = 0
        self.needsReset = False


        # reload results from the previous session and keep writing
        self.trainingResultsFile = open('results', 'a')
        self.resultsFile = open('../results.txt', 'w')



    # starts the beginning of a new trial.
    def StartTrial(self):
        self.trialSteps = 0
        self.currentTrial += 1

        # wait for user to give the ok if we need a reset
        if self.needsReset == True:
            while 1:
                try:
                    c = sys.stdin.read(1)
                    break
                except IOError: pass

        self.currentState = self.environment.StartTrial()
        self.currentAction = self.agent.StartTrial(self.currentState)



    # run the simulation for numSteps stepstotalRew
    def Steps(self, numSteps, trainingMode=True):

        for i in range(numSteps):
            # get user commands if any
            try:
                c = sys.stdin.read(1)
                self.handleUserCommands(c)
            except IOError: pass

            # push back action and state information
            self.prevState = self.currentState
            self.prevAction = self.currentAction

            # get new state and reward information
            environmentStepResult, isTerminal, self.needsReset = self.environment.Step(self.prevAction)
            self.currentState = environmentStepResult.sensation
            self.currentReward = environmentStepResult.reward

            # update agent and get new action information
            if trainingMode == True:
                self.currentAction = self.agent.Step(self.prevState, self.prevAction, self.currentReward, self.currentState)
                self.totalTimesteps += 1
            else:
                self.currentAction = self.agent.getAction(self.currentState)

            # update tracking information
            self.currentTrackingReward += self.currentReward
            self.trialSteps += 1

            # collect data
            if self.totalTimesteps % 100 == 0:
                self.CollectData(self.currentTrackingReward, 100)
                self.currentTrackingReward = 0

            if isTerminal == True:
                break

    def handleUserCommands(self, characterCommand):
        if characterCommand == 's':
            # save agent
            self.agent.saveAgent()

        elif characterCommand == 'l':
            # load agent
            self.agent.loadAgent()

        elif characterCommand == '!':
            # quit without saving q values or results
            # BUG: probably does not actually stop saving results
            sys.exit(0)

        elif characterCommand == 'q':
            # save results to file
            self.trainingResultsFile.close()
            self.resultsFile.close()

            # save agent
            self.agent.saveAgent()

            # reset to old terminal flags
            termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
            fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

            # quit
            sys.exit(0)

        elif characterCommand == 'n':
            # make new qvalues
            self.agent = Agent.CurlBotAgent()
            self.trainingResultsFile = open('../results/current_Results', 'w')

        elif characterCommand == 'p':
            # pause for a while
            while 1:
                try:
                    sys.stdin.read(1)
                    break
                except IOError: pass
        elif characterCommand == 'r':
            # start a new trial
            self.StartTrial()


    # run the simulation for the given number of trials
    def Trials(self, numTrials, maxStepsPerTrial):
        # run numTrials trials
        for i in range(numTrials):
            self.StartTrial()

            if i % 10 != 0 or i == 0 or True:
                self.Steps(maxStepsPerTrial, True)
            else:
                # every 10th trial do an evaluation run.
                self.Steps(maxStepsPerTrial, False)
                self.CollectData(self.totalTrialReward, self.trialSteps, False)
            print("finished trial: ", i)

    # Collect data about the performance of the algorithm
    def CollectData(self, trialReward, trialSteps, trainingMode = True):
        # get reward per timestep
        rewardPerTimestep = trialReward / trialSteps

        # print out to file
        if trainingMode == True:
            self.trainingResultsFile.write(str(self.totalTimesteps) + ",  " + str(rewardPerTimestep) + '\n')
        else:
            self.resultsFile.write(str(self.totalTimesteps) +  ',' + str(rewardPerTimestep) + '\n')
