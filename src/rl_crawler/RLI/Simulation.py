


from Agent import CurlBotAgent as Agent
from Environment import CurlBotEnvironment as Environment

from Data.State import RawState as State
from Data.Action import SimpleDisplacementAction as Action



# Manages the interaction between the agent and the environment.
class Simulation(object):
	# init the whole simulation
	def __init__(self, arguments):
		self.agent = Agent(arguments)
		self.environment = Environment(arguments)

	# starts the beginning of a new trial.
	def StartTrial(self):
		self.currentState = self.environment.StartTrial()
		self.currentAction = self.agent.StartTrial(self.currentState)

	# run the simulation for numSteps steps
	def Steps(self, numSteps):
		for i in range(numSteps):
			# push back action and state information
			self.prevState = self.currentState
			self.prevAction = self.currentAction

			# get new state and reward information
			environmentStepResult = self.environment.Step(self.prevAction)
			self.currentState = environmentStepResult.sensation
			self.currentReward = environmentStepResult.reward

			# collect performance data
			self.CollectData(self.prevState, self.prevAction, self.currentReward, self.currentState)

			# update agent and get new action information
			self.currentAction = self.agent.Step(self.prevState, self.prevAction, self.currentReward, self.currentState)

			# TODO add checks for terminal states (don't care for curlbot)

	# run the simulation for the given number of trials
	def Trials(self, numTrials, maxStepsPerTrial):
		# run numTrials trials
		for i in range(numTrials):
			self.StartTrial()
			Steps(maxStepsPerTrial)

	# Collect data about the performance of the algorithm
	def CollectData(self, sensation, action, reward, nextSensation):
		# TODO write something cool for this.
		pass

