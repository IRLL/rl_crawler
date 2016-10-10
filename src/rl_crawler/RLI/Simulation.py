


from Agent import CurlBotAgent as Agent
from Environment import CurlBotEnvironment as Environment



# Manages the interaction between the agent and the environment.
class Simulation(object):
	# init the whole simulation
	def __init__(self, arguments):
		self.agent = Agent(arguments)
		self.environment = Environment(arguments)

	# starts the beginning of a new trial.
	def StartTrial(self):
		pass

	# run the simulation for numSteps steps
	def Steps(self, numSteps):
		pass

	# run the simulation for the given number of trials
	def Trials(self, numTrials, maxStepsPerTrial):
		pass
	
	# return data to the user for report collection
	def CollectData(self, sensation, action, reward, nextSensation):
		pass

