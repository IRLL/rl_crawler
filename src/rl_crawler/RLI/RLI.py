
"""

	Standard Reinforcement learning interface

	This file provides interfaces for the proposed standard RL interface by Richard Sutton.

"""





"""
	class Agent
	
	Abstract interface for all agents.
	All agents are derived from agent.
	
	Agents are the entities that take actions in the environment and learn from the past in RL systems.
"""
class Agent(object):
	"""
		Initializes the agent to its starting state
		called once when the simulation is first started.
	"""
	def __init__(self, arguments):
		pass

	"""
		Prepares the agent for the start of a new trial and returns the action taken from the first sensation (State)
		input:
			sensation -  the first state the agent sees at the beginning of the current trial
		output:
			action - action the agent wishes to take 
	"""
	@classmethod
	def StartTrial(self, sensation):
		pass

	"""
		Responds to a simulator step.
		input:
			prevSensation - sensation from the last timestep
			prevAction - action taken in the last timestep
			reward - reward gained from the last timestep
			currentSensation - sensation in the current timestep
		output:
			action - action the agent wants to take in the current timestep
	"""
	@classmethod
	def Step(self, prevSensation, prevAction, reward, currentSensation):
		pass

"""
	class Environment

	abstract class for all environments.
	The environment for a problem basically defines the problem to be solved.
"""
class Environment(object):
	class StepResult(object):
		def __init__(self):
			self.reward = 0
			self.sensation = 0

	"""
		initializes the environment.
		If the environment changes with experience this should reset it to its original condition.
		Normally only called when the simulation is first initialized.
	"""
	def __init__(self, arguments):
		pass

	"""
		performs initialization of the environment for the beginning of a new trial. 
		input: NA
		output:
			sensation - the first sensation of the trial
	"""
	@classmethod
	def StartTrial(self):
		pass
	
	"""
		performs a single step of the environment simulation.
		Input:
			Action the agent wishes to take
		Output:
			StepResult object containing the reward the agent received and the next sensation.
			
	"""
	def Step(self, action):
		pass


"""
	Simulation manages the interaction between the agent and the environment
	User should only need to provide the implementation for CollectData and modify to allow for reporting.

"""
class Simulation(object):
	# init the whole simulation
	def __init__(self, arguments):
		pass

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



