
"""
	Agent.py
	
	Abstract interface for all agents.
	All agents are derived from agent
"""

from RLI.RLI import Agent



class CurlBotAgent(Agent):
	# intialize our curl bot agent
	def __init__(self, arguments = ""):
		pass

	# set the agent to the first sensation in the trial.
	@classmethod
	def StartTrial(self, sensation):
		pass

	# take a simulated step in the simulation
	@classmethod
	def Step(self, prevSensation, prevAction, reward, currentSensation):
		pass

