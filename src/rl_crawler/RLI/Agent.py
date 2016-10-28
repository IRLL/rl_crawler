
"""
	Agent.py
	
	Abstract interface for all agents.
	All agents are derived from agent
"""

from RLI.RLI import Agent
from RLI.QValues import TabularQValues as QValues



class CurlBotAgent(Agent):
	# intialize our curl bot agent
	def __init__(self, arguments = ""):
		# initialize QValues
		self.QValues = QValues()

	# set the agent to the first sensation in the trial.
	@classmethod
	def StartTrial(self, sensation):
		return self.QValues.getPiAction(sensation);

	# take a simulated step in the simulation
	@classmethod
	def Step(self, prevSensation, prevAction, reward, currentSensation):
		# update Q values
		self.QValues.updateQValues(prevSensation, prevAction, reward, currentSensation);

		# return desired action
		return self.QValues.getPiAction(currentSensation);

