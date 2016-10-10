

"""
	Class representing the interface between the curl bot and its environment.
		Gets rewards from the sensor onboard the curl bot (R)
		Sends commands to the servos on the curl bot (A)
		Gets state information from the servos on the curl bot (S, S')
"""
class CurlBotEnvironment(object):
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
		# tell the curl bot to go back to it's starting position
		pass

	"""
		performs initialization of the environment for the beginning of a new trial. 
		input: NA
		output:
			sensation - the first sensation of the trial
	"""
	@classmethod
	def StartTrial(self):
		# get the first set of state information from the robot
		pass
	
	"""
		performs a single step of the environment simulation.
		Input:
			Action the agent wishes to take
		Output:
			StepResult object containing:
				Reward the agent received
				Resulting state 
			
	"""
	def Step(self, action):
		# send action to robot

		# get next state from robot
		pass
