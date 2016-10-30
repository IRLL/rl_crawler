
import time

from RobotInterface import RobotInterface

from Data.State import RawState as State
from Data.Action import SimpleDisplacementAction as Action



"""
	Class representing the interface between the curl bot and its environment.
		Gets rewards from the sensor onboard the curl bot (R)
		Sends commands to the servos on the curl bot (A)
		Gets state information from the servos on the curl bot (S, S')
"""
class CurlBotEnvironment(object):
	class StepResult(object):
		def __init__(self, reward = 0, sensation = State()):
			self.reward = reward
			self.sensation = sensation

	"""
		initializes the environment.
		If the environment changes with experience this should reset it to its original condition.
		Normally only called when the simulation is first initialized.
	"""
	def __init__(self, arguments):
		# initialize robot
		self.robot = RobotInterface()

		self.currentState = 0


	"""
		performs initialization of the environment for the beginning of a new trial. 
		input: NA
		output:
			sensation - the first sensation of the trial
	"""
	@classmethod
	def StartTrial(self):
		# reset the robot
		self.robot.reInitialize()
	
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
		servoAction = action.convertToServoAction()
		self.robot.setServoState(servoAction)

		# wait a bit
		time.sleep(0.5)

		# get next state and reward from robot
		nextState = self.robot.getCurrentState()
		nextReward = self.robot.getNextReward()

		stepResult = StepResult(nextReward, nextState)

		return stepResult


