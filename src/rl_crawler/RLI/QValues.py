
"""
	File for holding classes related to storing and updating Q-values

"""

from Data.State import RawState as State
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

		for i in range n:
			# create state array of actions
			self.qValues.append([])

			# populate with default values
			for j in range m:
				self.qValues[i].append(initValue)

	# update given q value with new information
	def updateQValue(self, state, action, reward, rState, alpha=0.1, gamma=1):
		# find max of all possible actions
		_, maxValue = self._getMaxActionValue(state)

		# get previous q
		previousQ = self._getQValue(state, action)

		self._setQValue(state, action, previousQ + alpha * (reward + gamma*maxValue - previousQ))


	# retrieve the action to take given the current state
	def getPiAction(self, state):
		maxAction, _ = self._getMaxActionValue(state)
		return maxAction

	# finds the maximum value action and returns its value and the action itself
	def _getMaxActionValue(self, state):
		actionList = Action.getPossibleActions()

		maxAction = actionList[0]
		maxValue = self._getQValue(state, maxAction)

		for action in actionList:
			currentValue = self._getQValue(state, action)
			if currentValue > maxValue:
				maxAction = action
				maxValue = currentValue

		return maxAction, maxValue


	def _getQValue(self, state, action):
		stateID = state.getStateID()
		actionID = action.getActionID()
		return self.qValues[stateID][actionID]

	def _setQValue(self, state, action, value):
		stateID = state.getStateID()
		actionID = action.getActionID()

		self.qValues[stateID][actionID] = value


