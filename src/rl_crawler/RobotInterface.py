



# robot interface will provide the interface between the learning simulation and the real world robot.
# 
# interface:
#   get current state
#   set state
#   get reward
#
#   whenever the internal __state variable is changed that change should be sent to the servos.
#   in other words __state must always reflect the actual state of the robots servos
#
#   robot interface must be started after other ross nodes!
#   
#
# private functions:
#   get ultrasonic position
#   post desired state


import rospy
from rl_crawler.msg import distanceMsg as distance_msg
from rl_crawler.msg import actionMsg as action_msg


from Data.State import ServoState
from Data.Action import ServoAction

from Data.State import RawState as State
from Data.Action import SimpleDisplacementAction as Action


class RobotInterface(object):
    # initialize robot interface
    def __init__(self):
        # initialize internal state
        startingServoState = ServoState(0, 0)
        self.__state = State(startingServoState)

        # initialize starting distance
        self.distanceInitialzed = False

        # setup ross publishing and subscription
        self.sensor_subscriber = rospy.Subscriber('distance', distance_msg, self.updateDistance)
        self.action_publisher = rospy.Publisher('command', action_msg, queue_size=1)

        # set both servos to zero position
        startingActionMessage = action_msg(startingServoState.convertToServoAction())
        self.action_publisher.publish(startingActionMessage)
        # TODO ensure that action message can actually take a servoAction as a constructor argument


    # get current state
    def getCurrentState(self):
        # output last state given to the servos
        return self.__state

    # returns the change in distance from the wall since the last call to getNextReward
    def getNextReward(self):
        lastDistance = self.__lastDistance
        currentDistance = __getUltrasonicPosition()
        deltaDistance = currentDistance - lastDistance

        self.__lastDistance = currentDistance

        # TODO add some kind of softener or deadband to remove the noise from the ultrasonic sensor
        return deltaDistance


    ### private functions

    # returns a double of the position in inches from the wall as measured by the ultrasonic sensor on the curl bot
    def __getUltrasonicPosition(self):
        return self.currentDistance

    # tells the robot to move its servos to the given position
    def __postDesiredState(self, servoAction):
        actionMessage = action_msg(servoAction)
        self.action_publisher.publish(actionMessage)

    # callback for the sensor subscriber, updates the current distance 
    def __updateDistance(self, message):
        self.currentDistance = message
        # TODO check with James to make sure this works 

        if self.distanceInitialized == False:
            self.__lastDistance = self.currentDistance
            self.distanceInitialized = True


