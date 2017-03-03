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

import time


import rospy

import rl_crawler.msg

import Data.State
import Data.Action


class RobotInterface(object):
    # initialize robot interface
    def __init__(self):
        # initialize default state
        self.startingServoState = Data.State.ServoState(100, 40)

        # initialize starting distance
        self.distanceInitialized = False
        self.haveNewDistance = False
        self.__lastDistance = 0
        self.__isTerminal = False
        self.__needsReset = False


        # setup ross publishing and subscription
        self.sensor_subscriber = rospy.Subscriber('distance', rl_crawler.msg.distance, self.__updateDistance)
        self.action_publisher = rospy.Publisher('command', rl_crawler.msg.command, queue_size=1)

        self.reward_publisher = rospy.Publisher('reward', rl_crawler.msg.distance, queue_size=1)


        # set both servos to zero position
        self.reInitialize()

        # TODO ensure that action message can actually take a servoAction as a constructor argument

    # re initialize the robot to starting state values
    def reInitialize(self):
        self.setServoState(self.startingServoState)
        self.__lastDistance = self.__getUltrasonicPosition()

    # get current state
    def getCurrentState(self):
        # output last state given to the servos
        return self.__state, self.__isTerminal, self.__needsReset

    def setState(self, state):
        self.setServoState(state.convertToServoState())
    
    def setServoState(self, servoState):
        if not((servoState.farServo < 0) or (servoState.farServo > 120) or (servoState.nearServo < 70) or (servoState.nearServo > 140)):
            self.__postDesiredState(servoState)
            self.__state = servoState
            

    # returns the change in distance from the wall since the last call to getNextReward
    def getNextReward(self):
        if self.distanceInitialized == False:
            return 0

        lastDistance = self.__lastDistance
        currentDistance = self.__getUltrasonicPosition()
        deltaDistance = currentDistance - lastDistance

        if abs(deltaDistance) > 50:
            self.__lastDistance = currentDistance

            rewardMessage = rl_crawler.msg.distance()
            rewardMessage.wallDistance = deltaDistance
            
            self.reward_publisher.publish(rewardMessage)

            # end the episode when we get a good stroke
            self.__isTerminal = True
            return deltaDistance
        else:
            return -1


    ### private functions

    # returns a double of the position in inches from the wall as measured by the ultrasonic sensor on the curl bot
    def __getUltrasonicPosition(self):
        while self.haveNewDistance == False:
            time.sleep(0.05)

        self.haveNewDistance = False

        return self.currentDistance

    # tells the robot to move its servos to the given position
    def __postDesiredState(self, servoAction):
        actionMessage = rl_crawler.msg.command()
        actionMessage.nearServoPos = servoAction.nearServo
        actionMessage.farServoPos = servoAction.farServo

        self.action_publisher.publish(actionMessage)

    # callback for the sensor subscriber, updates the current distance 
    def __updateDistance(self, message):
        self.currentDistance = message.wallDistance
        self.haveNewDistance = True
        self.__isTerminal = self.currentDistance > 1000 or self.currentDistance < 200
        self.__needsReset = self.currentDistance > 1000 or self.currentDistance < 200

        if self.distanceInitialized == False:
            self.__lastDistance = self.currentDistance
            self.distanceInitialized = True


