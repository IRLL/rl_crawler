#!/usr/bin/env python

import RobotInterface
import Data.State

import rospy


main():
    rospy.init_node('robot_control')
    
    interface = RobotInterface.RobotInterface()

    state = Data.State.RawState()

    # find the maximum and minimum servo positions.
    while(1):
        # set the servo state
        interface.setState(state)

        #increment the state





main()




