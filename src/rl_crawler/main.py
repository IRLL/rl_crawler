#!/usr/bin/env python

from rl_crawler.RLI.Agent import CurlBotAgent

from rl_crawler.RLI.Simulation import Simulation

import rospy



rospy.init_node('rl_crawler')


print("We are doing something guys!")

simulation = Simulation()

simulation.Trials(300, 10000000)





