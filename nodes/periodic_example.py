#!/usr/bin/env python

import rospy
from rl_crawler.msg import action as action_msg
from rl_crawler.msg import state as state_msg

#how to import stuff from /src/rl_crawler/:
#from rl_crawler.filename import classname

class Node:
    def __init__(self):
        #run at 10 hz
        self.rate = rospy.Rate(10)
        
        #create subscriber to capture incoming messages
        self.sensor_sub = rospy.Subscriber('state', state_msg, self.callback)
        #create publisher to send action messages
        self.sensor_sub = rospy.Publisher('cmd', action_msg, queue_size=1)
        
        #create initial state
        self.current_state = state_msg()

    def callback(self, message):
        #just save message locally, will use in update function
        self.current_state = message

    def update(self):
        action = action_msg()
        #do RL logic


        #publish action message
        self.action_pub.publish(action)


if __name__ == "__main__":
   rospy.init_node('example node')
   node = Node()

    while not rospy.is_shutdown():
        node.update()
        node.rate.sleep()
    
