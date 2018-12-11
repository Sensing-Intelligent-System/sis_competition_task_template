#!/usr/bin/env python

import rospy

class hello_world(object):
	def __init__(self):
		for i in range(100): rospy.loginfo("hello World")
	def onShutdown(self):
		rospy.loginfo("Shutdown !!!")

if __name__ == '__main__': 
	rospy.init_node('hello_world',anonymous=False)
	hello_world = hello_world()
	rospy.on_shutdown(hello_world.onShutdown)
	rospy.spin()
