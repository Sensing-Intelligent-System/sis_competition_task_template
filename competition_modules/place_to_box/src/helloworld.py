#!/usr/bin/env python

import rospy	#import rospy 模組
from std_msgs.msg import String

if __name__ == '__main__':
	rospy.init_node('talker', anonymous=True)	# 初始化 talker
	pub = rospy.Publisher('chat', String, queue_size=10)
	rate = rospy.Rate(10)	# 該程式每秒執行10次

	while not rospy.is_shutdown():
		hello = 'hello world! %s' % rospy.get_time()	# %s = ropsy.get_time()
		pub.publish(hello)
		rospy.loginfo(hello)	# 在螢幕上印出 hello 之內容
		rate.sleep()	# 呼應 rate 每0.1秒執行1次
