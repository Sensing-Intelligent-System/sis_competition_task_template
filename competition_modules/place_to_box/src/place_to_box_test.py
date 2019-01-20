#!/usr/bin/env python

from place_to_box.srv import place_to_box, place_to_boxResponse
import rospy

def read_tagID(req):
	tag_id = req.filename.split(' ')[-1]
	print "The input tag_id is %d" % (int(tag_id))
	return place_to_boxResponse(int(tag_id))

def read_tagID_server():
	rospy.init_node('place_to_box_server')
	s = rospy.Service('place_to_box', place_to_box, read_tagID)
	print "Process successfully!"
	rospy.spin()

if __name__ == "__main__":
	read_tagID_server()
