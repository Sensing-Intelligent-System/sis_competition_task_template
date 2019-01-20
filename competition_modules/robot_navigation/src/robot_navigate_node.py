#!/usr/bin/env python
import rospy
import tf
import actionlib
import time
from robot_navigation.srv import robot_navigation
from apriltags2_ros.msg import AprilTagDetection, AprilTagDetectionArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pi, sqrt
import numpy as np
class RobotNavigate(object):
    def __init__(self):
        self.srv_navigate = rospy.Service("/robot_navigate", robot_navigation, self.cbNavigate)
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # stop at xx cm in front of tag
        #self.stop_distance_list = [0.40, 0.62]
        self.park_x = {'0':2.47, '1':2.47, '2':2.47, '5':0.77}
        self.park_y = {'0':0.30, '1':0.90, '2':1.20, '5':0.91}
        self.park_th = {'0':0, '1':0, '2':0, '5':pi}
        self.tag_id_track = 0
        self.cb_lock = False

        self.sub_tag = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.cbTag, queue_size = 1)

    def cbNavigate(self, req):
        print "cbNavigate"
        self.cb_lock = False
        print ('cb_lock == False')
        self.tag_id_track = req.id
        rospy.loginfo("NOW set tag_id_track as %d" %(req.id))        
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        
        if not (req.id == 0 or req.id == 1 or req.id == 2 or req.id == 5):
            return "NotSupportedTag"
     
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.park_x[str(req.id)]
        goal.target_pose.pose.position.y = self.park_y[str(req.id)]
        goal.target_pose.pose.position.z = 0

        quat = tf.transformations.quaternion_from_euler(0, 0, self.park_th[str(req.id)])

        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        # Go! Pikachu!
        self.client.send_goal(goal)
        _result = self.client.wait_for_result(rospy.Duration(15)) 
        if _result == False:
            self.client.cancel_goal()
        return str(_result)

    def cbTag(self, msg):
        if self.cb_lock == True:
            print "cb_lock = True"
            return
        if len(msg.detections) == 0:
            print "no detection of id %d"% (self.tag_id_track)
            return
        rospy.loginfo("cbTag")
        self.cb_lock = True
        distance = None
        for detection in msg.detections:
            if detection.id[0] == self.tag_id_track:
                rospy.loginfo("Tag %d found, start tracking.." %(self.tag_id_track))
                print "X: " + str(detection.pose.pose.pose.position.x)
                print "Z: " + str(detection.pose.pose.pose.position.z)
                distance = sqrt(detection.pose.pose.pose.position.x ** 2 + detection.pose.pose.pose.position.z ** 2)
                print "distance: " + str(distance)
                if distance <= 0.6:
                    rospy.loginfo("arrivedddddddddd")
                    exit(1)
                else:
                    try:
                        print('trytryrtytryrtyrty it')
                        rospy.wait_for_service('/robot_navigate')
                        ser_tag = rospy.ServiceProxy('/robot_navigate', robot_navigation)
                        print('testetestsetsetsetse')
                        print (ser_tag(detection.id[0]))
                        time.sleep(3)
                        self.cb_lock = False

                    except rospy.ServiceException as e:
                        print e
                        exit(1)



if __name__ == '__main__':
    rospy.init_node('robot_navigation',anonymous=False)
    node = RobotNavigate()
    rospy.spin()
