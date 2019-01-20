#!/usr/bin/python

import numpy as np
import rospy
import cv2
import math
import time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from pose_estimate_and_pick.srv import *





class Task2(object):
    def __init__(self):

        # val
        self.flag = False
        self.item_mask = Image()

        # Bridge to convert ROS Image type to OpenCV Image type
        self.cv_bridge = CvBridge()

        # Publisher for publishing images in rviz
        self.mask_pub = rospy.Publisher("mask_prediction", Image, queue_size = 10)

        # subscribe camera rgb image
        self.img_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.cb_forpubmask, queue_size = 10)
        #clinet call server to get mask
        self.ros_mask = self.get_mask_client()

        self.srv_pred = rospy.Service('/itemmask_task2', item, self.cb_doICPmask)

        
        

    def get_mask_client(self):
        rospy.wait_for_service('/prediction_task1')
        try:
            add_mask = rospy.ServiceProxy('/prediction_task1', pred)
            resp1 = add_mask()
            return resp1.image
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def cb_doICPmask(self, req):
        item_id = req.id
        self.flag = True
        cv_mask = self.cv_bridge.imgmsg_to_cv2(self.ros_mask, desired_encoding="passthrough")
        mask = cv_mask.copy()
        if item_id == 1:
            for yc in range(0, 479):
                for xc in range(0, 639):
                    #index = 480*xc + yc
                    if mask[yc, xc] == 1:
                        continue
                    else:
                        mask[yc, xc] = 0
        if item_id == 2:
            for yc in range(0, 479):
                for xc in range(0, 639):
                    #index = 480*xc + yc
                    if mask[yc, xc] == 2:
                        continue
                    else:
                        mask[yc, xc] = 0
        if item_id == 3:
            for yc in range(0, 479):
                for xc in range(0, 639):
                    #index = 480*xc + yc
                    if mask[yc, xc] == 3:
                        continue
                    else:
                        mask[yc, xc] = 0
        ret, binary = cv2.threshold(mask, 0, 255, cv2.THRESH_BINARY)
        self.item_mask = self.cv_bridge.cv2_to_imgmsg(binary, encoding="passthrough")
        reqs = itemResponse()
        reqs.state = "good luck"
        return reqs


    def cb_forpubmask(self, msg):
        immmm = msg
        if self.flag == True:
            print('publishing item_mask')
            self.mask_pub.publish(self.item_mask)
        else:
            print('please rosservice call to gnerate item_mask')




if __name__ == '__main__':
    rospy.init_node('task2_estimation', anonymous=True)
    task2 = Task2()
    rospy.spin()

