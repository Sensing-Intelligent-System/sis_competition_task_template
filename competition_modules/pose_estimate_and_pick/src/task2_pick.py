#!/usr/bin/python

import numpy as np
import rospy
import sys
import math
import time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge, CvBridgeError
from pose_estimate_and_pick.srv import *
import message_filters
import ik_4dof
import moveit_commander
import moveit_msgs.msg
from math import pi

class Task2_pick(object):
    def __init__(self):

        # val
        self.flag = False
        self.item_mask = Image()
        self.cv_depthimage2 = None

        # Get the camera calibration parameter for the rectified image
        self.pos_msg = rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo, timeout=None)
        #     [fx'  0  cx' Tx]
        # P = [ 0  fy' cy' Ty]
        #     [ 0   0   1   0]

        self.fx = self.pos_msg.P[0]
        self.fy = self.pos_msg.P[5]
        self.cx = self.pos_msg.P[2]
        self.cy = self.pos_msg.P[6]

        # Bridge to convert ROS Image type to OpenCV Image type
        self.cv_bridge = CvBridge()

        # Publisher for publishing images in rviz
        #self.mask_pub = rospy.Publisher("mask_prediction", Image, queue_size = 10)
        self.pub_gripper = rospy.Publisher("/gripper_joint/command",Float64,queue_size=1)

        # subscribe camera rgb image
        #self.img_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.cb_forpubmask, queue_size = 10)
        self.image_sub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)
        self.depth_sub = message_filters.Subscriber("/camera/depth_registered/sw_registered/image_rect", Image)
        ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 10, 0.5)
        ts.registerCallback(self.rosRGBDCallBack)

        self.srv_pick = rospy.Service('/pick_task2', item, self.cb_doPick)
        self.srv_pick_test = rospy.Service('/pick_task2_test', item, self.cb_doPick_test)

        check = True
        # you need to check if moveit server is open or not
        while(check):
            check = False
            try:
        # Instantiate a MoveGroupCommander object. This object is an interface to a planning group 
        # In our case, we have "arm" and "gripper" group

                self.move_group = moveit_commander.MoveGroupCommander("arm")
            except:
                print "moveit server isn't open yet"
                check = True
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        planning_frame = self.move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame
        eef_link = self.move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link
        group_names = self.robot.get_group_names()
        print "============ Available Planning Groups:", group_names
        print "============ Printing robot state", self.robot.get_current_state()
        print ""

    def get_item_xy(self, input):
        rospy.wait_for_service('/itemmask_task2')
        try:
            item_pos = rospy.ServiceProxy('/itemmask_task2', item)
            resp1 = item_pos(input)
            return resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def cb_doPick(self, req):
        item_id = req.id
        item_xy = self.get_item_xy(item_id)
        zc = self.cv_depthimage2[int(item_xy.y_center)][int(item_xy.x_center)]
        V1 = self.getXYZ(item_xy.x_center, item_xy.y_center, zc, self.fx, self.fy, self.cx, self.cy)
        print ("x:",V1[0], "y:", V1[1], "z:", V1[2])
        self.siwitch_on(V1)
        reqs = itemResponse()
        reqs.state = "nice pick"
        reqs.x_center = item_xy.x_center
        reqs.y_center = item_xy.y_center
        return reqs

    def cb_doPick_test(self, req):
        item = req.id
        print('iiiiiiiidddddd: ', item)
        V2 = [0.141, 0.04, 0.036]
        self.siwitch_on(V2)


    def getXYZ(self, xp, yp, zc, fx,fy,cx,cy):
        #### Definition:
        # cx, cy : image center(pixel)
        # fx, fy : focal length
        # xp, yp: index of the depth image
        # zc: depth
        inv_fx = 1.0/fx
        inv_fy = 1.0/fy
        x = (xp-cx) *  zc * inv_fx
        y = (yp-cy) *  zc * inv_fy
        z = zc
        return (x,y,z)

    def rosRGBDCallBack(self, rgb_data, depth_data):

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            cv_depthimage = self.cv_bridge.imgmsg_to_cv2(depth_data, "32FC1")
            self.cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32)
            #print("z:", self.cv_depthimage2[int(240)][int(320)])
        except CvBridgeError as e:
            print(e)

    def siwitch_on(self, vec):
        x = vec[0]
        y = vec[1]
        z = vec[2]
        rospy.loginfo('please support me 10 seconds')
        self.home()
        self.open_gripper()
        self.ik4dof(x, y-0.2, z)
        self.close_gripper()
        self.home()


    def ik4dof(self, ix, iy, iz):
        # After determining a specific point where arm should move, we input x,y,z,degree to calculate joint value for each wrist. 

        pose_goal = Pose()
        pose_goal.position.x = ix
        pose_goal.position.y = iy
        pose_goal.position.z = iz
        # ik_4dof.ik_solver(x, y, z, degree)
        joint_value = ik_4dof.ik_solver(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, -60)

        for joint in joint_value:
            joint = list(joint)
            # determine gripper state
            joint.append(0)
            try:
                self.move_group.go(joint, wait=True)
            except:
                rospy.loginfo(str(joint) + " isn't a valid configuration.")

    def close_gripper(self):
        rospy.sleep(2)
        grip_data = Float64()
        grip_data.data = 1.6 
        self.pub_gripper.publish(grip_data)

    def open_gripper(self):
        rospy.sleep(2)
        grip_data = Float64()
        grip_data.data = 0.5
        self.pub_gripper.publish(grip_data)
        rospy.sleep(2)

    def home(self):

        # Go home!!!
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0           # arm_shoulder_pan_joint
        joint_goal[1] = -pi*5/13       # arm_shoulder_lift_joint
        joint_goal[2] = pi*3/4       # arm_elbow_flex_joint
        joint_goal[3] = pi/3           # arm_wrist_flex_joint
        joint_goal[4] = 0           # gripper_joint

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

    '''
    def cb_forpubmask(self, msg):
        immmm = msg
        if self.flag == True:
            print('publishing item_mask')
            self.mask_pub.publish(self.item_mask)
        else:
            print('please rosservice call to gnerate item_mask')
    '''




if __name__ == '__main__':
    rospy.init_node('task2_pick', anonymous=True)
    task2 = Task2_pick()
    rospy.spin()

