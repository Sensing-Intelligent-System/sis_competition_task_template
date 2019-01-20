#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from math import pi
from std_msgs.msg import String, Float64
from moveit_commander.conversions import pose_to_list
from apriltags2_ros.msg import AprilTagDetection, AprilTagDetectionArray
import ik_4dof
from place_to_box.srv import put

class moveit_tutorial(object):
    def __init__(self):
        # initial publisher for gripper command topic which is used for gripper control
        self.pub_gripper = rospy.Publisher("/gripper_joint/command",Float64,queue_size=1)

        # task3 need apriltag
        self.sub_tag = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.cbTagpose, queue_size = 1)
        # task3 needed param.
        self.tag_trackid = 0
        self.pose_place = Pose()
        self.cb_lock = False

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


        # First initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        # Instantiate a RobotCommander object. 
        # Provides information such as the robot's kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()


        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print "============ Available Planning Groups:", group_names

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state", self.robot.get_current_state()
        print ""
        
                # create service to place
        self.srv_put = rospy.Service('/place_to_box', put, self.put_item)

    def put_item(self, req):
        self.tag_trackid = req.id
        self.cb_lock = False
    ### Go home
        if self.tag_trackid == 0:
            self.home() 

    ############################ Method : Using IK to calculate joint value ############################

    # After determining a specific point where arm should move, we input x,y,z,degree to calculate joint value for each wrist. 

            #pose_goal = Pose()
            #pose_goal.position.x = 0.141
            #pose_goal.position.y = 0.020
            #pose_goal.position.z = 0.042
          # ik_4dof.ik_solver(x, y, z, degree)
            self.ik4dof(self.pose_place)

    # Reference: https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html

    ### close gripper
    ### open gripper
            self.open_gripper()

    ### Go home
            self.home()


        if self.tag_trackid == 1:
            self.home() 

    ############################ Method : Using IK to calculate joint value ############################

    # After determining a specific point where arm should move, we input x,y,z,degree to calculate joint value for each wrist. 
            rospy.sleep(2)
        # ik_4dof.ik_solver(x, y, z, degree)
            self.ik4dof(self.pose_place)
    ### close gripper
    ### open gripper
            self.open_gripper()

    ### Go home
            self.home()
            

        if self.tag_trackid == 2:
            self.home() 

    ############################ Method : Using IK to calculate joint value ############################

            self.ik4dof(self.pose_place)
    ### open gripper
            self.open_gripper()

    ### Go home
            self.home()

        if self.tag_trackid == 3:
            self.home() 
            pose_goal = Pose()
            pose_goal.position.x = 0.141
            pose_goal.position.y = 0.020
            pose_goal.position.z = 0.042

    ############################ Method : Using IK to calculate joint value ############################

            self.ik4dof(pose_goal)
    ### close gripper
    ### open gripper
            self.open_gripper()

    ### Go home
            self.home()

        if self.tag_trackid == 4:
            self.close_gripper()

        return str("end")

    def ik4dof(self, msg):
        # After determining a specific point where arm should move, we input x,y,z,degree to calculate joint value for each wrist. 

        pose_goal = msg
        # ik_4dof.ik_solver(x, y, z, degree)
        joint_value = ik_4dof.ik_solver(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, 0)

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
        grip_data.data = 1.5
        self.pub_gripper.publish(grip_data)

    def open_gripper(self):
        rospy.sleep(2)
        grip_data = Float64()
        grip_data.data = 0.5
        self.pub_gripper.publish(grip_data)
        rospy.sleep(2)

    def home(self):

        ############################ Method : Joint values (Go home!)############################


        # We can get the joint values from the group and adjust some of the values:

        # Go home!!!
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0           # arm_shoulder_pan_joint
        joint_goal[1] = -pi*5/13       # arm_shoulder_lift_joint
        joint_goal[2] = pi*3/4       # arm_elbow_flex_joint
        joint_goal[3] = pi/3           # arm_wrist_flex_joint
        joint_goal[4] = 0           # gripper_joint

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

    def cbTagpose(self, msg):
        if self.cb_lock == True:
            print "cb_lock_place = True"
            return
        if len(msg.detections) == 0:
            print "no detection of id %d"% (self.tag_trackid)
            return
        rospy.loginfo("cbTagpose")
        self.cb_lock = True
        for detection in msg.detections:
            if detection.id[0] == self.tag_trackid:
                rospy.loginfo("Tag %d found, start tracking.." %(self.tag_trackid))
                print "X: " + str(detection.pose.pose.pose.position.x)
                print "Y: " + str(detection.pose.pose.pose.position.y)
                print "Z: " + str(detection.pose.pose.pose.position.z)
                self.pose_place.position.x = detection.pose.pose.pose.position.z * 4.5/10
                self.pose_place.position.y = -1 * detection.pose.pose.pose.position.x
                self.pose_place.position.z = 0 * detection.pose.pose.pose.position.y
                exit(1)
            else:
                print "Fail"
                exit(1)


    def onShutdown(self):
        rospy.loginfo("Shutdown.")

if __name__ == '__main__': 
    rospy.init_node('moveit_tutorial',anonymous=False)
    rospy.sleep(2)
    moveit_tutorial = moveit_tutorial()
    rospy.on_shutdown(moveit_tutorial.onShutdown)
    rospy.spin()
