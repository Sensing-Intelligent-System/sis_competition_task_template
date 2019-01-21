#!/usr/bin/python

# 2.12 Lab 4 object detection: a node for detecting objects
# Peter Yu Oct 2016
# Revised by Michael Su Oct 2018

import rospy
import numpy as np
import cv2  # OpenCV module

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import ColorRGBA

from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math

rospy.init_node('object_detection', anonymous=True)

# Publisher for publishing pyramid marker in rviz
vis_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

# Publisher for publishing images in rviz
mask_eroded_pub = rospy.Publisher('/object_detection/mask_eroded', Image, queue_size=10)
mask_ero_dil_pub = rospy.Publisher('/object_detection/mask_eroded_dilated', Image, queue_size=10)
img_result_pub = rospy.Publisher('/object_detection/img_result', Image, queue_size=10)

# Bridge to convert ROS Image type to OpenCV Image type
cv_bridge = CvBridge()

# Get the camera calibration parameter for the rectified image
msg = rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo, timeout=None)
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]

fx = msg.P[0]
fy = msg.P[5]
cx = msg.P[2]
cy = msg.P[6]

def main():
    #    Subscribe to both RGB and Depth images with a Synchronizer
    image_sub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)
    depth_sub = message_filters.Subscriber("/camera/depth_registered/sw_registered/image_rect", Image)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10, 0.5)
    ts.registerCallback(rosRGBDCallBack)
    rospy.spin()

####red
def HSVObjectDetection_red(cv_image, toPrint = True):
    # convert image to HSV color space
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define range of red color in HSV
    lower_red = np.array([130,50,50])
    upper_red = np.array([180,255,255])

    # Threshold the HSV image to get only red colors
    mask = cv2.inRange(hsv_image, lower_red, upper_red)

    mask_eroded = cv2.erode(mask, None, iterations = 3)
    mask_eroded_dilated = cv2.dilate(mask_eroded, None, iterations = 3)

    mask_eroded_pub.publish(cv_bridge.cv2_to_imgmsg(mask_eroded,encoding="passthrough"))
    mask_ero_dil_pub.publish(cv_bridge.cv2_to_imgmsg(mask_eroded_dilated,encoding="passthrough"))
    image, contours, hierarchy = cv2.findContours(
        mask_eroded_dilated,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    return contours, mask_eroded_dilated
####green
def HSVObjectDetection_green(cv_image, toPrint = True):
    # convert image to HSV color space
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define range of red color in HSV
    lower_red = np.array([30,50,50])
    upper_red = np.array([60,255,255])

    # Threshold the HSV image to get only red colors
    mask = cv2.inRange(hsv_image, lower_red, upper_red)

    mask_eroded = cv2.erode(mask, None, iterations = 3)
    mask_eroded_dilated = cv2.dilate(mask_eroded, None, iterations = 3)

    mask_eroded_pub.publish(cv_bridge.cv2_to_imgmsg(mask_eroded,encoding="passthrough"))
    mask_ero_dil_pub.publish(cv_bridge.cv2_to_imgmsg(mask_eroded_dilated,encoding="passthrough"))
    image, contours, hierarchy = cv2.findContours(
        mask_eroded_dilated,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    return contours, mask_eroded_dilated
####blue
def HSVObjectDetection_blue(cv_image, toPrint = True):
    # convert image to HSV color space
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define range of red color in HSV
    lower_red = np.array([70,120,120])
    upper_red = np.array([110,255,255])

    # Threshold the HSV image to get only red colors
    mask = cv2.inRange(hsv_image, lower_red, upper_red)

    mask_eroded = cv2.erode(mask, None, iterations = 3)
    mask_eroded_dilated = cv2.dilate(mask_eroded, None, iterations = 3)

    mask_eroded_pub.publish(cv_bridge.cv2_to_imgmsg(mask_eroded,encoding="passthrough"))
    mask_ero_dil_pub.publish(cv_bridge.cv2_to_imgmsg(mask_eroded_dilated,encoding="passthrough"))
    image, contours, hierarchy = cv2.findContours(
        mask_eroded_dilated,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    return contours, mask_eroded_dilated


def rosRGBDCallBack(rgb_data, depth_data):
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(rgb_data, "bgr8")
        cv_depthimage = cv_bridge.imgmsg_to_cv2(depth_data, "32FC1")
        cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32)
    except CvBridgeError as e:
        print(e)

    org_image=cv_image
    contours, mask_image = HSVObjectDetection_red(org_image, toPrint = False)

    for cnt in contours: 
	    cv2.drawContours(cv_image, cnt, -1, (255,0,0), 3)
	    side = cv2.arcLength(cnt, False)
            shape = cv2.approxPolyDP(cnt, 0.04 * side, True)
	    if len(shape) == 3:
               obj_shape = "triangle"
            elif len(shape) == 4:
               obj_shape = "square"
            else :
               obj_shape = "circle"
    	    if len(cnt) < 1:
                    continue
            x_coor, y_coor = cnt[0][0][:]
            if obj_shape == "triangle":
               cv2.putText(cv_image, "triangle", (x_coor, y_coor), 0, 1,
                   (0, 0, 0), 3)
            elif obj_shape == "square":
               cv2.putText(cv_image, "square", (x_coor, y_coor), 0, 1,
                   (0, 0, 0), 3)
            elif obj_shape == "circle":
               cv2.putText(cv_image, "circle", (x_coor, y_coor), 0, 1,
                   (0, 0, 0), 3)
     
    contours, mask_image = HSVObjectDetection_green(org_image, toPrint = False)
    for cnt in contours: 
	    cv2.drawContours(cv_image, cnt, -1, (255,0,0), 3)
	    side = cv2.arcLength(cnt, False)
            shape = cv2.approxPolyDP(cnt, 0.04 * side, True)
	    if len(shape) == 3:
               obj_shape = "triangle"
            elif len(shape) == 4:
               obj_shape = "square"
            else :
               obj_shape = "circle"
    	    if len(cnt) < 1:
                    continue
            x_coor, y_coor = cnt[0][0][:]
            if obj_shape == "triangle":
               cv2.putText(cv_image, "triangle", (x_coor, y_coor), 0, 1,
                   (0, 0, 0), 3)
            elif obj_shape == "square":
               cv2.putText(cv_image, "square", (x_coor, y_coor), 0, 1,
                   (0, 0, 0), 3)
            elif obj_shape == "circle":
               cv2.putText(cv_image, "circle", (x_coor, y_coor), 0, 1,
                   (0, 0, 0), 3)
    contours, mask_image = HSVObjectDetection_blue(org_image, toPrint = False)
    for cnt in contours: 
	    cv2.drawContours(cv_image, cnt, -1, (255,0,0), 3)
	    side = cv2.arcLength(cnt, False)
            shape = cv2.approxPolyDP(cnt, 0.04 * side, True)
	    if len(shape) == 3:
               obj_shape = "triangle"
            elif len(shape) == 4:
               obj_shape = "square"
            else :
               obj_shape = "circle"
    	    if len(cnt) < 1:
                    continue
            x_coor, y_coor = cnt[0][0][:]
            if obj_shape == "triangle":
               cv2.putText(cv_image, "triangle", (x_coor, y_coor), 0, 1,
                   (0, 0, 0), 3)
            elif obj_shape == "square":
               cv2.putText(cv_image, "square", (x_coor, y_coor), 0, 1,
                   (0, 0, 0), 3)
            elif obj_shape == "circle":
               cv2.putText(cv_image, "circle", (x_coor, y_coor), 0, 1,
                   (0, 0, 0), 3)
    cv2.imshow("dtect",cv_image)
    #cv2.imwrite("/home/nvidia/detection_1.bmp",cv_image)
    img_result_pub.publish(cv_bridge.cv2_to_imgmsg(cv_image,
        encoding="passthrough"))

def getXYZ(xp, yp, zc, fx,fy,cx,cy):
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



# Create a pyramid using 4 triangles
def showPyramid(xp, yp, zc, w, h):
    # X1-X4 are the 4 corner points of the base of the pyramid
    X1 = getXYZ(xp-w/2, yp-h/2, zc, fx, fy, cx, cy)
    X2 = getXYZ(xp-w/2, yp+h/2, zc, fx, fy, cx, cy)
    X3 = getXYZ(xp+w/2, yp+h/2, zc, fx, fy, cx, cy)
    X4 = getXYZ(xp+w/2, yp-h/2, zc, fx, fy, cx, cy)
    vis_pub.publish(createTriangleListMarker(1, [X1, X2, X3, X4], rgba = [1,0,0,1], frame_id = "/camera_rgb_optical_frame"))

# Create a list of Triangle markers for visualizationi
def createTriangleListMarker(marker_id, points, rgba, frame_id = "/camera_rgb_optical_frame"):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = marker.TRIANGLE_LIST
    marker.scale = Vector3(1,1,1)
    marker.id = marker_id

    n = len(points)

    if rgba is not None:
        marker.color = ColorRGBA(*rgba)

    o = Point(0,0,0)
    for i in xrange(n):
        p = Point(*points[i])
        marker.points.append(p)
        p = Point(*points[(i+1)%4])
        marker.points.append(p)
        marker.points.append(o)

    marker.pose = poselist2pose([0,0,0,0,0,0,1])
    return marker


def poselist2pose(poselist):
    return Pose(Point(*poselist[0:3]), Quaternion(*poselist[3:7]))


if __name__=='__main__':
    main()
