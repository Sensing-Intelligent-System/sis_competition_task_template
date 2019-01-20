#!/usr/bin/python

import numpy as np
import rospy
import cv2
import math
import time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from object_detection.srv import * #pred, predRequest(), predResponse()

import torch
import torch.nn as nn
import torch.optim as optim
import torchvision
from torchvision import models, transforms, utils, datasets
from torchvision.models.vgg import VGG
import random
import sys

if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

from torch.optim import lr_scheduler
from torch.autograd import Variable
from torch.utils.data import Dataset, DataLoader


class Task1(object):
    def __init__(self):

		# Bridge to convert ROS Image type to OpenCV Image type
        self.cv_bridge = CvBridge()

        # Publisher for publishing images in rviz
        self.mask_eroded_pub = rospy.Publisher('/task1_detection/mask_eroded', Image, queue_size=10)
        self.mask_ero_dil_pub = rospy.Publisher('/task1_detection/mask_eroded_dilated', Image, queue_size=10)
        self.img_result_pub = rospy.Publisher('/task1_detection/img_result', Image, queue_size=10)
        #self.pub_pred = rospy.Publisher('/task1_detection/prediction', Image, queue_size = 10)

        # subscribe camera rgb image
        self.img_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.cbgetRGBimage, queue_size = 10)
        self.raw_image = None
        self.center_xy = []
        self.sim_param = False
        self.pred_param = False

        if rospy.has_param('pre_parameter'):
            self.pred_param = rospy.get_param('pre_parameter')
        if rospy.has_param('sim_parameter'):
            self.sim_param = rospy.get_param('sim_parameter')


        
        # rosservice
        self.srv_pred = rospy.Service('/prediction_task1', pred, self.cbPredict)
        self.srv_pred = rospy.Service('/simulation_prediction_task1', pred, self.cbsimulation)

    def cbgetRGBimage(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.raw_image = cv_image.copy()
            self.raw_image2 = cv_image.copy()
        except CvBridgeError as e:
            print(e)

    def cbsimulation(self, req):
        reqs = predResponse()
        if self.sim_param == True:
            color_mask = self.rosHSVProcess(self.raw_image2)
            reqs.state = str("end simulation")
            reqs.image = self.cv_bridge.cv2_to_imgmsg(color_mask, encoding="passthrough")
            return reqs
        else:
            rospy.loginfo("sim_param = False")
            reqs.state = str("end simulation")
            #reqs.image = None
            return reqs

    
    def cbPredict(self, req):
        img  = self.raw_image

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # reduce mean
        means     = np.array([103.939, 116.779, 123.68]) / 255. # mean of three channels in the order of BGR
        img = np.transpose(img, (2, 0, 1)) / 255.
        img[0] -= means[0]
        img[1] -= means[1]
        img[2] -= means[2]

        # convert to tensor

        self.image = torch.from_numpy(img.copy()).float()
        self.image = self.image[np.newaxis,:,:,:]
        reqs = predResponse()
        if self.pred_param == True:
            self.predict()
            FCN_mask = self.labelpredimage()
            reqs.state = str("end prediction")
            reqs.image = self.cv_bridge.cv2_to_imgmsg(FCN_mask, encoding="passthrough")
            return reqs
        else:
            rospy.loginfo("pred_param = False")
            reqs.state = str("end predtiction")
            #reqs.image = None
            return reqs


    def predict(self):
    # load pretrain models
        state_dict = torch.load("/root/sis_mini_competition_2018/catkin_ws/src/object_detection/module/FCNs_mini_competition_batch10_epoch39_RMSprop_lr0.0001.pkl")
        fcn_model.load_state_dict(state_dict)

        data = self.image

        if use_gpu:
            inputs = Variable(data.cuda())
        else:
            inputs = Variable(data)

        output = fcn_model(inputs)
        output = output.data.cpu().numpy()

        N, _, h, w = output.shape
        pred = output.transpose(0, 2, 3, 1).reshape(-1, n_class).argmax(axis = 1).reshape(N, h, w)
        #self.pub_pred.publish(self.cv_bridge.cv2_to_imgmsg(pred[0], encoding="passthrough"))
        self.pred_mask = pred[0]
        cv2.imwrite("/root/Pictures/FCN_mask.jpg", self.pred_mask)
        print('save the image in Path =:/root/Pictures/FCN_mask.jpg')
        
    

    def labelpredimage(self):
        self.pred_mask = cv2.imread("/root/Pictures/FCN_mask.jpg")
        self.pred_mask = cv2.cvtColor(self.pred_mask, cv2.COLOR_RGB2GRAY)
        pred_mask_eroded = cv2.erode(self.pred_mask, None, iterations = 3)
        pred_mask_eroded_dilated = cv2.dilate(pred_mask_eroded, None, iterations = 3)
        ret, binary = cv2.threshold(pred_mask_eroded_dilated, 0, 255, cv2.THRESH_BINARY)
        cv2.imwrite("/root/Pictures/binary.jpg", binary)
        image, pred_contours, hierarchy = cv2.findContours(
            binary,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        pred_image = cv2.drawContours(self.raw_image, pred_contours, -1, [0, 0, 255], 3)
        font = cv2.FONT_HERSHEY_SIMPLEX
        for cnt in pred_contours:
            x_str, y_str = cnt[0][0][:]
            xp,yp,w,h = cv2.boundingRect(cnt)
            xc = xp + w/2
            yc = yp + h/2
            #print(xc,yc)
            #print(pred_mask_eroded_dilated[yc, xc])
            if pred_mask_eroded_dilated[yc, xc] == 1:
                cv2.putText(pred_image, "Doubkemint", (x_str, y_str), font, 1, (0, 255, 255), 1, cv2.LINE_AA)
            elif pred_mask_eroded_dilated[yc, xc] == 2:
                cv2.putText(pred_image, "Kinder", (x_str, y_str), font, 1, (0, 255, 255), 1, cv2.LINE_AA)
            elif pred_mask_eroded_dilated[yc, xc] == 3:
                cv2.putText(pred_image, "Kusan", (x_str, y_str), font, 1, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.imwrite("/root/Pictures/label_image.jpg", pred_image)
        print('save the image in Path =:/root/Pictures/label_image.jpg')
        return pred_mask_eroded_dilated
        #b,g,r = cv2.split(self.pred_mask)
        #aaa = cv2.merge((b*80,g*80,r*80))
        #cv2.imwrite('./FCN_mask_after80.jpg', aaa)
            

    def rosHSVProcess(self, msg):
        cv_image = msg.copy()
        contours, contours1, contours2, mask_image, mask_image1, mask_image2 = self.HSVObjectDetection(cv_image)
        mask_added1 = cv2.add(mask_image, mask_image1)
        mask_added2 = cv2.add(mask_added1, mask_image2)
        res = cv2.bitwise_and(cv_image, cv_image, mask = mask_added2)
        self.img_result_pub.publish(self.cv_bridge.cv2_to_imgmsg(res, encoding="passthrough"))
        self.cv2_mask = res
        con_image = cv_image.copy()
        con_image = cv2.drawContours(con_image, contours, -1, [0,255,255], 4)
        con_image = cv2.drawContours(con_image, contours1, -1, [0,0,255], 4)
        con_image = cv2.drawContours(con_image, contours2, -1, [255,255,255], 4)        
        font = cv2.FONT_HERSHEY_SIMPLEX
        for cnt in contours:
            x_str, y_str = cnt[0][0][:]
            cv2.putText(con_image, "Red", (x_str, y_str), font, 1, (0, 255, 255), 2, cv2.LINE_AA)
            xp,yp,w,h = cv2.boundingRect(cnt)
            xc = xp + w/2
            yc = yp + h/2
            self.center_xy = [xc, yc]
        for cnt in contours1:
            x_str, y_str = cnt[0][0][:]
            cv2.putText(con_image, "Blue", (x_str, y_str), font, 1, (0, 255, 255), 2, cv2.LINE_AA)
        for cnt in contours2:
            x_str, y_str = cnt[0][0][:]
            cv2.putText(con_image, "Green", (x_str, y_str), font, 1, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.imwrite("/root/Pictures/raw_image.jpg", cv_image)
        cv2.imwrite("/root/Pictures/label_color_image.jpg", con_image)
        cv2.imwrite("/root/Pictures/OpenCv_mask.jpg", self.cv2_mask)
        print('save the image in Path =:/root/Pictures/raw_image.jpg, label_color_image.jpg, OpenCv_mask.jpg')
            #image_a = cv2.imread("/home/arg/FCN_mask.jpg")
            #print (image_a.size)
        return res

       
    def HSVObjectDetection(self, cv_image, toPrint = True):
        # convert image to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # define range of red color in HSV
        lower_red = np.array([160,50,50])
        upper_red = np.array([180,255,255])
        lower_blue = np.array([95,100,100])
        upper_blue = np.array([120,255,255])
        lower_green = np.array([65,50,50])
        upper_green = np.array([85,255,255])


        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv_image, lower_red, upper_red)

        mask_eroded = cv2.erode(mask, None, iterations = 3)
        mask_eroded_dilated = cv2.dilate(mask_eroded, None, iterations = 3)

        mask1 = cv2.inRange(hsv_image, lower_blue, upper_blue)

        mask_eroded1 = cv2.erode(mask1, None, iterations = 3)
        mask_eroded_dilated1 = cv2.dilate(mask_eroded1, None, iterations = 3)

        mask2 = cv2.inRange(hsv_image, lower_green, upper_green)

        mask_eroded2 = cv2.erode(mask2, None, iterations = 3)
        mask_eroded_dilated2 = cv2.dilate(mask_eroded2, None, iterations = 3)


        self.mask_eroded_pub.publish(self.cv_bridge.cv2_to_imgmsg(mask_eroded,
            encoding="passthrough"))
        self.mask_ero_dil_pub.publish(self.cv_bridge.cv2_to_imgmsg(mask_eroded_dilated,
            encoding="passthrough"))
        image, contours, hierarchy = cv2.findContours(
            mask_eroded_dilated,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        image, contours1, hierarchy = cv2.findContours(
            mask_eroded_dilated1,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        image, contours2, hierarchy = cv2.findContours(
            mask_eroded_dilated2,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        return contours, contours1, contours2, mask_eroded_dilated, mask_eroded_dilated1, mask_eroded_dilated2

class FCN16s(nn.Module):

    def __init__(self, pretrained_net, n_class):
        super(FCN16s, self).__init__()
        self.n_class = n_class
        self.pretrained_net = pretrained_net
        self.relu    = nn.ReLU(inplace = True)
        self.deconv1 = nn.ConvTranspose2d(512, 512, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn1     = nn.BatchNorm2d(512)
        self.deconv2 = nn.ConvTranspose2d(512, 256, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn2     = nn.BatchNorm2d(256)
        self.deconv3 = nn.ConvTranspose2d(256, 128, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn3     = nn.BatchNorm2d(128)
        self.deconv4 = nn.ConvTranspose2d(128, 64, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn4     = nn.BatchNorm2d(64)
        self.deconv5 = nn.ConvTranspose2d(64, 32, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn5     = nn.BatchNorm2d(32)
        self.classifier = nn.Conv2d(32, n_class, kernel_size=1)
        
    def forward(self, x):
        output = self.pretrained_net(x)
        x5 = output['x5']  # size=(N, 512, x.H/32, x.W/32)
        x4 = output['x4']  # size=(N, 512, x.H/16, x.W/16)

        score = self.relu(self.deconv1(x5))               # size=(N, 512, x.H/16, x.W/16)
        score = self.bn1(score + x4)                      # element-wise add, size=(N, 512, x.H/16, x.W/16)
        score = self.bn2(self.relu(self.deconv2(score)))  # size=(N, 256, x.H/8, x.W/8)
        score = self.bn3(self.relu(self.deconv3(score)))  # size=(N, 128, x.H/4, x.W/4)
        score = self.bn4(self.relu(self.deconv4(score)))  # size=(N, 64, x.H/2, x.W/2)
        score = self.bn5(self.relu(self.deconv5(score)))  # size=(N, 32, x.H, x.W)
        score = self.classifier(score)                    # size=(N, n_class, x.H/1, x.W/1)

        return score

ranges = {
    'vgg11': ((0, 3), (3, 6),  (6, 11),  (11, 16), (16, 21)),
    'vgg13': ((0, 5), (5, 10), (10, 15), (15, 20), (20, 25)),
    'vgg16': ((0, 5), (5, 10), (10, 17), (17, 24), (24, 31)),
    'vgg19': ((0, 5), (5, 10), (10, 19), (19, 28), (28, 37))
}

# cropped version from https://github.com/pytorch/vision/blob/master/torchvision/models/vgg.py
cfg = {
    'vgg11': [64, 'M', 128, 'M', 256, 256, 'M', 512, 512, 'M', 512, 512, 'M'],
    'vgg13': [64, 64, 'M', 128, 128, 'M', 256, 256, 'M', 512, 512, 'M', 512, 512, 'M'],
    'vgg16': [64, 64, 'M', 128, 128, 'M', 256, 256, 256, 'M', 512, 512, 512, 'M', 512, 512, 512, 'M'],
    'vgg19': [64, 64, 'M', 128, 128, 'M', 256, 256, 256, 256, 'M', 512, 512, 512, 512, 'M', 512, 512, 512, 512, 'M'],
}

class VGGNet(VGG):
    def __init__(self, pretrained=True, model='vgg16', requires_grad=True, remove_fc=True, show_params=False):

        super(VGGNet, self).__init__(make_layers(cfg[model]))
        self.ranges = ranges[model]

        if pretrained:
            exec("self.load_state_dict(models.%s(pretrained=True).state_dict())" % model)

        if not requires_grad:
            for param in super().parameters():
                param.requires_grad = False

        if remove_fc:  # delete redundant fully-connected layer params, can save memory
            del self.classifier

        if show_params:
            for name, param in self.named_parameters():
                print(name, param.size())

    def forward(self, x):
        output = {}

        # get the output of each maxpooling layer (5 maxpool in VGG net)
        for idx in range(len(self.ranges)):
            for layer in range(self.ranges[idx][0], self.ranges[idx][1]):      
                x = self.features[layer](x)
            output["x%d"%(idx+1)] = x
        return output

def make_layers(cfg, batch_norm=False):
    layers = []
    in_channels = 3
    for v in cfg:
        if v == 'M':
            layers += [nn.MaxPool2d(kernel_size=2, stride=2)]
        else:
            conv2d = nn.Conv2d(in_channels, v, kernel_size=3, padding=1)
            if batch_norm:
                layers += [conv2d, nn.BatchNorm2d(v), nn.ReLU(inplace=True)]
            else:
                layers += [conv2d, nn.ReLU(inplace=True)]
            in_channels = v
    return nn.Sequential(*layers)

n_class = 4
use_gpu = torch.cuda.is_available()
num_gpu = list(range(torch.cuda.device_count()))

vgg_model = VGGNet(requires_grad=True, remove_fc=True)
fcn_model = FCN16s(pretrained_net=vgg_model, n_class=n_class)

if use_gpu:
    ts = time.time()
    vgg_model = vgg_model.cuda()
    fcn_model = fcn_model.cuda()
    fcn_model = nn.DataParallel(fcn_model, device_ids=num_gpu)
    print("Finish cuda loading, time elapsed {}".format(time.time() - ts))



if __name__ == '__main__':
    rospy.init_node('task1_detection', anonymous=True)
    task1 = Task1()
    rospy.spin()

