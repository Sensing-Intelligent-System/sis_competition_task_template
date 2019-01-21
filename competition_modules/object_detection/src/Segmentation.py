import matplotlib.pyplot as plt
import matplotlib.image as pli
import numpy as np
import random
import os
import time

import torch
import torch.nn as nn
from torchvision import models
from torchvision.models.vgg import VGG
from torch.autograd import Variable
from torch.utils.data import Dataset, DataLoader

import threading

import cv2

ranges = {
    'vgg11': ((0, 3), (3, 6), (6, 11), (11, 16), (16, 21)),
    'vgg13': ((0, 5), (5, 10), (10, 15), (15, 20), (20, 25)),
    'vgg16': ((0, 5), (5, 10), (10, 17), (17, 24), (24, 31)),
    'vgg19': ((0, 5), (5, 10), (10, 19), (19, 28), (28, 37))
}

# cropped version from https://github.com/pytorch/vision/blob/master/torchvision/models/vgg.py
cfg = {
    'vgg11': [64, 'M', 128, 'M', 256, 256, 'M', 512, 512, 'M', 512, 512, 'M'],
    'vgg13': [64, 64, 'M', 128, 128, 'M', 256, 256, 'M', 512, 512, 'M', 512, 512, 'M'],
    'vgg16': [64, 64, 'M', 128, 128, 'M', 256, 256, 256, 'M', 512, 512, 512, 'M', 512, 512, 512, 'M'],
    'vgg19': [64, 64, 'M', 128, 128, 'M', 256, 256, 256, 256, 'M', 512, 512, 512, 512, 'M', 512, 512, 512, 512,
              'M'],
}

class VGGNet(VGG):
    def __init__(self, pretrained=True, model='vgg16', requires_grad=True, remove_fc=True, show_params=False):
        super(VGGNet,self).__init__(make_layers(cfg[model]))
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


def size_cal(pred_img,object_name):
    if object_name== "doublemint":
        label=1
    if object_name== "kinder":
        label=2
    if object_name== "kusan":
        label=3
    dim=pred_img[0].shape
    record = torch.zeros(3, 4)
    for i in range(0,3):
        record[i,0]=480
        record[i,2]=640
    for i in range(0,480):
        for j in range(0,640):
            #doublemint
            if(pred_img[i,j]==label):
                #x_min
                if(i<record[0,0]):    
                    record[0,0]=i
                #x_max
                if(i>record[0,1]):
                    record[0,1]=i
                #y_min
                if(j<record[0,2]):
                    record[0,2]=j
                #y_max
                if(j>record[0,3]):
                    record[0,3]=j
    object_x=record[0,0]
    object_y=record[0,2]
    object_l=record[0,3]-record[0,2]
    object_w=record[0,1]-record[0,0]
    object_size={'x':object_x,'Y':object_y,'lenght':object_l,'width':object_w}
    return object_size


def preprocessing(org_img):
    means   = np.array([103.939, 116.779, 123.68]) / 255. # mean of three channels in the order of BGR
    org_img = np.transpose(org_img, (2, 0, 1)) / 255.        
    # reduce mean
    org_img[0] -= means[0]
    org_img[1] -= means[1]
    org_img[2] -= means[2]
    # convert to tensor
    org_img = torch.from_numpy(org_img.copy()).float()
    t_img=np.resize(org_img,(1,3,480,640))
    t_img = torch.from_numpy(t_img.copy()).float()

    if torch.cuda.is_available():
        inputs = Variable(t_img.cuda())
    else:
        inputs = Variable(t_img)

    return inputs



class Segmentation:

    def __init__(self, model_path):
        #initial GPU
        # sample test
        use_gpu = torch.cuda.is_available()
        num_gpu = list(range(torch.cuda.device_count()))  # GPU number
        ######################################################################
        # load pretrain models and para
        self.vgg_model = VGGNet(requires_grad=True, remove_fc=True)
        self.fcn_model = FCN16s(pretrained_net=self.vgg_model, n_class=4)

        if use_gpu:
            ts = time.time()
            self.vgg_model = self.vgg_model.cuda()
            self.fcn_model = self.fcn_model.cuda()
            self.fcn_model = nn.DataParallel(self.fcn_model, device_ids=num_gpu)
            print("Finish cuda loading, time elapsed {}".format(time.time() - ts))

        state_dict = torch.load(model_path)
        self.fcn_model.load_state_dict(state_dict)



    def seg_one_frame(self, img):
        # preprocessing
        inputs = preprocessing(img)
        # predict
        output = self.fcn_model(inputs)
        output = output.data.cpu().numpy()
        N, _, h, w = output.shape
        pred = output.transpose(0, 2, 3, 1).reshape(-1, 4).argmax(axis=1).reshape(N, h, w)

        return pred[0]
