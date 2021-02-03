#!/usr/bin/env python
from CameraClient import CameraClient, ImageType, NetCamCmd, CameraIntri
import sys
import cv2
import numpy as np
# import rospy
# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import Image
import os
import open3d

SAVE_PATH = "."

if __name__ == '__main__':

    camera = CameraClient()
    save_file = True
    # camera ip should be modified to actual ip address
    camera_ip = "192.168.3.76"
    # always set ip before do anything else
    if not camera.connect(camera_ip):
        exit(-1)
    # get some camera info like intrincis, ip, id and Version
    intri = camera.getCameraIntri()
    print ("Camera IP: %s" % (camera.getCameraIp())) 
    print ("Camera ID: %s" % (camera.getCameraId()))
    print ("Version: %s" % (camera.getCameraVersion()))

    # capture depth image and color image and save them
    depth = camera.captureDepthImg()
    color = camera.captureColorImg()

    if len(depth) == 0 or len(color) == 0:
        exit(-2)

    if save_file:
        if not os.path.exists(SAVE_PATH):
            os.makedirs(SAVE_PATH)
        cv2.imwrite(SAVE_PATH + "/mechmind_depth.png", depth)
        cv2.imwrite(SAVE_PATH + "/mechmind_color.jpg", color)
    # set some parameters of camera, you can refer to parameters' names in Mech_eye
    
    # get rgb point cloud. Using open3d to store and visualize the cloud.
    pcd = camera.captureCloud()
    open3d.visualization.draw_geometries([pcd])
    # set some parameters of camera, you can refer to parameters' names in Mech_eye
    camera.setParameter("camera2DExpTime",15)
    print(camera.getParameter("camera2DExpTime"))
    camera.setParameter("camera2DExpTime",20)
    print(camera.getParameter("camera2DExpTime"))
    # The following is all parameters can be set
    print(camera.setParameter("period", 20))
    print(camera.setParameter("isNanoType", 0))
    print(camera.setParameter("lightPower", 300))
    print(camera.setParameter("syncExposure", 1))
    print(camera.setParameter("exposure1", 0.3))
    print(camera.setParameter("exposure2", 6))
    print(camera.setParameter("exposure3", 6))
    print(camera.setParameter("gain", 0))
    print(camera.setParameter("useBinning", 0))
    print(camera.setParameter("useColorHdr", 0))
    print(camera.setParameter("camera2DExpTime", 40))
    print(camera.setParameter("expectedGrayValue", 120))
    print(camera.setParameter("sharpenFactor", 0))
    print(camera.setParameter("contrastThres", 10))
    print(camera.setParameter("strength", 5))
    print(camera.setParameter("useMedianBlur", 1))
    print(camera.setParameter("hasThinObject", 0))
    print(camera.setParameter("lowerLimit", 800))
    print(camera.setParameter("upperLimit", 1100))
    exit(0)
