#!/usr/bin/env python
from CameraClient import CameraClient, ImageType, Command, CameraIntri
import sys
import cv2
import numpy as np
import os
import open3d

SAVE_PATH = "D:"

if __name__ == '__main__':

    camera = CameraClient()
    save_file = True
    # The camera IP should be modified to the actual IP address.
    camera_ip = "192.168.3.226"
    # Always set IP before doing anything else.
    if not camera.connect(camera_ip):
        exit(-1)
    # Get some camera information like intrincis, IP, ID, version and image size.
    intri = camera.getCameraIntri()
    print ("Camera Info: %s" % (camera.getCameraInfo()))
    print ("Camera ID: %s" % (camera.getCameraId()))
    print ("Version: %s" % (camera.getCameraVersion()))
    print ("Color Image Size: %s %s" % (camera.getColorImgSize()))
    print ("Depth Image Size: %s %s" % (camera.getDepthImgSize()))

    # Capture depth image and color image and save them.
    depth = camera.captureDepthImg()
    color = camera.captureColorImg()

    if len(depth) == 0 or len(color) == 0:
        exit(-2)

    if save_file:
        if not os.path.exists(SAVE_PATH):
            os.makedirs(SAVE_PATH)
        cv2.imwrite(SAVE_PATH + "/mechmind_depth.tif", depth)
        cv2.imwrite(SAVE_PATH + "/mechmind_color.jpg", color)

    # Get rgb point cloud and use open3d to store and visualize the cloud.
    pcd = camera.captureCloud()
    open3d.visualization.draw_geometries([pcd])
    # Set some parameters of camera which can refer to parameters' names in Mech_Eye.
    print(camera.setParameter("scan2dExposureMode",0)) # set exposure mode to Timed
    print(camera.getParameter("scan2dExposureMode"))
    print(camera.setParameter("scan2dExposureTime",20)) # set exposure time to 20ms
    print(camera.getParameter("scan2dExposureTime"))
    exit(0)
