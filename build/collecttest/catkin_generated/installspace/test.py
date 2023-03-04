#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
import numpy as np

bridge = CvBridge()


def depth_callback(depth_msg):
    global gelsight_depth
    img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
    img[np.isnan(img)] = np.inf
    print("camera的数据格式:\n",np.shape(img), np.max(img), np.min(img), img.dtype)
    gelsight_depth = img
    print(img[400][400])
    np.save("/home/cp/gelsight/src/collecttest/src/1.npy", img)


if __name__ == '__main__':
    rospy.init_node('gelsight_simulation_dc')

    # rospy.Subscriber("/camera1/depth/image_raw", Image, depth_callbacklow)
    rospy.Subscriber("/probot_anno/camera/depth/image_raw", Image, depth_callback)

    rospy.spin()