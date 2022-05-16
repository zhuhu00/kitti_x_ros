#!/usr/bin/env python
import imp
import cv2
import os
import numpy as np
import time

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pcl2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf

from pub_utils import *
from readdata_utils import *


def main():
    frame=0
    rospy.init_node("kitti360_tutorial", anonymous = True)
    rate = rospy.Rate(10)
    bridge = CvBridge()

    Datapath = rospy.get_param("datapath")

    while not rospy.is_shutdown():
        # pub_img00(Datapath, bridge, frame)
        # pub_img01(Datapath, bridge, frame)
        # pub_img02(Datapath, bridge, frame)
        # pub_img03(Datapath, bridge, frame)
        imu_data = read_imu(Datapath, frame)
        pub_pcl(Datapath, frame)
        pub_car(Datapath, frame)
        pub_imu(Datapath,frame, imu_data)
        # pub_gps(Datapath, frame, imu_data)

        rate.sleep()
        frame+=1

if __name__ == '__main__':
    main()
