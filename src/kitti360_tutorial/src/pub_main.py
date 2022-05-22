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
from kitti360_aggregation import *


def main():
    frame=0
    rospy.init_node("kitti360_tutorial", anonymous = True)
    rate = rospy.Rate(10)
    bridge = CvBridge()

    df_tracking = read_tracking("")
    calib = Calibration('', from_video=True)

    Datapath = rospy.get_param("datapath")

    while not rospy.is_shutdown():
        df_tracking_frame = df_tracking[df_tracking['frame'] == frame]

        imu_data = read_imu(Datapath, frame)
        boxes_2d = np.array(df_tracking[df_tracking.frame==frame]['bbox_left','bbox_top','bbox_right','bbox_bottom'])
        types = np.array(df_tracking[df_tracking.frame==frame]['type'])
        boxes_3d = np.array(df_tracking_frame[['height','width','length','pos_x','pos_y','pos_z']].values)

        corners_3d_velos = []
        for box_3d in boxes_3d:
            corners_3d_cam2= compute_3d_box_cam2(*box_3d)
            corners_3d_velo = calib.project_rect_to_velo(corners_3d_cam2.T)
            corners_3d_velos+=[corners_3d_velo]

        # Pub data
        # pub_img00(Datapath, bridge, frame)
        # pub_img01(Datapath, bridge, frame)
        # pub_img02(Datapath, bridge, frame)
        # pub_img03(Datapath, bridge, frame)
        pub_pcl(Datapath, frame)
        pub_car(frame)
        pub_imu(Datapath,frame, imu_data)
        pub_gps(Datapath, frame, imu_data)
        pub_3dbox(Datapath,frame. boxes_3d, types)

        # pub tracking data (2D bboxes) to rviz
        pub_2dbox(Datapath, frame, boxes_2d, types)

        rate.sleep()
        frame+=1

if __name__ == '__main__':
    main()
