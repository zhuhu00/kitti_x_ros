#!/usr/bin/env python
import cv2
import os
import numpy as np
import time

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pcl2

# Datapath = "/home/roma/Documents/00-SLAM_dataset/kitti-360/data_2d_raw/2013_05_28_drive_0000_sync/"

def pub_img00(filepath,bridge, frame):
    img00 = cv2.imread(os.path.join(filepath,"data_2d_raw/2013_05_28_drive_0000_sync/image_00/data_rgb/%010d.png"%frame))

    img00_pub = rospy.Publisher('/kitti360_img00', Image, queue_size = 10)
    img00_frame= bridge.cv2_to_imgmsg(img00, "bgr8")

    header = Header(stamp=rospy.Time.now())
    header.frame_id = "map"
    img00_frame.header = header
    img00_pub.publish(img00_frame)
    # rospy.loginfo("pub Camera")

def pub_img01(filepath,bridge, frame):
    img01 = cv2.imread(os.path.join(filepath,"data_2d_raw/2013_05_28_drive_0000_sync/image_01/data_rgb/%010d.png"%frame))

    img01_pub = rospy.Publisher('/kitti360_img01', Image, queue_size = 10)
    img01_frame= bridge.cv2_to_imgmsg(img01, "bgr8")

    header = Header(stamp=rospy.Time.now())
    header.frame_id = "map"
    img01_frame.header = header
    img01_pub.publish(img01_frame)
    print(header.stamp)

    # rospy.loginfo("pub Camera")

def pub_img02(filepath,bridge, frame):
    img02 = cv2.imread(os.path.join(filepath,"data_2d_raw/2013_05_28_drive_0000_sync/image_02/data_rgb/%010d.png"%frame))

    img02_pub = rospy.Publisher('/kitti360_img02', Image, queue_size = 10)
    img02_frame= bridge.cv2_to_imgmsg(img02, "bgr8")

    header = Header(stamp=rospy.Time.now())
    header.frame_id = "map"
    img02_frame.header = header
    img02_pub.publish(img02_frame)
    print(header.stamp)

    # rospy.loginfo("pub Fish Camera")

def pub_img03(filepath,bridge, frame):
    img03 = cv2.imread(os.path.join(filepath,"data_2d_raw/2013_05_28_drive_0000_sync/image_03/data_rgb/%010d.png"%frame))

    img03_pub = rospy.Publisher('/kitti360_img03', Image, queue_size = 10)
    img03_frame= bridge.cv2_to_imgmsg(img03, "bgr8")

    header = Header(stamp=rospy.Time.now())
    header.frame_id = "map"
    img03_frame.header = header
    img03_pub.publish(img03_frame)
    print(header.stamp)
    # rospy.loginfo("pub Fish Camera")

def pub_pcl(filepath, frame):
    pcl_data = np.fromfile(os.path.join(filepath, "data_3d_raw/2013_05_28_drive_0000_sync/velodyne_points/data/%010d.bin"%frame), dtype=np.float32).reshape(-1, 4)
    pcl_pub = rospy.Publisher('/kitti360_pointcloud', PointCloud2, queue_size = 10)

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"
    pcl_pub.publish(pcl2.create_cloud_xyz32(header, pcl_data[:,:3]))
    print(header.stamp)

    # rospy.loginfo("pub PointCloud")

    return 

def main():
    frame=0
    rospy.init_node("kitti360_tutorial", anonymous = True)
    rate = rospy.Rate(10)
    bridge = CvBridge()

    Datapath = rospy.get_param("datapath")

    while not rospy.is_shutdown():
        pub_img00(Datapath, bridge, frame)
        pub_img01(Datapath, bridge, frame)
        pub_img02(Datapath, bridge, frame)
        pub_img03(Datapath, bridge, frame)
        pub_pcl(Datapath, frame)

        rate.sleep()
        frame+=1

if __name__ == '__main__':
    main()
