#!/usr/bin/env python
import imp
import cv2
import os
import numpy as np
import time

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, Imu
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pcl2
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
import tf
from readdata_utils import *

FRAME_ID = "map"

def pub_img00(filepath,bridge, frame):
    img00 = cv2.imread(os.path.join(filepath,"data_2d_raw/image_00/data_rgb/%010d.png"%frame))

    img00_pub = rospy.Publisher('/kitti360_img00', Image, queue_size = 10)
    img00_frame= bridge.cv2_to_imgmsg(img00, "bgr8")

    header = Header(stamp=rospy.Time.now())
    header.frame_id = "map"
    img00_frame.header = header
    img00_pub.publish(img00_frame)
    # rospy.loginfo("pub Camera")

def pub_img01(filepath,bridge, frame):
    img01 = cv2.imread(os.path.join(filepath,"data_2d_raw/image_01/data_rgb/%010d.png"%frame))

    img01_pub = rospy.Publisher('/kitti360_img01', Image, queue_size = 10)
    img01_frame= bridge.cv2_to_imgmsg(img01, "bgr8")

    header = Header(stamp=rospy.Time.now())
    header.frame_id = "map"
    img01_frame.header = header
    img01_pub.publish(img01_frame)
    print(header.stamp)

    # rospy.loginfo("pub Camera")

def pub_img02(filepath,bridge, frame):
    img02 = cv2.imread(os.path.join(filepath,"data_2d_raw/image_02/data_rgb/%010d.png"%frame))

    img02_pub = rospy.Publisher('/kitti360_img02', Image, queue_size = 10)
    img02_frame= bridge.cv2_to_imgmsg(img02, "bgr8")

    header = Header(stamp=rospy.Time.now())
    header.frame_id = "map"
    img02_frame.header = header
    img02_pub.publish(img02_frame)
    print(header.stamp)

    # rospy.loginfo("pub Fish Camera")

def pub_img03(filepath,bridge, frame):
    img03 = cv2.imread(os.path.join(filepath,"data_2d_raw/image_03/data_rgb/%010d.png"%frame))

    img03_pub = rospy.Publisher('/kitti360_img03', Image, queue_size = 10)
    img03_frame= bridge.cv2_to_imgmsg(img03, "bgr8")

    header = Header(stamp=rospy.Time.now())
    header.frame_id = "map"
    img03_frame.header = header
    img03_pub.publish(img03_frame)
    print(header.stamp)
    # rospy.loginfo("pub Fish Camera")

def pub_pcl(filepath, frame):
    pcl_data = np.fromfile(os.path.join(filepath, "data_3d_raw/velodyne_points/data/%010d.bin"%frame), dtype=np.float32).reshape(-1, 4)
    pcl_pub = rospy.Publisher('/kitti360_point_raw', PointCloud2, queue_size = 10)

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"
    pcl_pub.publish(pcl2.create_cloud_xyz32(header, pcl_data[:,:3]))
    print(header.stamp)

    # rospy.loginfo("pub PointCloud")

def pub_car(filepath, frame):
    """
    Publish left and right 45 degreee FOV lines and ego car model mesh
    """
    car_pub = rospy.Publisher('kitti360_car_model', MarkerArray, queue_size=10)
    
    marker_array = MarkerArray()

    # Add fov lines
    # fov_marker = Marker()
    # fov_marker.header.frame_id = FRAME_ID
    # fov_marker.header.stamp = rospy.Time.now()
    # fov_marker.id = 0
    # fov_marker.action = Marker.ADD
    # fov_marker.lifetime = rospy.Duration()
    # fov_marker.type = Marker.LINE_STRIP

    # fov_marker.color.r = 0.0
    # fov_marker.color.g = 1.0
    # fov_marker.color.b = 0.0
    # fov_marker.color.a = 1.0
    # fov_marker.scale.x = 0.2

    # fov_marker.points = []
    # fov_marker.points.append(Point(10, -10, 0))
    # fov_marker.points.append(Point(0, -0, 0))
    # fov_marker.points.append(Point(10, 10, 0))
    
    # marker_array.markers.append(fov_marker)

    # Add car model
    mesh_marker = Marker()
    mesh_marker.header.frame_id = FRAME_ID
    mesh_marker.header.stamp = rospy.Time.now()

    mesh_marker.id = -1
    mesh_marker.lifetime = rospy.Duration()
    mesh_marker.type = Marker.MESH_RESOURCE
    mesh_marker.mesh_resource = "package://kitti360_tutorial/config/model/car.dae"

    mesh_marker.pose.position.x = 0.0
    mesh_marker.pose.position.y = 0.0
    mesh_marker.pose.position.z = -1.73

    q = tf.transformations.quaternion_from_euler(0,0,  np.pi/2)
    mesh_marker.pose.orientation.x = q[0]
    mesh_marker.pose.orientation.y = q[1]
    mesh_marker.pose.orientation.z = q[2]
    mesh_marker.pose.orientation.w = q[3]

    mesh_marker.color.r = 1.0
    mesh_marker.color.g = 1.0
    mesh_marker.color.b = 1.0
    mesh_marker.color.a = 1.0

    mesh_marker.scale.x = 1.1
    mesh_marker.scale.y = 1.1
    mesh_marker.scale.z = 1.1

    marker_array.markers.append(mesh_marker)
    car_pub.publish(marker_array)

def pub_imu(filepath, frame, imu_data):
    imu_pub = rospy.Publisher('kitti360_imu', Imu, queue_size=10)
    
    imu  = Imu()
    imu.header.frame_id = FRAME_ID
    imu.header.stamp = rospy.Time.now()

    q = tf.transformations.quaternion_from_euler(float(imu_data.roll),float(imu_data.pitch), float(imu_data.yaw))
    imu.orientation.x = q[0]
    imu.orientation.y = q[1]
    imu.orientation.z = q[2]
    imu.orientation.w = q[3]
    imu.linear_acceleration.x = imu_data.af
    imu.linear_acceleration.y = imu_data.al
    imu.linear_acceleration.z = imu_data.au
    imu.angular_velocity.x = imu_data.wf
    imu.angular_velocity.y = imu_data.wl
    imu.angular_velocity.z = imu_data.wu

    imu_pub.publish(imu)

def pub_gps(filepath, frame, gps_data):
    gps_pub = rospy.Publisher('kitti360_gps', NavSatFix, queue_size=10)
    

    gps_pub.publish(gps_data)