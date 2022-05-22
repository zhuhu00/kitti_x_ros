#!/usr/bin/env python
import imp
import cv2
import numpy as np
import pandas as pd
import os

IMU_COLUMN_NAMES = ['lat','lon','alt','roll','pitch','yaw','vn','ve','vf','vl','vu','ax','ay','az','af','al','au','wx','wy','wz','wf','wl','wu','posacc','velacc','navstat','numsats','posmode','velmode','orimode']

# TRACKING_COLUMN_NAMES = ['frame','track_id', 'type', 'truncated', 'occluded', 'alpha', 'bbox_left', 'bbox_top', 'bbox_right', 'bbox_bottom', 'dimensions_l', 'dimensions_w', 'dimensions_h', 'location_x', 'location_y', 'location_z', 'rotation_y', 'score']
TRACKING_COLUMN_NAMES = ['frame', 'track_id', 'type', 'truncated', 'occluded', 'alpha', 'bbox_left', 'bbox_top',
                         'bbox_right', 'bbox_bottom', 'height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y']

def read_imu(filepath, frame):
    df = pd.read_csv(os.path.join(filepath, "data_pose/oxts/data/%010d.txt"%frame), header=None, sep=' ')
    df.columns = IMU_COLUMN_NAMES
    return df

def read_tracking(path):
    df = pd.read_csv(path, header=None, sep=' ')
    df.columns = ['frame','x','y','z','qx','qy','qz','qw']
    return df