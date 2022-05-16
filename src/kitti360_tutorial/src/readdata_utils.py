#!/usr/bin/env python
import imp
import cv2
import numpy as np
import pandas as pd
import os

IMU_COLUMN_NAMES = ['lat','lon','alt','roll','pitch','yaw','vn','ve','vf','vl','vu','ax','ay','az','af','al','au','wx','wy','wz','wf','wl','wu','posacc','velacc','navstat','numsats','posmode','velmode','orimode']

def read_imu(filepath, frame):
    df = pd.read_csv(os.path.join(filepath, "data_pose/oxts/data/%010d.txt"%frame), header=None, sep=' ')
    df.columns = IMU_COLUMN_NAMES
    return df