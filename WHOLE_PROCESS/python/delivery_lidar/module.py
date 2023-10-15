import pandas as pd
import numpy as np
import time
import math
# ----------------------------------------------------------

import ctypes as ct
import ctypes.wintypes as wt
# ----------------------------------------------------------

import cv2 as cv
# ----------------------------------------------------------

import matplotlib.pyplot as plt
import drawnow
import csv
# ----------------------------------------------------------

from math import *
from multiprocessing import shared_memory


class SHAREDMEM :

    def __init__(self) :

        self.from_morai_to_main   =   "MORAI_TO_MAIN"
        self.from_main_to_morai   =   "MAIN_TO_MORAI"
        self.from_cam_to_delivry  =   "CAM_TO_DELIVERY"


class UNIT :

    def __init__(self) :

        self.D2R            =   pi/180
        self.R2D            =   180/pi
        self.KM2MS          =   0.27777             # km/h to m/s

    def RAD2DEG(self, val) :

        return val * self.R2D

    def DEG2RAD(self, val) :

        return val * self.D2R


class REALTIME :

    def __init__(self) :

        self.time_curr      =   0
        self.time_cnt       =   0
        self.time_stime     =   0
        self.time_final     =   5000
        self.time_ts        =   0.02



class PARALLEL :

    def __init__(self) :

        self.range_max            =   9.0
        self.range_min            =   0.5

        self.near_dist            =   0

        self.mean_weight          =   1.0
        self.var_weight           =   2.0
        self.near_cluster_weight  =   7.0



class PERIPHERAL :

    def __init__(self) :

        self.colorYellow    =   (25, 255, 255)
        self.colorWhite     =   (255, 255, 255)
        self.colorRed       =   (0, 0, 255)
        self.colorBlue      =   (255, 0, 0)
        self.colorGreen     =   (0, 255, 0)
        self.userFont       =   cv.FONT_HERSHEY_COMPLEX
        self.fontSize       =   15



class DYNAMIC :

    def __init__(self) :

        self.range_max       =   9.0
        self.range_min       =   0.5
        self.lidar_num       =   761
        self.dist_thresh     =   2.5
        self.angle_thresh    =   20

        self.epsilon         =   0.25
        self.minSample       =   3



class STATIC :

    def __init__(self) :

        self.lidar_num       = 761
        self.range_max       = 9
        self.range_min       = 0.5

        self.variance        = 0.7
        self.IFF_weight      = 0.9
        self.delta_f         = 0

        # OFF expansion
        self.expand_r        = 0.12

        self.array_OFF       = np.zeros(self.lidar_num)
        self.array_OFF_exp   = np.zeros(self.lidar_num)
        self.array_SFF       = np.zeros(self.lidar_num)
        self.array_IFF       = np.zeros(self.lidar_num)


    def power(self, val) :

        return val **2



class ERP42 :

    def __init__(self) :

        self.velocity       = 2
        self.steer          = 0
        self.brake          = 0



class CALIBRATION_WITHOUT_LENS :

    def __init__(self) :

        self.unit     =   UNIT()
        self.Alpha    =   86 * self.unit.D2R
        self.Beta     =   0  * self.unit.D2R
        self.Gamma    =   0  * self.unit.D2R

        self.lidar_num     =    12000
        self.cam_height    =    0.33
        self.cam_recede    =    0.20
        self.real_recede   =    self.cam_recede * cos(self.Beta)
        self.focal_len     =    0.00367
        self.img_width     =    640
        self.img_height    =    480
        self.fov_X         =    58.92   *  self.unit.D2R
        self.fov_Y         =    58.1432 *  self.unit.D2R

        self.center_X      =    self.img_width/2
        self.center_Y      =    self.img_height/2
        self.scaling_X     =    self.focal_len * math.tan(0.5 * self.fov_X)/(0.5 * self.img_width)
        self.scaling_Y     =    self.focal_len * math.tan(0.5 * self.fov_Y)/(0.5 * self.img_height)

        self.rotation_X    = np.array([[1   ,          0        ,         0                  ],
                                       [0   ,   np.cos(self.Alpha)   ,   -np.sin(self.Alpha) ],
                                       [0   ,   np.sin(self.Alpha)   ,    np.cos(self.Alpha) ]])


        self.rotation_Y    = np.array([[np.cos(self.Beta)       ,   0   ,    np.sin(self.Beta) ],
                                       [    0                   ,   1   ,        0             ],
                                       [-np.sin(self.Beta)      ,   0   ,    np.cos(self.Beta) ]])


        self.rotation_Z    = np.array([[np.cos(self.Gamma)      ,   -np.sin(self.Gamma)  ,    0   ],
                                       [np.sin(self.Gamma)      ,   np.cos(self.Gamma)   ,    0   ],
                                       [    0                   ,        0               ,    1   ]])


        self.rotation_mat  = self.rotation_Z @ self.rotation_Y @ self.rotation_X


        self.translation_mat = np.array([[       0             ],
                                         [   self.cam_height   ],
                                         [   self.real_recede  ]])


        self.extrinsic_mat = np.hstack((self.rotation_mat, self.translation_mat))


        self.intrinsic_mat = np.array([[ self.focal_len/self.scaling_X    ,        0                            ,    self.center_X ],
                                       [      0                           ,  self.focal_len/self.scaling_Y      ,    self.center_Y ],
                                       [      0                           ,        0                            ,     1            ]])


# Distorted image (by wide angle lens) calibration
class CALIBRATION_WIDE_LENS :

    def __init__(self) :

        self.wide_focal_x    =   370.54525668
        self.wide_focal_y    =   367.47281991

        self.wide_center_x   =   322.3588068
        self.wide_center_y   =   230.43829046

        self.dist_coef_k1    =   -0.16524716
        self.dist_coef_k2    =   -0.0169815
        self.dist_coef_p1    =   -0.00244594
        self.dist_coef_p2    =   0.00038361
        self.dist_coef_k3    =   0.01692057

        self.wide_intrinsic_mat = np.array([[280.38650513  ,        0          ,        322.35003847   ],
                                            [      0       ,    269.07183838   ,        226.76398398   ],
                                            [      0       ,        0          ,                1      ]])


        self.wide_camera_mat = np.array([[   self.wide_focal_x  ,             0           , self.wide_center_x ],
                                         [        0             ,     self.wide_focal_y   , self.wide_center_y ],
                                         [        0             ,             0           ,         1          ]], dtype='double')


        self.dist_coeffs = np.array([self.dist_coef_k1, self.dist_coef_k2, self.dist_coef_p1, self.dist_coef_p2, self.dist_coef_k3])


"""
    Considers the stop condition with (platform velocity & dist thresh value)
"""
class DELIVERY :

    def __init__(self) :

        self.unit                   =   UNIT()
        self.is_A_sign_done         =   0               # True or False
        self.is_deliv_control       =   False
        self.is_deliv_brake         =   False
        self.dist_thresh            =   2.0             # unit : [m]
        self.deliv_velocity         =   5.0             # velocity command in delivery mission

        self.start_time             =   None

# TODO 배달 알고리즘 확인해봐
    def sign_stop(self, distance):

        if distance <= self.dist_thresh:

            self.is_deliv_control = True

            if not self.start_time:  self.start_time = time.time()

        else:

            self.is_deliv_control = False
            self.is_deliv_brake = False
            self.start_time = None

        if self.is_deliv_control and not self.is_deliv_brake:

            elapsed_time = time.time() - self.start_time

            time_to_brake = self.dist_thresh / (self.deliv_velocity * self.unit.KM2MS)

            if elapsed_time >= time_to_brake:
                self.is_deliv_brake = True
                self.is_A_sign_dong = 1  # Pickup flag update

                print("Braking activated ...!")











