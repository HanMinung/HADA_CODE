import pandas as pd
import numpy as np
import time 
import math
import ctypes as ct
import ctypes.wintypes as wt
import cv2 as cv
import matplotlib.pyplot as plt
import drawnow

from math import *
from globVariable import *
from multiprocessing import shared_memory

from serial_node import *

# real time variable  -----------------------------------------------------

time_curr  = 0
time_cnt   = 0
time_stime = 0 
time_ts    = 0.02
time_final = 5000


# peripheral variable  ---------------------------------------------------

colorYellow    = (25, 255, 255)
colorWhite     = (255, 255, 255)
colorRed       = (0, 0, 255)
colorBlue      = (255, 0, 0)
colorGreen     = (0, 255, 0)

D2R            = pi/180
R2D            = 180/pi


# calibration variable  -------------------------------------------------

Alphadeg       = 106
Alpha          = Alphadeg * D2R
Beta           = 0 * D2R
Gamma          = 0 * D2R

cam_height     = 0.95
cam_recede     = 0.65
focalLen       = 0.00367
img_width      = 640
img_height     = 480

realRecede     = sqrt(cam_height**2 + cam_recede**2) * cos(atan(cam_height / cam_recede) - (Alphadeg - 90) * D2R)
realHeight     = sqrt(cam_height**2 + cam_recede**2) * sin(atan(cam_height / cam_recede) - (Alphadeg - 90) * D2R)

rotation_X = np.array([[1 ,          0        ,              0     ], 
                       [0 ,   np.cos(Alpha)   ,   -np.sin(Alpha)   ], 
                       [0 ,   np.sin(Alpha)   ,    np.cos(Alpha)   ]])   

rotation_Y = np.array([[ np.cos(Beta)        ,      0     ,    np.sin(Beta)], 
                       [     0               ,      1     ,        0       ], 
                       [-np.sin(Beta)        ,      0     ,    np.cos(Beta)]])

rotation_Z = np.array([[np.cos(Gamma)        ,   -np.sin(Gamma)    ,   0 ], 
                       [np.sin(Gamma)        ,    np.cos(Gamma)    ,   0 ], 
                       [    0                ,        0            ,   1 ]])    

rotation_mat   = rotation_Z @ rotation_Y @ rotation_X

translation_mat = np.array([[      0       ],
                            [  realHeight  ],
                            [  realRecede  ]]) 

extrinsic_mat = np.hstack((rotation_mat, translation_mat)) 

wide_focal_x = 370.54525668
wide_focal_y = 367.47281991

center_x = img_width/2    
center_y = img_height/2

dist_coef_k1   =   -0.16524716
dist_coef_k2   =   -0.0169815
dist_coef_p1   =   -0.00244594
dist_coef_p2   =   0.00038361
dist_coef_k3   =   0.01692057   

modified_intMat = np.array([[270.38650513,              0,        322.35003847 ],
                            [  0,                  269.07183838,  226.76398398 ],
                            [  0,                       0,           1         ]])

camera_matrix = np.array([[wide_focal_x , 0             , center_x],
                          [0            , wide_focal_y  , center_y],
                          [0            , 0             , 1       ]], dtype='double')

dist_coeffs = np.array([dist_coef_k1, dist_coef_k2, dist_coef_p1, dist_coef_p2, dist_coef_k3])  

# command variable  -------------------------------------------------

device     = 'com3'
maxsteerR  = 28
maxsteerL  = -28
vel_cmd    = 6