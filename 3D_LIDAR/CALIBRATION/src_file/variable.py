"""
   - CODE DESCRIPTION   :  3D 라이다 - 카메라 센서 캘리브레이션을 진행하기 위한 여러 변수 선언 파일
   - writer             :  한민웅           
   - 코드 구성           :  calibration.py (캘리브레이션 테스트),  devliveryMission.py (배달 미션 테스트), signDistance.py (캘리브레이션 통한 표지판 거리 측정)
"""

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

# ----------------------------------------------------------

from serial_node import *

"""
    GLOBAL VARIABLE DEFINITION
"""

"""
    1. REAL-TIME VARIABLE
"""

time_curr  = 0
time_cnt   = 0
time_stime = 0 
time_ts    = 0.15
time_final = 5000


"""
    2. 잡다한 변수들
"""

colorYellow    = (25, 255, 255)
colorWhite     = (255, 255, 255)
colorRed       = (0, 0, 255)
colorBlue      = (255, 0, 0)
colorGreen     = (0, 255, 0)
userFont       = cv.FONT_HERSHEY_COMPLEX
D2R            = pi/180
R2D            = 180/pi


"""
    3. CABLIRATION VARIABLE
"""

Alpha          = 86   * D2R
Beta           = 0    * D2R
Gamma          = 0    * D2R

nLidar         = 12000
camHeight      = 0.33
camRecede      = 0.26
real_recede    = camRecede * cos(Beta)
focalLen       = 0.00367
imgWidth       = 640
imgHeight      = 480
fovX           = 61.92   *  D2R
fovY           = 58.1432 *  D2R


ox             = imgWidth/2                                              
oy             = imgHeight/2
sx             = focalLen * math.tan(0.5 * fovX)/(0.5 * imgWidth) 
sy             = focalLen * math.tan(0.5 * fovY)/(0.5 * imgHeight)

rotX = np.array([[1   ,          0        ,         0        ], 
                 [0   ,   np.cos(Alpha)   ,   -np.sin(Alpha) ], 
                 [0   ,   np.sin(Alpha)   ,    np.cos(Alpha) ]])   


rotY = np.array([[np.cos(Beta)       ,   0   ,    np.sin(Beta) ], 
                 [    0              ,   1   ,        0        ], 
                 [-np.sin(Beta)      ,   0   ,    np.cos(Beta) ]])


rotZ = np.array([[np.cos(Gamma)      ,   -np.sin(Gamma)  ,    0 ], 
                 [np.sin(Gamma)      ,   np.cos(Gamma)   ,    0 ], 
                 [    0              ,        0          ,    1 ]])  
      
        
rotMat   = rotZ @ rotY @ rotX


transMat = np.array([[       0        ],
                     [   camHeight    ],
                     [   real_recede  ]])

        
extMat = np.hstack((rotMat, transMat))


intMat = np.array([[ focalLen/sx    ,        0      ,    ox ],        
                   [      0         ,  focalLen/sy  ,    oy ],
                   [      0         ,        0      ,     1 ]])  
        
        
"""
    4. SHARED MEMORY VARIABLE
"""

shared_memory_receive = "HADA3_CAM_DETECT"
shared_memory_send    = "HADA3_ERP_STOP"

send_mem_size         = 4 * 1

"""
    5. COMMAND VARIABLE
"""
device_name = 'com3'

brake       = 0 
vel_cmd     = 10


"""
    6. DELIVERY VARIABLE
"""
dist_thresh = 2.5



