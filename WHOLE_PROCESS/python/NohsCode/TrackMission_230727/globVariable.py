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

Alphadeg       = 105
Alpha          = Alphadeg * D2R
Beta           = 0 * D2R
Gamma          = 0 * D2R

camHeight      = 0.95
camRecede      = 0.77
focalLen       = 0.00367
imgWidth       = 640
imgHeight      = 480
fovX           = 90 * pi/180
fovY           = 70 * pi/180
ox             = imgWidth/2                                              
oy             = imgHeight/2
sx             = focalLen * math.tan(0.5 * fovX)/(0.5 * imgWidth);      
sy             = focalLen * math.tan(0.5 * fovY)/(0.5 * imgHeight);   

realRecede     = sqrt(camHeight**2 + camRecede**2) * cos(atan(camHeight/camRecede) - (Alphadeg - 90) * D2R)
realHeight     = sqrt(camHeight**2 + camRecede**2) * sin(atan(camHeight/camRecede) - (Alphadeg - 90) * D2R)

rotX = np.array([[1 ,          0             ,              0     ], 
                      [0 ,   np.cos(Alpha)   ,   -np.sin(Alpha)   ], 
                      [0 ,   np.sin(Alpha)   ,    np.cos(Alpha)   ]])   

rotY = np.array([[ np.cos(Beta)         ,        0     ,    np.sin(Beta)], 
                 [     0                ,        1     ,        0       ], 
                 [-np.sin(Beta)         ,        0     ,    np.cos(Beta)]])

rotZ = np.array([[np.cos(Gamma)         ,   -np.sin(Gamma)      ,   0 ], 
                 [np.sin(Gamma)         ,    np.cos(Gamma)      ,   0 ], 
                 [    0                 ,        0              ,   1 ]])    


rotMat   = rotZ @ rotY @ rotX


transMat = np.array([[      0       ],
                     [  realHeight  ],
                     [  realRecede  ]]) 

extMat = np.hstack((rotMat, transMat))


intMat = np.array([[focalLen/sx ,       0        ,  ox ],            
                   [0           ,  focalLen/sy   ,  oy ],
                   [0           ,       0        ,   1 ]])  


# command variable  -------------------------------------------------

device    = 'com4'
maxsteerR = 28
maxsteerL = -28
velCMD    = 0