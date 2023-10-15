import math
from math import *
import numpy as np

D2R            = pi/180
R2D            = 180/pi

Alphadeg        =  106
Alpha           =  Alphadeg * D2R
Beta            =  -1 * D2R
Gamma           =  0 * D2R

cam_height      =  0.92
cam_recede      =  0.73
focal_length    =  0.00367
img_width       =  640
img_height      =  480
fov_X           =  60.92 * pi/180
fov_Y           =  53.1432  * pi/180
center_X        =  img_width/2                                              
center_Y        =  img_height/2
scaling_X       =  focal_length * math.tan(0.5 * fov_X)/(0.5 * img_width);      
scaling_Y       =  focal_length * math.tan(0.5 * fov_Y)/(0.5 * img_height);   

real_recede     =  sqrt(cam_height**2 + cam_recede**2) * cos(atan(cam_height/cam_recede) - (Alphadeg - 90) * D2R)
real_height     =  sqrt(cam_height**2 + cam_recede**2) * sin(atan(cam_height/cam_recede) - (Alphadeg - 90) * D2R)

roation_X = np.array([  [1 ,          0        ,         0          ], 
                        [0 ,   np.cos(Alpha)   ,   -np.sin(Alpha)   ], 
                        [0 ,   np.sin(Alpha)   ,    np.cos(Alpha)   ]])   

rotation_Y = np.array([[ np.cos(Beta)         ,        0     ,    np.sin(Beta)], 
                       [     0                ,        1     ,        0       ], 
                       [-np.sin(Beta)         ,        0     ,    np.cos(Beta)]])

rotation_Z = np.array([[np.cos(Gamma)         ,   -np.sin(Gamma)      ,   0 ], 
                       [np.sin(Gamma)         ,    np.cos(Gamma)      ,   0 ], 
                       [    0                 ,        0              ,   1 ]])    

wide_intrinsic_mat = np.array([ [280.38650513 ,        0        ,        322.35003847   ],
                                [  0          ,    269.07183838 ,        226.76398398   ],
                                [  0          ,        0        ,                1      ]])

rotation_mat   = rotation_Z @ rotation_Y @ roation_X


translation_mat = np.array([[      0       ],
                            [  real_height  ],
                            [  real_recede  ]]) 

extrinsic_mat = np.hstack((rotation_mat, translation_mat))


intrinsic_mat = np.array([[focal_length/scaling_X ,       0                   ,  center_X ],            
                          [0                      ,  focal_length/scaling_Y   ,  center_Y ],
                          [0                      ,       0                   ,   1       ]])