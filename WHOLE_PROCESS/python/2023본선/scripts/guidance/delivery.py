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
# import drawnow
import csv
# ----------------------------------------------------------

import sys

from math import *
from sympy import symbols, solve, sqrt

from multiprocessing import shared_memory
from sklearn.cluster import DBSCAN
import keyboard

from HADA import lidar3d_py
from scripts.guidance import *
from project_val import *

class UNIT :
    
    def __init__(self) :
        
        self.D2R            =   pi/180
        self.R2D            =   180/pi
        self.KM2MS          =   0.27777             # km/h  to  m/s
        
        self.LAT2METER      =   110950.59672489
        self.LON2METER      =   88732.3577032982

    def RAD2DEG(self, val) :

        return val * self.R2D

    def DEG2RAD(self, val) :

        return val * self.D2R


class SHAREDMEM :  
    
    def __init__(self) :
        
        self.from_morai_to_main   =   "MORAI_TO_MAIN"
        self.from_main_to_morai   =   "MAIN_TO_MORAI"
        self.from_cam_to_delivery =   "CAM_TO_DELIVERY"                 # camera deep learning process to lidar
        self.from_deliv_to_cam    =   "DELIVERY_TO_CAM"                  # lidar process to camera



"""
    Considers the stop condition with (platform velocity & dist thresh value)
"""

class DELIVERY :

    def __init__(self) :

        self.velodyne               =   lidar3d_py.driver()
        self.velodyne.run()
        self.projector              =   lidar3d_py.projector()

        self.unit                   =   UNIT()
        self.sm                     =   SHAREDMEM()
        self.shm_to_cam             =   shared_memory.SharedMemory(name = self.sm.from_deliv_to_cam , create = True, size = 4 * 1)

        self.cam_middle             =   []
          
        self.is_deliv_control       =   False
        self.is_deliv_brake         =   False
        self.is_A_sign_done         =   False
        self.is_B_sign_done         =   False 
        self.sign_flag              =   0X00

        self.start_time             =   None
        self.brake_start_time       =   None
        
        self.prev_dist              =   0
        self.sign_dist              =   0

        self.print_cnt              =   0
    
        print("delivery mission : Initialization completed...!")
    


    # delivery mission algorithm
    def delivery_mission(self, mission_flag, offset, org_cmd) :
        
        cmd_gear, cmd_velocity, cmd_steer, cmd_brake = 0, 0, 0, 0
        
        self.sign_flag   =  0X00

        if mission_flag == MISSION_DELIVERY_A or mission_flag == MISSION_DELIVERY_B :

            self.receive_from_cam(self.sm.from_cam_to_delivery)          
    
            self.get_sign_dist()                                   
    
            self.sign_stop(offset)                                      
                                
            self.check_restart() 
            
            self.check_flag(mission_flag)         # algorithm operates only when brake command is activated

            self.send_to_cam()  

            self.print_result()

        cmd_gear, cmd_velocity, cmd_steer, cmd_brake = self.control_platform(org_cmd)
        
        # print(cmd_gear, cmd_velocity, cmd_steer, cmd_brake)
        
        return cmd_gear, cmd_velocity, cmd_steer, cmd_brake
     

    def check_flag(self, mission_flag) :

        # [ A sign done | B sign done ] flag update part
        
        if self.is_deliv_brake and mission_flag == MISSION_DELIVERY_A : self.is_A_sign_done = True
        
        if self.is_deliv_brake and mission_flag == MISSION_DELIVERY_B : self.is_B_sign_done = True
        
        # sign flag for camera process update part
        
        if not self.is_A_sign_done and mission_flag == MISSION_DELIVERY_A  :  self.sign_flag = DELIV_A_START
            
        if self.is_A_sign_done and mission_flag == MISSION_DELIVERY_A  :  self.sign_flag = DELIV_A_FINISH
            
        if not self.is_B_sign_done and mission_flag == MISSION_DELIVERY_B  :  self.sign_flag = DELIV_B_START 

        if self.is_B_sign_done and mission_flag == MISSION_DELIVERY_B  :  self.sign_flag = DELIV_B_FINISH   
            
            

    # receive camera object detection infromation
    def receive_from_cam(self, shared_mem_name) :

        self.cam_middle = []

        try :
           
            shm          =  shared_memory.SharedMemory(name = shared_mem_name)
            shared_data  =  np.ndarray((2, ), dtype = 'int', buffer = shm.buf)
            
            self.cam_middle = np.copy(shared_data)

        except FileNotFoundError :

            self.cam_middle = []


    # calculation of sign distance
    def get_sign_dist(self) :

        if len(self.cam_middle) != 0 :

            self.sign_dist = self.projector.PCD(self.velodyne, self.cam_middle[0], self.cam_middle[1], True)

            self.prev_dist = self.sign_dist

            if self.sign_dist == 0  :  self.sign_dist = 1000  

        else : self.sign_dist = 1000


    # control platform with ideal braking distance
    def sign_stop(self, offset) :

        dist_thresh    =  5
        deliv_velocity =  6.0           # need to adjust to the reference velocity in delivery mission

        time_to_brake  =  (dist_thresh + offset) / (deliv_velocity * self.unit.KM2MS)

        if self.sign_dist <= dist_thresh  :  
            
            self.is_deliv_control = True

            if not self.start_time  :  self.start_time = time.time()

        if self.is_deliv_control and not self.is_deliv_brake :

            elapsed_time   =  time.time() - self.start_time

            if elapsed_time >= time_to_brake :

                self.is_deliv_brake   =  True                   # full brake flag update
                self.is_A_sign_done   =  1                      # send A sign done flag to camera process

                self.brake_start_time = time.time()


    def send_to_cam(self) :

        shm = shared_memory.SharedMemory(name = self.sm.from_deliv_to_cam)

        shared_data = np.ndarray((1,), dtype ='int', buffer = shm.buf)

        shared_data[0] = self.sign_flag


    def check_restart(self) :

        if self.is_deliv_brake :

            if self.brake_start_time is None :

                self.brake_start_time = time.time()

            if time.time() - self.brake_start_time >= 6.0 :             # 안전하게 6초 정차 후 출발
                
                # flag initialization
                self.is_deliv_control  =  False
                self.is_deliv_brake    =  False
                
                self.start_time        =  None
                self.brake_start_time  =  None
                
                self.sign_dist         =  1000
 
 
    def control_platform(self, org_cmd) :

        # print(org_cmd)

        if self.is_deliv_control == False and self.is_deliv_brake == False :
            
            return org_cmd[0], org_cmd[1], org_cmd[2], org_cmd[3]
        
        if self.is_deliv_brake == True :
            
            return 1, 0, org_cmd[2], 200                # full brake if delivery sign is near enough
        
        else :
            return org_cmd[0], org_cmd[1], org_cmd[2], org_cmd[3]
        
        

    def print_result(self) :

        self.print_cnt += 1

        if self.print_cnt % 30 == 0 :

            print("----------------------------------------------------------------")
            print(f"CAM MID          :   {self.cam_middle}"                         )
            print(f"SIGN DIST        :   {round(self.sign_dist, 2)}   [m]"          )
            print(f"is_deliv_control :   {self.is_deliv_control}"                   )
            print(f"BRAKE COMMAND    :   {self.is_deliv_brake}"                     )
            print(f"A SIGN DONE      :   {self.is_A_sign_done}"                     )
            print(f"B SIGN DOEN      :   {self.is_B_sign_done}"                     )
            # print(f"SIGN FLAG        :   {self.sign_flag}"                          )
            
            # if self.brake_start_time is not None :
            #     print(f"TIME : {time.time() - self.brake_start_time}")
            