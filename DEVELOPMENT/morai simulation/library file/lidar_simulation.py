#!/usr/bin/env python
# -*- coding: utf-8 -*-
from lib.morai_udp_parser import udp_parser,udp_sender
# from lib.gps_util import UDP_GPS_Parser
# from lib.imu_util import udp_sensor_parser
from lib.lidar_util import UDP_LIDAR_Parser
from lib.utils import pathReader,findLocalPath,purePursuit,Point
from math import cos,sin,sqrt,pow,atan2,pi

import pandas as pd
import sys
import numpy as np
import time
import threading
import math
import msvcrt
import ctypes as ct
import ctypes.wintypes as wt
import os,json
from multiprocessing import shared_memory

from module import *


path = os.path.dirname( os.path.abspath( __file__ ) )

with open(os.path.join(path,("params.json")),'r') as fp :
    params = json.load(fp)

params=params["params"]

user_ip = params["user_ip"]
host_ip = params["host_ip"]

lidar_port = params["lidar_dst_port"]

ctrl_cmd_port = params["ctrl_cmd_host_port"]

params_lidar = {
    "Range" : 90, #min & max range of lidar azimuths
    "CHANNEL" : 16, #verticla channel of a lidar
    "localIP": user_ip,
    "localPort": lidar_port,
    "Block_SIZE": int(1206)
}

# need update 
lidar_max = 9

class planner :

    def __init__(self):

        self.udp_lidar = UDP_LIDAR_Parser(ip=params_lidar["localIP"], port=params_lidar["localPort"], params_lidar=params_lidar)
        self.ctrl_cmd = udp_sender(host_ip,ctrl_cmd_port,'erp_ctrl_cmd')
        
        self.is_sharedmemory    = False
        self.shm_to_main        = None
        self.shm_to_main_name   = "MORAI_TO_MAIN"
        self.shm_from_main      = "MAIN_TO_MORAI"

        
        self.command_steering   = 0      # [deg]
        self.command_gear       = 4
        self.command_mode       = 0
        
        self.speed              = 0
        self.steer              = 0
        self.brake              = 0
        
        self.cnt                = 0
        

    def sharedmemory_open(self):
        
        self.shm_to_main  = shared_memory.SharedMemory(name = self.shm_to_main_name, create = True, size = 8 * 761)
        
        self.is_sharedmemory = True
        
        
    def main_loop(self, mode):
        
        round_distance = 0.0

        if self.udp_lidar.is_lidar ==True:

            buf_distance     = np.zeros(1800)          
            buf_buf_distance = np.zeros(951)       
            round_distance   = np.zeros(761)         
            idx_angle        = 0
            Azimuth_buf      = pd.DataFrame(self.udp_lidar.Azimuth)
            Distance_buf     = pd.DataFrame(self.udp_lidar.Distance[:,1])
            Distance_buf.to_csv("Export_Distance.csv")
            Azimuth_buf.to_csv("Export_Azimuth.csv")
            lidar_azimuth    = self.udp_lidar.Azimuth*5  
            lidar_distance   = self.udp_lidar.Distance

            for i in range(3600):                   
                
                idx_angle = lidar_azimuth[i].astype(int)
                buf_distance[idx_angle] = lidar_distance[i,1]
            
            buf_buf_distance[:475]  = buf_distance[-475:]
            buf_buf_distance[-476:] = buf_distance[:476]    # -95~95 degree, 0.2 resolution

            idx_origin = 0
            idx_round  = 0
            
            for i in range(190): 
                
                round_distance[idx_round]   = buf_buf_distance[idx_origin]
                round_distance[idx_round+1] = (buf_buf_distance[idx_origin+1]+buf_buf_distance[idx_origin+2])*1/2
                round_distance[idx_round+2] = (buf_buf_distance[idx_origin+2]+buf_buf_distance[idx_origin+3])*1/2
                round_distance[idx_round+3] = (buf_buf_distance[idx_origin+3]+buf_buf_distance[idx_origin+4])*1/2
                idx_origin +=5
                idx_round  +=4
                
            round_distance[760] = buf_buf_distance[950]

            for i in range(761):      
                
                if round_distance[i]>lidar_max or round_distance[i]==0:
                    round_distance[i]=lidar_max
                
            round_distance = round_distance * 500
            round_distance = (ct.c_double * len(round_distance)).from_buffer(round_distance)
        
        else:   

            round_distance = np.zeros(761)

            for i in range(761):   
                    
                if round_distance[i] > lidar_max or round_distance[i] == 0:
                    round_distance[i] = lidar_max
            
            round_distance = round_distance * 500
            round_distance = (ct.c_double*len(round_distance)).from_buffer(round_distance)
        

        if self.is_sharedmemory == True :
            
            try :
                
                shm = shared_memory.SharedMemory(name = self.shm_to_main_name)
                shared_data = np.ndarray((761,), dtype ='double', buffer = shm.buf)
                
                for Idx in range (len(round_distance)) :
                    
                    shared_data[Idx] = round_distance[Idx]
        
            except FileNotFoundError :
            
                print("cannot send sm data to 'MAIN'")
            
            self.command_gear = 0

            if   self.command_gear  ==  0  : self.command_gear = 4
            elif self.command_gear  ==  1  : self.command_gear = 3
            elif self.command_gear  ==  2  : self.command_gear = 2
            elif self.command_gear  ==  3  : self.command_gear = 1
            
        else:
            
            print("No shared memory data ...!")
            self.speed            = 100         # [km/h]
            self.command_steering = 0           # [deg]
            self.command_gear     = 4

        self.drive_mode_selection(mode)

        self.receive_command()

        self.command_to_platform()
    
    
    
    def drive_mode_selection(self, mode) :
        
        if mode == "auto"    :  self.control_mode = 2
        if mode == "manual"  :  self.control_mode = 1
        
        
        
    def receive_command(self) :
        
        try :
            
            shm         = shared_memory.SharedMemory(name = self.shm_from_main)
            shared_data = np.ndarray((3, ), dtype ='int', buffer = shm.buf) 
                        
            self.steer  = int(shared_data[0]) / 28
            self.speed  = int(shared_data[1]) * 10
            self.brake  = int(shared_data[2])

            
        except FileNotFoundError : pass
    
    
    def command_to_platform(self) :
                                                    #  |-----------------------------------------------------------------------------------------|
        ctrl_mode     =  self.control_mode          #  |      2 : AutoMode       1 : KeyBoard                                                    |
        Gear          =  self.command_gear          #  |      1 : (P / parking ) 2 : (R / reverse) 3 : (N / Neutral)  4 : (D / Drive) 5 : (L)    |
        cmd_type      =  2                          #  |      1 : Throttle       2 : Velocity      3 : Acceleration                              |
                                                    #  |-----------------------------------------------------------------------------------------|
        acceleration  =  0                 
        accel         =  1
        
        self.ctrl_cmd.send_data([ctrl_mode, Gear, cmd_type, self.speed, acceleration, accel, self.brake, self.steer])
        

    def sharedmemory_close(self):
        
        self.shm_to_main.unlink()
            
    
if __name__ == "__main__":

    realtime   =  REALTIME()
    simulator  =  planner()                   
    simulator.sharedmemory_open()                  

    time_start =  time.time()
    
    while (realtime.time_stime < realtime.time_final) :

        simulator.main_loop(sys.argv[1])
        
        while(1):
            
            time_curr  =  time.time()
            
            time_del = time_curr - time_start - realtime.time_stime
            
            if (time_del > realtime.time_ts):
                
                realtime.time_cnt += 1
                time_stime = realtime.time_cnt * realtime.time_ts
                break

    simulator.sharedmemory_close()

