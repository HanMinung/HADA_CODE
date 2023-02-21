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


# 보내기용 구조체
class WRITE_DATA(ct.Structure):   # Struct with Lidar
    # _fields_ = [("Yaw",ct.c_double),("Latitude",ct.c_double),("Longitude",ct.c_double),("Distance",ct.c_double*761),("Traffic_Flag",ct.c_int*8)]
    _fields_ = [("Distance",ct.c_double*761)]

# 받기용 구조체
class READ_DATA(ct.Structure):
    # _fields_ = [("Velocity", ct.c_int), ("Steering", ct.c_double), ("Gear",ct.c_int)]
    _fields_ = [("Velocity", ct.c_double), ("Steering", ct.c_double)]

path = os.path.dirname( os.path.abspath( __file__ ) )

with open(os.path.join(path,("params.json")),'r') as fp :
    params = json.load(fp)

params=params["params"]

user_ip = params["user_ip"]
host_ip = params["host_ip"]

# Sensor port
lidar_port = params["lidar_dst_port"]

# Receive from Simulator
# status_port = params["vehicle_status_dst_port"]
# object_port =params["object_info_dst_port"]
# get_traffic_port=params["get_traffic_dst_port"]

# # Command to Simulator
ctrl_cmd_port = params["ctrl_cmd_host_port"]
# set_traffic_port=params["set_traffic_host_port"]
# traffic_greenlight_setting = params["traffic_greenlight_setting"]

# Sensor setting
params_lidar = {
    "Range" : 90, #min & max range of lidar azimuths
    "CHANNEL" : 16, #verticla channel of a lidar
    "localIP": user_ip,
    "localPort": lidar_port,
    "Block_SIZE": int(1206)
}
1
# need update 
lidar_max = 6

class planner :

    def __init__(self):
        # Information such as traff%ic lights, vehicle and sensor in the Simulator can be received as a function of udp parser. 
        # self.status=udp_parser(user_ip, status_port,'erp_status')

        self.udp_lidar = UDP_LIDAR_Parser(ip=params_lidar["localIP"], port=params_lidar["localPort"], params_lidar=params_lidar)
        self.ctrl_cmd = udp_sender(host_ip,ctrl_cmd_port,'erp_ctrl_cmd')

        # self._is_status=False
        # self.is_obj=False
        # self.is_traffic=False
        # self.is_sharedmemory = False
        # self.traffic_light = 16

        # while not self._is_status :
        #     if not self.status.get_data() :
        #         print('No Status Data Cannot run main_loop')
        #         time.sleep(1)
        #     else :
        #         self._is_status=True



    def sharedmemory_open(self):
        # Shared Memory Process
        self.FILE_MAP_ALL_ACCESS = 0x000F001F
        self.FILE_MAP_READ = 0x0004
        self.INVALID_HANDLE_VALUE = -1
        self.SHMEMSIZE = 0x100
        self.PAGE_READWRITE = 0x04
        self.TRUE  = 1
        self.FALSE = 0

        self.kernel32_dll = ct.windll.kernel32
        self.msvcrt_dll = ct.cdll.msvcrt  # To be avoided

        self.CreateFileMapping = self.kernel32_dll.CreateFileMappingW
        self.CreateFileMapping.argtypes = (wt.HANDLE, wt.LPVOID, wt.DWORD, wt.DWORD, wt.DWORD, wt.LPCWSTR)
        self.CreateFileMapping.restype = wt.HANDLE

        self.OpenFileMapping = self.kernel32_dll.OpenFileMappingW
        self.OpenFileMapping.argtypes = (wt.DWORD, wt.BOOL, wt.LPCWSTR)
        self.OpenFileMapping.restype = wt.HANDLE

        self.MapViewOfFile = self.kernel32_dll.MapViewOfFile
        self.MapViewOfFile.argtypes = (wt.HANDLE, wt.DWORD, wt.DWORD, wt.DWORD, ct.c_ulonglong)
        self.MapViewOfFile.restype = wt.LPVOID

        self.memcpy = self.msvcrt_dll.memcpy
        self.memcpy.argtypes = (ct.c_void_p, ct.c_void_p, ct.c_size_t)
        self.memcpy.restype = wt.LPVOID

        self.UnmapViewOfFile = self.kernel32_dll.UnmapViewOfFile
        self.UnmapViewOfFile.argtypes = (wt.LPCVOID,)
        self.UnmapViewOfFile.restype = wt.BOOL

        self.CloseHandle = self.kernel32_dll.CloseHandle
        self.CloseHandle.argtypes = (wt.HANDLE,)
        self.CloseHandle.restype = wt.BOOL

        self.GetLastError = self.kernel32_dll.GetLastError
        
        #------------------------------------------------------------------------------------------------------------
        # 파일 이름 선언
        self.wfile_mapping_name_ptr = ct.c_wchar_p("Hils_smdat_ReadData")  # python -> C
        self.rfile_mapping_name_ptr = ct.c_wchar_p("ERP42_smdat_ReadData") # c-> python

        # 파일 크기 선언
        self.wbyte_len = ct.sizeof(WRITE_DATA)    
        self.rbyte_len = ct.sizeof(READ_DATA)    

        # class Definition
        # line 21 File structure to exchange data 
        # line 25 File structure to exchange data     


        # w파일 맵핑 및 맵핑 객체 선언
        self.wmapping_handle = self.CreateFileMapping(self.INVALID_HANDLE_VALUE,0, self.PAGE_READWRITE, 0, self.wbyte_len, self.wfile_mapping_name_ptr)
        if not self.wmapping_handle:
            print("Could not open file mapping object: {:d}".format(self.GetLastError()))
            raise ct.WinError()

        self.wmapped_view_ptr = self.MapViewOfFile(self.wmapping_handle, self.FILE_MAP_ALL_ACCESS, 0, 0, self.wbyte_len)
        if not self.wmapped_view_ptr:
            print("Could not map view of file: {:d}".format(self.GetLastError()))
            self.CloseHandle(self.wmapping_handle)
            raise ct.WinError()

        # r파일 맵핑 및 맵핑 객체 선언
        self.rmapping_handle = self.CreateFileMapping(self.INVALID_HANDLE_VALUE,0, self.PAGE_READWRITE, 0, self.rbyte_len, self.rfile_mapping_name_ptr)
        if not self.rmapping_handle:
            print("Could not open file mapping object: {:d}".format(self.GetLastError()))
            raise ct.WinError()

        self.rmapped_view_ptr = self.MapViewOfFile(self.rmapping_handle, self.FILE_MAP_ALL_ACCESS, 0, 0, self.rbyte_len)
        if not self.rmapped_view_ptr:
            print("Could not map view of file: {:d}".format(self.GetLastError()))
            self.CloseHandle(self.rmapping_handle)
            raise ct.WinError()
        
        self.is_sharedmemory = True

        # You can search it to find out the detailed structure of shared memory


    def main_loop(self):
        """ Sharedmemory open, Sharedmoemory close 필요 """
        # #-----------------------------------------------------------
        # # Status & Object & Traffic Initialization
        round_distance = 0.0

        # #-----------------------------------------------------------
        # #----------------------------Sensor data parsing -----------
        # # Sensor Data

        # LIDAR Data Parsing
        if self.udp_lidar.is_lidar ==True:

            buf_distance = np.zeros(1800)             ## distance값 저장을 위한 buffer
            buf_buf_distance = np.zeros(951)          # -95~95 degree, 0.2 resolution
            round_distance = np.zeros(761)            # -95~95 degree, 0.25 resolution
            idx_angle = 0
            # print(self.udp_lidar.Azimuth)
            # print(len(self.udp_lidar.Azimuth))
            # Azimuth_buf = pd.DataFrame(self.udp_lidar.Azimuth)
            # Distance_buf = pd.DataFrame(self.udp_lidar.Distance[:,1])
            # Distance_buf.to_csv("Export_Distance.csv")
            # Azimuth_buf.to_csv("Export_Azimuth.csv")
            Azimuth_buf = pd.DataFrame(self.udp_lidar.Azimuth)
            Distance_buf = pd.DataFrame(self.udp_lidar.Distance[:,1])
            Distance_buf.to_csv("Export_Distance.csv")
            Azimuth_buf.to_csv("Export_Azimuth.csv")
            lidar_azimuth  = self.udp_lidar.Azimuth*5 ## 방위각 resolution 
            lidar_distance = self.udp_lidar.Distance
            # print(lidar_azimuth)

            
            for i in range(3600):                     # 0~359 .8 degree, 0.2 resolution : 1800 array
                idx_angle = lidar_azimuth[i].astype(int)
                buf_distance[idx_angle] = lidar_distance[i,1]
                # 2번 채널
            
            buf_buf_distance[:475]  = buf_distance[-475:]
            buf_buf_distance[-476:] = buf_distance[:476]    # -95~95 degree, 0.2 resolution

            idx_origin = 0
            idx_round = 0

            ## 버퍼 재정의 -->> normalized distance --> 강진이형한테 자세한 코드의 목적에 대한 설명을 첨부해야할듯
            
            #                           DBSCAN clustering algorithm
            
            #   1. core point를 기준으로 epsilon내에 점이 n개 이상 있으면 군집으로 분류하는 알고리즘
            #   2. 가장 가까운 두 군집의 평균점을 구한다.
            #   3. 두 군집 중심점의 평균점과 원점을 잇는 직선의 y축과 이루는 각도를 구해 조향 명령을 구한다.  
            
            for i in range(190):                            # -95~95 degree, 0.25 resolution
                round_distance[idx_round]   = buf_buf_distance[idx_origin]
                round_distance[idx_round+1] = (buf_buf_distance[idx_origin+1]+buf_buf_distance[idx_origin+2])*1/2
                round_distance[idx_round+2] = (buf_buf_distance[idx_origin+2]+buf_buf_distance[idx_origin+3])*1/2
                round_distance[idx_round+3] = (buf_buf_distance[idx_origin+3]+buf_buf_distance[idx_origin+4])*1/2
                idx_origin+=5
                idx_round+=4
            round_distance[760] = buf_buf_distance[950]

            for i in range(761):       # Normalize in Clustering
                if round_distance[i]>lidar_max or round_distance[i]==0:
                    round_distance[i]=lidar_max
            
            round_distance = round_distance*500
            round_distance = (ct.c_double*len(round_distance)).from_buffer(round_distance)
        else:   # if there is no data
            round_distance = np.zeros(761)

            for i in range(761):       
                if round_distance[i]>lidar_max or round_distance[i]==0:
                    round_distance[i]=lidar_max
            
            round_distance = round_distance*500
            round_distance = (ct.c_double*len(round_distance)).from_buffer(round_distance)
        
        #-----------------------------------------------------------
        #    SharedMemory Write

        ###  현재 simulator의 데이터에서 알고리듬의 c파일로 보내기 위한 Shared memory 
        if self.is_sharedmemory==True:
            
            write_smdat = WRITE_DATA()
            write_smdat.Distance        = round_distance

            wmsg_ptr = ct.pointer(write_smdat)
            self.memcpy(self.wmapped_view_ptr, wmsg_ptr, self.wbyte_len) 

            read_smdat = READ_DATA()
            rmsg_ptr = ct.pointer(read_smdat)
            self.memcpy(rmsg_ptr,self.rmapped_view_ptr,self.rbyte_len)

            command_velocity = read_smdat.Velocity    # [km/h]
            command_steering = read_smdat.Steering    # [deg]
            # command_gear     = read_smdat.Gear
            command_gear     = 0

            if command_gear == 0  : command_gear = 4
            elif command_gear == 1: command_gear = 3
            elif command_gear == 2: command_gear = 2
            elif command_gear == 3: command_gear = 1
            
        else:
            print("no shared memory")
            command_velocity = 20    # [km/h]
            command_steering = 0    # [deg]
            command_gear     = 4

        # --------------------------------------------------------- 
        #   Command value to ERP42 
        ctrl_mode = 2       # 2 = AutoMode / 1 = KeyBoard
        Gear = command_gear # 1 : (P / parking ) 2 : (R / reverse) 3 : (N / Neutral)  4 : (D / Drive) 5 : (L)
        cmd_type = 2        # 1 : Throttle  /  2 : Velocity  /  3 : Acceleration   

        send_velocity = command_velocity    #cmd_type이 2일때 원하는 속도를 넣어준다.
        acceleration = 0                    # cmd_type이 3일때 원하는 가속도를 넣어준다.
        
        accel=1
        brake=0
    
        steering_angle=command_steering/28
        #print("steering: {0}, send_velocity: {1}".format(steering_angle, send_velocity))
        # ERP 명령 넣는 코드
        self.ctrl_cmd.send_data([ctrl_mode,Gear,cmd_type,send_velocity,acceleration,accel,brake,steering_angle])
        # if not len(traffic_data) == 0 :
        #     self.set_traffic.send_data([traffic_data[0],16])

    def sharedmemory_close(self):
        self.UnmapViewOfFile(self.wmapped_view_ptr)
        self.CloseHandle(self.wmapping_handle)
        self.UnmapViewOfFile(self.rmapped_view_ptr)
        self.CloseHandle(self.rmapping_handle)

    def print(self):
            print("")
            """ main_loop index 모음 용도 """
            # print('------------------------------------------------------')
            # print('position_x : ',status_data[12])
            # print('position_y : ',status_data[13])
            #print('position_z : ',status_data[14])
            # print('heading  : ',status_data[17])+90    # degree
            # print('velocity : ',status_data[18])
            # print('------------------------------------------------------')
            # print('obj_id = ',obj_data[0,0])
            # print('obj_type = ',obj_data[0,1])
            # print('obj_pos_x = ',obj_data[0,2])
            # print('obj_pos_y = ',obj_data[0,3])
            # print('obj_pos_z = ',obj_data[0,4])
            # print('------------------------------------------------------')
            #print('Lat : {0} , Long : {1}'.format(latitude,longitude))
            # print('------------------------------------------------------')
            # print(' ori_w:{0}  ori_x {1}  ori_y {2}  ori_z {3}'.format(round(imu_parser.parsed_data[0],2),round(imu_parser.parsed_data[1],2),round(imu_parser.parsed_data[2],2),round(imu_parser.parsed_data[3],2)))
            # print(' ang_vel_x :{0}  ang_vel_y : {1}  ang_vel_z : {2} '.format(round(imu_parser.parsed_data[4],2),round(imu_parser.parsed_data[5],2),round(imu_parser.parsed_data[6],2)))
            # print(' lin_acc_x :{0}  lin_acc_y : {1}  lin_acc_z : {2} '.format(round(imu_parser.parsed_data[7],2),round(imu_parser.parsed_data[8],2),round(imu_parser.parsed_data[9],2)))
            # print('------------------------------------------------------')
            
    
if __name__ == "__main__":

    simulator=planner()                            ## Python class -> simulator 
    simulator.sharedmemory_open()                   
    ## sharedmemory open [def] --> A functiona to use shared memory ---  
    ## [Python -- C] --> Method for communication between Python and C 
    # simulator.write_traffic_sign()

    time_curr = 0
    time_cnt = 0
    time_stime = 0
    time_ts = 0.02
    time_final = 100

    time_start = time.time()
    while (time_stime < time_final) :
    # while(1):
        simulator.main_loop()
        
        # print(time_stime)
        while(1):
            time_curr = time.time()
            time_del = time_curr - time_start - time_stime
            if (time_del > time_ts):
                time_cnt+=1
                time_stime = time_cnt*time_ts
                break

    # simulator.sharedmemory_close()
    ## sharedmemory close  [def] --> A function to use shared memory ---  
    ## [Python -- C] --> Method for communication between Python and C 
    ## Open - Close 
 
