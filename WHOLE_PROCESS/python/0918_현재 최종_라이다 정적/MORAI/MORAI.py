

#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pyinstaller MORAI.py
# ============================================================================
# Module Import
# ============================================================================
from lib.morai_udp_parser import udp_parser,udp_sender
from lib.gps_util import UDP_GPS_Parser
from lib.imu_util import udp_sensor_parser
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

# ============================================================================
# Communication Setting
# ============================================================================
# 보내기용 구조체
class WRITE_DATA(ct.Structure):   # Struct with Lidar
    _fields_ = [("Roll",ct.c_double),("Pitch",ct.c_double),("Yaw",ct.c_double),("Acc_x",ct.c_double),("Acc_y",ct.c_double),("Acc_z",ct.c_double),("Latitude",ct.c_double),("Longitude",ct.c_double),("Distance",ct.c_double*761),("Velocity",ct.c_double)]# ,,("Traffic_Flag",ct.c_int*8)]

# 받기용 구조체  
class READ_DATA(ct.Structure):
    _fields_ = [ ("Steering", ct.c_double), ("Velocity", ct.c_double),("Gear",ct.c_int),("Brake",ct.c_double)]


isExe = os.path.split( os.path.abspath( './../' ) )[1] == 'dist'
if isExe    :
    path = os.path.abspath( './../../' )
else        :
    path = path = os.path.dirname( os.path.abspath( __file__ ) )



with open(os.path.join(path,("params.json")),'r') as fp :
    params = json.load(fp)

params  = params["params"]

user_ip = params["user_ip"]
host_ip = params["host_ip"]

# Sensor port
gps_port   = params["gps_dst_port"]
imu_port   = params["imu_dst_port"]
lidar_port = params["lidar_dst_port"]

# Receive from Simulator (모라이에서 받아올 값)
status_port      = params["vehicle_status_dst_port"]
object_port      = params["object_info_dst_port"]
get_traffic_port = params["get_traffic_dst_port"]

# Command to Simulator    (모라이에 넘겨줄 값)
ctrl_cmd_port              = params["ctrl_cmd_host_port"]
set_traffic_port           = params["set_traffic_host_port"]
traffic_greenlight_setting = params["traffic_greenlight_setting"]

# ============================================================================
# Sensor Setting
# ============================================================================
# Sensor setting
params_lidar = {
    "Range" : 90, #min & max range of lidar azimuths
    "CHANNEL" : 16, #verticla channel of a lidar
    "localIP": user_ip,
    "localPort": lidar_port,
    "Block_SIZE": int(1206) 
}

max_distance = 10

STANLEY_UDP = True

def euler_from_quaternion(x, y, z, w): 
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll  is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw   is rotation around z in radians (counterclockwise) 
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # [rad]

class planner :

    # ============================================================================
    # Communication Variable Definition
    # ============================================================================
    def __init__(self):
        ## Information such as traffic lights, vehicle and sensor in the Simulator can be received as a function of udp parser.
        # data_1 = ctrl_mode, gear,   signed_vel, map_id, accel,        brake,   size_x, size_y, size_z,   overhang,  wheelbase, rear_overhang
        # data_2 = pose_x,    pose_y, pose_z,     roll,   pitch, yaw,   vel_x,   vel_y,  vel_z,  accel_x,  accel_y,   accel_z, steer 
        self.status  = udp_parser(user_ip, status_port,'erp_status')       # 모라이 내 erp 정보 가져오기    , data1 + data2의 정보를 가짐
        self.obj     = udp_parser(user_ip, object_port,'erp_obj')          # 모라이 내 장애물 정보 가져오기 
        self.traffic = udp_parser(user_ip, get_traffic_port,'get_traffic') # 모라이 내 신호등 정보 가져오기

        # 모라이에게 넘겨줌
        self.ctrl_cmd    = udp_sender(host_ip,ctrl_cmd_port,'erp_ctrl_cmd')
        self.set_traffic = udp_sender(host_ip,set_traffic_port,'set_traffic')
        
        # 모라이에서 받아옴
        self.gps_parser = UDP_GPS_Parser(user_ip, gps_port,'GPRMC') 
        self.imu_parser = udp_sensor_parser(user_ip, imu_port,'imu')
        self.udp_lidar  = UDP_LIDAR_Parser(ip=params_lidar["localIP"], port=params_lidar["localPort"], params_lidar=params_lidar)

        self._is_status      = True
        self.is_obj          = False
        self.is_traffic      = False
        self.is_sharedmemory = False

        while not self._is_status :
            if not self.status.get_data() :
                print('No Status Data Cannot run main_loop')
                time.sleep(1)
            else :
                self._is_status=True

    # ============================================================================
    # SharedMemory Open (파이썬과 C 코드 연결을 위함)
    # ============================================================================
    def sharedmemory_open(self):
        # Shared Memory Process
        self.FILE_MAP_ALL_ACCESS  = 0x000F001F
        self.FILE_MAP_READ        = 0x0004
        self.INVALID_HANDLE_VALUE = -1
        self.SHMEMSIZE            = 0x100
        self.PAGE_READWRITE       = 0x04
        self.TRUE  = 1
        self.FALSE = 0

        self.kernel32_dll               = ct.windll.kernel32
        self.msvcrt_dll                 = ct.cdll.msvcrt  # To be avoided

        self.CreateFileMapping          = self.kernel32_dll.CreateFileMappingW
        self.CreateFileMapping.argtypes = (wt.HANDLE, wt.LPVOID, wt.DWORD, wt.DWORD, wt.DWORD, wt.LPCWSTR)
        self.CreateFileMapping.restype  = wt.HANDLE

        self.OpenFileMapping            = self.kernel32_dll.OpenFileMappingW
        self.OpenFileMapping.argtypes   = (wt.DWORD, wt.BOOL, wt.LPCWSTR)
        self.OpenFileMapping.restype    = wt.HANDLE

        self.MapViewOfFile              = self.kernel32_dll.MapViewOfFile
        self.MapViewOfFile.argtypes     = (wt.HANDLE, wt.DWORD, wt.DWORD, wt.DWORD, ct.c_ulonglong)
        self.MapViewOfFile.restype      = wt.LPVOID

        self.memcpy                     = self.msvcrt_dll.memcpy
        self.memcpy.argtypes            = (ct.c_void_p, ct.c_void_p, ct.c_size_t)
        self.memcpy.restype             = wt.LPVOID

        self.UnmapViewOfFile            = self.kernel32_dll.UnmapViewOfFile
        self.UnmapViewOfFile.argtypes   = (wt.LPCVOID,)
        self.UnmapViewOfFile.restype    = wt.BOOL

        self.CloseHandle                = self.kernel32_dll.CloseHandle
        self.CloseHandle.argtypes       = (wt.HANDLE,)
        self.CloseHandle.restype        = wt.BOOL

        self.GetLastError               = self.kernel32_dll.GetLastError
        
        # 파일 이름 선언
        self.wfile_mapping_name_ptr = ct.c_wchar_p("Hils_smdat_ReadData")
        self.rfile_mapping_name_ptr = ct.c_wchar_p("ERP42_smdat_ReadData")

        # 파일 크기 선언
        self.wbyte_len = ct.sizeof(WRITE_DATA)    
        self.rbyte_len = ct.sizeof(READ_DATA)    

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

        ## You can search it to find out the detailed structure of shared memory

    # ============================================================================
    # Functions Used in Main Loop
    # ============================================================================
    def main_loop(self):
        """ Sharedmemory open, Sharedmoemory close 필요 """
        # ============================================================================
        # Status & Object & Traffic Initialization
        # ============================================================================
        roll  = 0.0
        pitch = 0.0
        yaw   = 0.0
        lin_acc_x = 0.0
        lin_acc_y = 0.0
        lin_acc_z = 0.0
        latitude  = 0.0
        longitude = 0.0
        round_distance = 0.0
        flag           = np.zeros(8)      # 1: Traffic 2: 장애인구역 3: 배달A 4: 배달B
        
        # ============================================================================
        # Get Data from MORAI : Envoronment
        # ============================================================================
        ## self.status.get_data()와 같은 데이터를 udp통신을 통해 받아와 status_data라는 변수를 정의하여 288,299라인을 보면 carx,y의 좌표게를 정의할 수 있다.
        status_data  = self.status.get_data()
        obj_data     = self.obj.get_data()
        traffic_data = self.traffic.get_data()

        if len(status_data) != 0:
            ERP_velocity = status_data[18]
        else: ERP_velocity = 0
        # print('------------------------------------------------------')
        # print('position_x : ', status_data[11] )
        # print('position_x : ',status_data[12])
        # print('position_y : ',status_data[13])
        # print('position_z : ',status_data[14])
        # print('heading    : ',status_data[17])    # degree
        # print('velocity   : ',status_data[18])
        # print('------------------------------------------------------')

        # ============================================================================
        # Get Data from MORAI : Sensor
        # ============================================================================
        # Gps Data Parsing 
        if self.gps_parser.parsed_data!=None :
            latitude  = self.gps_parser.parsed_data[0]
            longitude = self.gps_parser.parsed_data[1]
            # print('------------------------------------------------------')
            # print('Lat : {0} , Long : {1}'.format(latitude,longitude))
        # IMU Data Parsing
        if len(self.imu_parser.parsed_data)==10 :
            ori_w = self.imu_parser.parsed_data[0]
            ori_x = self.imu_parser.parsed_data[1]
            ori_y = self.imu_parser.parsed_data[2]
            ori_z = self.imu_parser.parsed_data[3]
            roll, pitch, gamma = euler_from_quaternion(ori_x, ori_y, ori_z, ori_w)
            gamma = gamma * (180/pi)         # [degree]
            yaw = 90 - gamma                # [degree]

            ang_vel_x = self.imu_parser.parsed_data[4]
            ang_vel_y = self.imu_parser.parsed_data[5]
            ang_vel_z = self.imu_parser.parsed_data[6]
            
            lin_acc_x = self.imu_parser.parsed_data[7]
            lin_acc_y = self.imu_parser.parsed_data[8]
            lin_acc_z = self.imu_parser.parsed_data[9]
            # print('------------------------------------------------------')
            # print(' roll:      {0}  pitch:      {1}  yaw:        {2}'.format(round(roll,2),round(pitch,2),round(yaw,2)))
            # print(' ang_vel_x :{0}  ang_vel_y : {1}  ang_vel_z : {2}'.format(round(ang_vel_x,2),round(ang_vel_y,2),round(ang_vel_z,2)))
            # print(' lin_acc_x :{0}  lin_acc_y : {1}  lin_acc_z : {2}'.format(round(lin_acc_x,2),round(lin_acc_y,2),round(lin_acc_z,2)))
            # print('------------------------------------------------------')
            
            ## -180~180으로 표현하기 위한 if 조건문
            if yaw > 180 : yaw = yaw - 360
            elif yaw <-180 : yaw = yaw +360

        # LIDAR Data Parsing
        if self.udp_lidar.is_lidar ==True:

            buf_distance = np.zeros(1800)             ## distance를 위한 버퍼 
            buf_buf_distance = np.zeros(951)          # -95~95 degree, 0.2 resolution
            round_distance = np.zeros(761)            # -95~95 degree, 0.25 resolution
            idx_angle = 0
            # print(self.udp_lidar.Azimuth)
            # print(len(self.udp_lidar.Azimuth))
            Azimuth_buf = pd.DataFrame(self.udp_lidar.Azimuth)
            Distance_buf = pd.DataFrame(self.udp_lidar.Distance[:,1])
            lidar_azimuth  = self.udp_lidar.Azimuth*5 ## 방위각 resolution 
            lidar_distance = self.udp_lidar.Distance
            # print(lidar_distance)
            
            for i in range(3600):                     # 0~359.8 degree, 0.2 resolution : 1800 array
                idx_angle = lidar_azimuth[i].astype(int)
                buf_distance[idx_angle] = lidar_distance[i,1]# 2번 채널
                
            
            buf_buf_distance[:475]  = buf_distance[-475:]
            buf_buf_distance[-476:] = buf_distance[:476]    # -95~95 degree, 0.2 resolution
            
            idx_origin = 0
            idx_round = 0

            ## 버퍼 재정의 -->> normalized distance --> 강진이형한테 자세한 코드의 목적에 대한 설명을 첨부해야할듯
            
            for i in range(190):                            # -95~95 degree, 0.25 resolution
                round_distance[idx_round]   = buf_buf_distance[idx_origin]
                round_distance[idx_round+1] = (buf_buf_distance[idx_origin+1]+buf_buf_distance[idx_origin+2])*1/2
                round_distance[idx_round+2] = (buf_buf_distance[idx_origin+2]+buf_buf_distance[idx_origin+3])*1/2
                round_distance[idx_round+3] = (buf_buf_distance[idx_origin+3]+buf_buf_distance[idx_origin+4])*1/2
                idx_origin+=5
                idx_round+=4
            round_distance[760] = buf_buf_distance[950]

            for i in range(761):       # Normalize in Clustering
                if round_distance[i]>max_distance or round_distance[i]==0:
                    round_distance[i]=max_distance
                
            round_distance = round_distance*500
            round_distance = (ct.c_double*len(round_distance)).from_buffer(round_distance)
        else:   # if there is no data
            round_distance = np.zeros(761)

            for i in range(761):       
                if round_distance[i]>max_distance or round_distance[i]==0:
                    round_distance[i]=max_distance
            
            round_distance = round_distance*500
            round_distance = (ct.c_double*len(round_distance)).from_buffer(round_distance)
        
        # ============================================================================
        # Send Data to C code
        # ============================================================================
        # SharedMemory Write
        ###  현재 simulator의 데이터에서 알고리듬의 c파일로 보내기 위한 Shared memory 
        if self.is_sharedmemory==True:
            
            write_smdat                 = WRITE_DATA()
            write_smdat.Roll            = roll
            write_smdat.Pitch           = pitch
            write_smdat.Yaw             = yaw
            write_smdat.Acc_x           = lin_acc_x
            write_smdat.Acc_y           = lin_acc_y
            write_smdat.Acc_z           = lin_acc_z
            write_smdat.Latitude        = latitude
            write_smdat.Longitude       = longitude
            write_smdat.Distance        = round_distance
            write_smdat.Velocity        = ERP_velocity
            # write_smdat.Traffic_Flag    = flag

            # print(write_smdat.Velocity)
            wmsg_ptr                    = ct.pointer(write_smdat)
            self.memcpy(self.wmapped_view_ptr, wmsg_ptr, self.wbyte_len) # 1, 2번째 파라미터 순서가 이상한 거 같은데..

            read_smdat                  = READ_DATA()
            rmsg_ptr                    = ct.pointer(read_smdat)
            self.memcpy(rmsg_ptr,self.rmapped_view_ptr,self.rbyte_len)

            command_velocity = read_smdat.Velocity    # [km/h]
            command_steering = read_smdat.Steering    # [deg]
            command_gear     = read_smdat.Gear
            command_brake    = read_smdat.Brake



            if   command_gear == 0 : command_gear = 4
            elif command_gear == 1 : command_gear = 3
            elif command_gear == 2 : command_gear = 2
            elif command_gear == 3 : command_gear = 1
            
        else:
            command_velocity = 0    # [km/h]
            command_steering = 0    # [deg]
            command_gear     = 4

        # ============================================================================
        # Command to ERP
        # ============================================================================
        ctrl_mode = 2                       # 1 : KeyBoard       2 : AutoMode       
        cmd_type  = 2                       # 1 : Throttle       2 : Velocity      3 : Acceleration   
        Gear      = command_gear            # 1 : (P / parking ) 2 : (R / reverse) 3 : (N / Neutral)  4 : (D / Drive) 5 : (L)

        send_velocity = command_velocity # cmd_type이 2일때 원하는  속도를 넣어준다.
        acceleration = 0                    # cmd_type이 3일때 원하는 가속도를 넣어준다.
        
        accel = 1
        brake = command_brake
     
        steering_angle = command_steering/28
        #print("steering: {0}, send_velocity: {1}".format(steering_angle, send_velocity))

        self.ctrl_cmd.send_data([ctrl_mode,Gear,cmd_type,send_velocity,acceleration,accel,brake,steering_angle])
    
    # ============================================================================
    # SharedMemory Close (파이썬과 C 코드 연결을 위함)
    # ============================================================================
    def sharedmemory_close(self):
        self.UnmapViewOfFile(self.wmapped_view_ptr)
        self.CloseHandle(self.wmapping_handle)
        self.UnmapViewOfFile(self.rmapped_view_ptr)
        self.CloseHandle(self.rmapping_handle)

      
                        
if __name__ == "__main__":

    simulator = planner()                            ## Python class -> simulator 
    simulator.sharedmemory_open()                  
    ## sharedmemory open [def] --> A function to use shared memory ---  
    ## [Python -- C] --> Method for communication between Python and C 

    time_curr  = 0
    time_cnt   = 0
    time_stime = 0 
    time_ts    = 0.05
    time_final = 5000

    time_start = time.time()
    print("Initialization Complete")

    while True:#(time_stime < time_final) :

        # 모라이로부터 환경, 센서 데이터 받아옴 -> C 코드로 넘겨줌 -> $C 에서 받아오는 듯한 내용이 없는 거 같은데..$ 
        simulator.main_loop()

        # while(1):
        #     time_curr = time.time()
        #     time_del = time_curr - time_start - time_stime
        #     if (time_del > time_ts):
        #         time_cnt   += 1
        #         time_stime =  time_cnt*time_ts
        #         break

    simulator.sharedmemory_close()
    ## sharedmemory close  [def] --> A function to use shared memory ---  
    ## [Python -- C] --> Method for communication between Python and C 
    ## Open - Close 

# =================================================================================
# Print Code
# =================================================================================
     
    # def print(self):
    #     print("ye")
    #     """ main_loop index 모음 용도 """
    #     print('------------------------------------------------------')
    #     print('position_x : ',self.status_data[12])
    #     print('position_y : ',self.status_data[13])
    #     print('position_z : ',self.status_data[14])
    #     print('heading  : ',self.status_data[17])+90    # degree
    #     print('velocity : ',self.status_data[18])
    #     print('------------------------------------------------------')
    #     print('obj_id = ',self.obj_data[0,0])
    #     print('obj_type = ',self.obj_data[0,1])
    #     print('obj_pos_x = ',self.obj_data[0,2])
    #     print('obj_pos_y = ',self.obj_data[0,3])
    #     print('obj_pos_z = ',self.obj_data[0,4])



