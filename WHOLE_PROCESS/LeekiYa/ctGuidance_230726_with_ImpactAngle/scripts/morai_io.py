

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
[sys.path.append(i) for i in ['.','..']]
from lib.morai_udp_parser import udp_parser,udp_sender
from lib.gps_util import UDP_GPS_Parser
from lib.imu_util import udp_sensor_parser
from lib.utils import pathReader,findLocalPath,purePursuit,Point
from math import cos,sin,sqrt,pow,atan2,pi

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

    def recv_data_MORAI(self):
        roll  = 0.0
        pitch = 0.0
        yaw   = 0.0
        lin_acc_x = 0.0
        lin_acc_y = 0.0
        lin_acc_z = 0.0
        latitude  = 0.0
        longitude = 0.0
                # ============================================================================
        # Get Data from MORAI : Envoronment
        # ============================================================================
        ## self.status.get_data()와 같은 데이터를 udp통신을 통해 받아와 status_data라는 변수를 정의하여 288,299라인을 보면 carx,y의 좌표게를 정의할 수 있다.
        status_data  = self.status.get_data()
        if self.gps_parser.parsed_data!=None :
            latitude  = self.gps_parser.parsed_data[0]
            longitude = self.gps_parser.parsed_data[1]
        
        if len(status_data) != 0:
            ERP_velocity = status_data[18]
        else: ERP_velocity = 0
        
        # IMU Data Parsing
        if len(self.imu_parser.parsed_data)==10 :
            ori_w = self.imu_parser.parsed_data[0]
            ori_x = self.imu_parser.parsed_data[1]
            ori_y = self.imu_parser.parsed_data[2]
            ori_z = self.imu_parser.parsed_data[3]
            roll, pitch, gamma = euler_from_quaternion(ori_x, ori_y, ori_z, ori_w)
            gamma = gamma * (180/pi)         # [degree]
            yaw = 90 - gamma                 # [degree]

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

        return latitude,longitude,roll,pitch,yaw,ang_vel_x,ang_vel_y,ang_vel_z,ERP_velocity

    def cmd_to_MORAI(self,ctrl_mode,gear,command_velocity,command_steering):
        # ============================================================================
        # Command to ERP
        # ============================================================================
        # ctrl_mode = 1             # 1 : KeyBoard       2 : AutoMode       
        cmd_type  = 2                       # 1 : Throttle       2 : Velocity      3 : Acceleration   
        Gear      = gear            # 1 : (P / parking ) 2 : (R / reverse) 3 : (N / Neutral)  4 : (D / Drive) 5 : (L)

        send_velocity = command_velocity # cmd_type이 2일때 원하는  속도를 넣어준다.
        acceleration = 1             # cmd_type이 3일때 원하는 가속도를 넣어준다.
        
        accel = 0
        brake = 0
     
        steering_angle = command_steering #[degrees]
        # print("steering: {0}, send_velocity: {1}".format(steering_angle, send_velocity))
        # print(f"Yaw: {yaw}")
        self.ctrl_cmd.send_data([ctrl_mode,Gear,cmd_type,send_velocity,acceleration,accel,brake,steering_angle])

