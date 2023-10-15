"""

Path tracking simulation with pure pursuit steering and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)

"""
import numpy as np
import math
import matplotlib.pyplot as plt
import csv
import cubic_spline_planner
import sys
import keyboard
import datetime
import time
from GPS import *
from IMU import *
from serial_node import erpSerial

sys.path.append("./scripts")
sys.path.append("./waypoint")
from morai_io import planner


# Parameters
k = 0.1  # look forward gain
Lfc = 2.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 2.9  # [m] wheel base of vehicle
filename = "waypoint/WP_TRACK0.csv"

LAT2METER = 110950.59672489
LON2METER = 90048.170449268
LON2METER	=   5159243.427952315 * np.pi / 180 # HGU 

def read_csv_file(file_path):
    data = []
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file, delimiter=',')
        for row in csv_reader:
            data.append([float(row[0]), float(row[1]), float(row[2])])
    return data

def waypoint_to_xy(data):# EN to XY 
    wp_x = [(row[1]-data[0][1]) * LON2METER  for row in data]
    wp_y = [(row[0]-data[0][0]) * LAT2METER  for row in data]
    return wp_x , wp_y

def my_xy_pos(lat,lon,yaw ,init_lat, init_lon):
    yaw = -yaw +90
    x = (lon-init_lon) * LON2METER
    y = (lat-init_lat) * LAT2METER

    return x, y , yaw

class TargetCourse:
    
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, px,py):
        
        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [px - icx for icx in self.cx]
            dy = [py - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = np.hypot(self.cx[ind] - px,self.cy[ind] -py)

            while (ind + 1) < len(self.cx):
                
                distance_next_index = np.hypot(self.cx[ind + 1] - px, self.cy[ind + 1] - py)
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind


        # Lf = k * state.v + Lfc  # update look ah
        # lead distance
        Lf =4

        
        # search look ahead target point index
        while Lf > np.hypot(self.cx[ind] - px, self.cy[ind] - py):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1
        self.old_nearest_point_index = ind
        return self.old_nearest_point_index, Lf

class RealTime():
    def __init__(self):
        self.time_cnt = 0
        self.simtime = 0
        self.Ts = 0.01
        self.time_curr = 0
        self.time_prev = 0  
    
     
    
def pure_pursuit_steer_control(px,py,yaw, trajectory, pind):
    ind, Lf = trajectory.search_target_index(px,py)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1
    gamma = yaw * np.pi /180 
    los = math.atan2(ty - py , tx - px )
    # alpha = -(math.atan2(ty - py, tx - px) - yaw * np.pi /180)
    eta = gamma - los
    Kp = 3
    # delta = 2 * WB * math.sin(alpha) / Lf
    delta = 2 * Kp * math.sin(eta) / Lf #*180/ np.pi 
    delta = min(max(delta,-28),28)
    return delta, ind , los , eta

def impact_Angle_steer_control(px,py,yaw,cyaw ,trajectory, pind):
    ind, Lf = trajectory.search_target_index(px,py)
    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1
    gamma = yaw * np.pi /180 
    los = math.atan2(ty - py , tx - px )
    # alpha = -(math.atan2(ty - py, tx - px) - yaw * np.pi /180)
    eta = gamma - los
    c1=2 * 1.1 * 0.095
    c2= 0.095**2
    vel = 20
    tgo = vel  / Lf
    # gamdotc = c1*c2*math.sin(eta) + (c1+1.0)* 10 * math.sin( eta ) / Lf
    lamdot = vel * math.sin( eta ) / Lf
    gamdotc = -(c1 * lamdot + c2 * (los - cyaw[pind]  )) / tgo + 2*lamdot
    delta = gamdotc * 1.4 / vel# *180/ np.pi 
    delta = min(max(delta,-28),28)

    return delta, ind , los , eta

def stanley_control(px,py,yaw,cyaw, trajectory, pind):
    ind, Lf = trajectory.search_target_index(px,py)
    cx,cy = trajectory.cx , trajectory.cy
    # if pind >= ind:
    #     ind = pind

    # if ind < len(trajectory.cx):
    #     tx = trajectory.cx[ind]
    #     ty = trajectory.cy[ind]
    # else:  # toward goal
    #     tx = trajectory.cx[-1]
    #     ty = trajectory.cy[-1]
    #     ind = len(trajectory.cx) - 1
        
    dx = [px - icx for icx in cx]
    dy = [py - icy for icy in cy]
    d = np.hypot(dx, dy)
    ind = np.argmin(d)
    K = 0.1
    vel = 20
    gamma = yaw * np.pi /180 
    los = math.atan2(cy[ind] - py , cx[ind] - px )
    # alpha = -(math.atan2(ty - py, tx - px) - yaw * np.pi /180)
    eta = gamma - los
    
    psi = yaw - cyaw[ind] * 180 / np.pi
    cross_track_err = np.hypot(cx[ind]-px , cy[ind]-py)  * math.sin(eta)
    delta = psi * 0.5 + math.atan2(K * cross_track_err , vel)
    # delta = cross_track_err * K
    delta = min(max(delta,-28),28)  
    # print(cross_track_err)
    return delta ,ind  

def velocity_control(t_index,ck , refvel):
    
    if math.fabs(ck[t_index] )> 0.35:
        vel = 0.5 * refvel
    else:
        vel = refvel
    return vel


    
if __name__ == '__main__':
    mode = "morai"
    rt = RealTime()
    data = read_csv_file(filename)  # read wp data
    #=====================================================================REAL==========================================================================
    if  mode == "real": 
        device = 'com4'
        command  =  erpSerial(device)
        GPS_REAL = GPS_SM()
        GPS_REAL.sharedmemory_open()
        IMU_REAL = IMU_SM()
        IMU_REAL.sharedmemory_open()
        
        wp_x , wp_y = waypoint_to_xy(data) # convert wp data to xy coordinate
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(wp_x, wp_y, ds=0.1) # calculate continuous path
        target_course = TargetCourse(cx,cy)
        
        #Real GPS,IMU 초기값 받기
        init_heading = -121 # 초기헤딩
        lat,lon,h,GPS_qual=GPS_REAL.recv_data()
        yaw = (IMU_REAL.recv_data()  + init_heading)            #초기만 앵글,pi/2는 함수안에서 더해줌 
        x,y,yaw = my_xy_pos(lat,lon,yaw,data[0][0],data[0][1])
        target_ind, _ = target_course.search_target_index(x,y)
        
    #=====================================================================MORAI==========================================================================    
    elif mode == "morai" :   
    #MORAI
        sim = planner()
        wp_x , wp_y = waypoint_to_xy(data) # convert wp data to xy coordinate
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(wp_x, wp_y, ds=0.1) # calculate continuous path
        target_course = TargetCourse(cx,cy)
        lat,lon,yaw = sim.recv_data_MORAI()
        x,y,yaw = my_xy_pos(lat,lon,yaw,data[0][0],data[0][1])
        target_ind, _ = target_course.search_target_index(x,y)
        
        
    exportdata = []
    
    while True :
        rt.time_prev = time.time()

        lat,lon,yaw = sim.recv_data_MORAI()
                #Real GPS,IMU 초기값 받기
        # init_heading = -121 # 초기헤딩
        # lat,lon,h,GPS_qual=GPS_REAL.recv_data()
        # target_ind, _ = target_course.search_target_index(x,y)
        # yaw = (IMU_REAL.recv_data()  + init_heading) 
       
        x,y,yaw = my_xy_pos(lat,lon,yaw,data[0][0],data[0][1])
        # delta, target_ind ,los,eta = pure_pursuit_steer_control(x,y,yaw, target_course, target_ind)
        delta, target_ind ,los,eta = impact_Angle_steer_control(x,y,yaw,cyaw, target_course, target_ind)
        # delta, target_ind = stanley_control(x,y,yaw,cyaw, target_course, target_ind)
        tx = cx[target_ind]
        ty = cy[target_ind]
        velcmd = 20
        vel_curvature = velocity_control(target_ind,ck,velcmd)
        
        print(delta) 
        if mode == "morai":
            sim.cmd_to_MORAI(2,4,velcmd ,delta )
        elif mode =="real":
            command.send_ctrl_cmd(velcmd , round(delta) , 0)
        plt.cla()
        # plt.plot(wp_x,wp_y,'k*')
        plt.plot(cx,cy,'b-')
        plt.plot(x,y,'yo',markersize =8)
        plt.plot(tx,ty,'kx')
        # # plt.xlim((x-5,x+5))
        # # plt.ylim((y-5,y+5))
        plt.grid()
        plt.pause(0.0001)
       
        

        rt.time_cnt +=1
        rt.simtime = rt.time_cnt * rt.Ts
        exportdata.append([delta, los , yaw, eta,rt.simtime , x,y,tx,ty] )
        
        if keyboard.is_pressed('q') :# or target_ind == (len(cx) -1):
            break
        while (time.time() - rt.time_prev )< rt.Ts:
            continue

        
    
    
    #Export Data
    nowdate = datetime.datetime.now()
    exportfile = nowdate.strftime("%Y-%m-%d_%H%M") + "guidance_exp.csv" 
    
    
   
    with open(exportfile , 'w', newline='') as file:
        # CSV writer 생성
        csv_writer = csv.writer(file)

        # 각 행을 CSV 파일에 작성
        for row in exportdata:
            csv_writer.writerow(row)

        


