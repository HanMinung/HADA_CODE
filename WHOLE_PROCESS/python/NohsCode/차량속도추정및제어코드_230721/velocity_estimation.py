"""
Longitydinal velocity estimation

Author:nohkyungha
"""

import numpy as np
import time as tm
import math
import matplotlib.pyplot as plt
from math import sin, cos,asin,sqrt,atan
from serial_node import erpSerial
from imu_driver  import IMU_SM
from GPS import * 


Ts = 0.05 # 50Hz
LAT2METER = 110950.59672489
LON2METER = 90048.170449268
# System Model
x_init = np.zeros((2,1))  # [vel_x , acc_x]
P_init = np.eye(2,2) * 100
F = np.array([[1  , Ts],
              [0  , 1]])
G = np.array([[Ts,  Ts**2 /2 ]])
G = G.T 

H = np.eye(2,2)

q =1 **2
Q = q * G.T @ G

R =np.array([[0.1, 0.0],
              [0.0 , 0.1]])


# Moving average filter

lat_buffer = []
lon_buffer = []

def system_propagation(x,P,u):
    x = F @ x #+ G * u
    P = F @ P @ F.T + Q
    return x , P

def measurement_update(z_v,z_a,x,P):
    z = np.vstack((z_v,z_a))
    y = z - H @ x   # residual
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)

    x = x + K @ y
    P = (np.eye(2,2)- K @ H) @ P
    return x , P

def gps_velocity_est(lat, lon):
    

    
def main():
    device = 'com3'
    command  =  erpSerial(device)
    imu = IMU_SM()
    imu.sharedmemory_open()
    gps = GPS_SM()
    Kp = 5
    Ki = 0.5
    err = 0
    i_err = 0
    ref_vel = 2
    
    sim_time = 0
    cnt =0
    timeline =[]
    final_time = 1000 # sec
    sampling_time = Ts

    # x_post , P_post = measurement_update(x_pri,P_pri)

    velocity_buf = []
    vel_z_buf =[]
    vcmd_buf =[]
    acc_buf = []
    acc_bias =0
    posx =0
    # bias check
    while sim_time < 3:
        prev_time = tm.time()
        acc_bias += imu.recv_data()
        
        while (tm.time() - prev_time) < sampling_time:
            pass

  
        sim_time += sampling_time
        cnt +=1
    
    acc_bias = acc_bias/cnt
    print(acc_bias)
    x_pri , P_pri = system_propagation(x_init,P_init,0)
    
    sim_time = 0
    timeline =[]
    while sim_time < final_time:
        prev_time = tm.time()
        z_a = -(imu.recv_data()-acc_bias)
        z_v = -np.sign(command.gear-1)*command.speed /(3.6*12)  # km/h to m/s
        x_pri , P_pri = system_propagation(x_pri,P_pri,z_a)   # A priori pdf
        # measurement update 
        

        x_post , P_post = measurement_update(z_v,z_a,x_pri,P_pri) # A posteriori pdf
        
        velocity_buf.append(x_post[0] * 3.6)
        vel_z_buf.append(z_v * 3.6)
        acc_buf.append(x_post[1])
        timeline.append(sim_time)


        err = ref_vel - x_post[0]*3.6
        i_err += err *Ts
        vel_cmd = Kp * err +Ki * i_err
        vcmd_buf.append(vel_cmd)
        if command.speed >0.1:
            posx += x_post[0] *Ts
            
        if i_err > 6:
            i_err =6
   
        # next state update
        x_pri = x_post
        P_pri = P_post
        
        plt.cla()
        plt.plot(timeline, velocity_buf)
        plt.plot(timeline, vel_z_buf)
        # plt.plot(timeline, acc_buf)
        plt.legend('velhat, velmea')
        plt.xlim((sim_time-2,sim_time+2))
        plt.ylim((-2,7))
        plt.grid()
        plt.xlabel('Time [s]')
        plt.ylabel('Vel [km/h]')
        plt.pause(0.0001)
        if vel_cmd >=0:
            command.send_ctrl_cmd(int(vel_cmd),0,0)
        else :
            command.send_ctrl_cmd(0,0,int(15 * (1-math.exp(-vel_cmd))))
        # print(f"loop time:{time_del:.3f}")
        while (tm.time() - prev_time) < sampling_time:
            pass
        time_del  =tm.time() - prev_time
  
        sim_time += sampling_time
        # command.send_ctrl_cmd(int(5),0,0)
        print(posx)

    
    imu.sharedmemory_close()
        





if __name__ == '__main__':

    main()