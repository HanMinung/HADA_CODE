



import numpy as np
import time 
import math
from scripts.guidance.guidance import *
from math import cos,sin

class Navigation:
    def __init__(self,px,py):
        self.Ts = 0.05
        Ts =self.Ts
        self.X = np.array([[px],[py],[0],[0], [0],[0]])
        self.F = np.array([[1 , 0 , Ts, 0 , 0.5*Ts**2 , 0         ],
                           [0 , 1 , 0 , Ts, 0,          0.5*Ts**2],
                           [0 , 0 , 1 , 0,  Ts,         0 ],
                           [0 , 0 , 0 , 1,  0,          Ts],
                           [0 , 0,  0,  0,  1,          0],
                           [0 , 0,  0,  0,  0,          1]])
        self.u = np.array([[0],
                           [0],
                           [0],
                           [0],
                           [0],
                           [0]])
        self.P = np.eye(6,6)
        self.Q = np.array([[1/4 * Ts**4,0 ,           1/2 *Ts**3 , 0 ,         0.5*Ts**2 , 0         ],
                           [0          ,1/4 * Ts**4 , 0 ,          1/2 *Ts**3, 0,          0.5*Ts**2],
                           [1/2 *Ts**3 ,0 ,           Ts**2 ,      0,          Ts,         0 ],
                           [0 ,         1/2 *Ts**3,   0 ,          Ts**2,      0,          Ts],
                           [0.5*Ts**2 , 0,            Ts,          0,          1,          0],
                           [0 ,         0.5*Ts**2,    0,           Ts,         0,          1]]) * 0.001
        self.R = np.eye(2,2) *0.1
        self.H = np.array([[1,0,0,0,0,0],
                           [0,1,0,0,0,0]])
        
    def estimation(self,ax_meas, ay_meas,x_meas,y_meas,yaw):
        self.u[0][0] = ax_meas * math.cos(-yaw * np.pi/180) - ay_meas * math.sin(-yaw * np.pi/180) * 0.5 * self.Ts**2
        self.u[1][0] = -ax_meas * math.sin(-yaw * np.pi/180) + ay_meas * math.cos(-yaw * np.pi/180) * 0.5 * self.Ts**2
        self.u[2][0] = ax_meas * math.cos(-yaw * np.pi/180) - ay_meas * math.sin(-yaw * np.pi/180) * self.Ts
        self.u[3][0] = -ax_meas * math.sin(-yaw * np.pi/180) + ay_meas * math.cos(-yaw * np.pi/180) * self.Ts
        #system propagation
        # print(self.u[2][0] ,self.u[3][0] )
        print(ax_meas,ay_meas)
        self.X = self.F @ self.X #+  self.u
        self.P = self.F @ self.P @ self.F.T  + self.Q
        # print(np.shape(self.X))
        # measurement update
        z = np.array([[x_meas],[y_meas]])
        K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
        self.X = self.X + K @ (z - self.H @ self.X)
        self.P = self.P - K @ self.H @ self.P 
        x_hat = self.X[0][0]
        y_hat = self.X[1][0]
        return x_hat,y_hat
    

            
class YawUpdate(): # Yaw Update 9-29-금 수정
    def __init__(self,x,y) -> None:
        self.prev_x ,self.prev_y = x,y
        self.est_yaw_buf = [0 for i in range(14)]
        self.minho = 0.0
    
    def update_yaw(self, x,y ,yaw):
        var = np.inf
        if np.hypot(x - self.prev_x , y - self.prev_y) > 1: 
            estim_yaw = 90 - np.arctan2(y -  self.prev_y , x -  self.prev_x) * 180 /np.pi
            
            self.est_yaw_buf[0] = estim_yaw
            
            copy_est_yaw_buf = np.copy(self.est_yaw_buf)
            
            self.prev_x ,self.prev_y = x,y
            
            var = np.var(self.est_yaw_buf)
            
            # print(var)
            # if var < 30 and not (0 in self.est_yaw_buf):
            #     meanyaw = np.mean(self.est_yaw_buf)
            #     self.minho  =estim_yaw
            #     print(meanyaw)
            #     yaw_bias = self.minho - yaw
            #     # print("fgh")
            #     return yaw_bias   
            
            for i in range(1,14):
                self.est_yaw_buf[i] = copy_est_yaw_buf[i-1]
                
        if var < 2 and not (0 in self.est_yaw_buf):
            meanyaw = np.mean(self.est_yaw_buf)
            self.minho  = meanyaw
            # print(meanyaw)
            yaw_bias = self.minho - yaw
            # print("fgh")
            return yaw_bias       
        else:
            return 0
        
        
        
        
class VelFilter: # 09_29 추가

    def __init__(self,lat,lon):
        self.residual = 0
        ts= 0.05
        beta = 0.67      
        Lat = lat* LAT2METER
        Lon = lon* LON2METER

        self.p = 10000
        self.bias = 0
        self.F = np.array([[1.0,  ts,   0,   0],
                        [  0, 1.0,   0,   0],
                        [  0,   0, 1.0,  ts],
                        [  0,   0,   0, 1.0]])

        self.K = np.array([[(1.0-beta**2)             ,0],
                        [(1.0-beta)**2/ts          ,0],
                        [0,             (1.0-beta**2)],
                        [0,          (1.0-beta)**2/ts]])


        self.H = np.array([[1.0, 0,   0, 0],
                        [  0, 0, 1.0, 0]])



        self.Xbar = np.array([[Lat],
                            [0],
                            [Lon],
                            [0]])


        self.Xhat = np.array([[0],
                           [0],
                           [0],
                           [0]])

        self.ePos = np.array([[0],
                           [0],
                           [0],
                           [0]])

        self.y = np.array([[Lat],
                        [Lon]])

        self.yawest = 0

    def main(self,lat,lon):  

        Lat = lat * LAT2METER
        Lon = lon * LON2METER   


         
        self.y[0][0] = Lat
        self.y[1][0] = Lon
   
        
        residual    = self.y - self.H @ self.Xbar
        self.Xhat   = self.Xbar + self.K @ residual
        self.ePos   = self.F @ self.Xhat
        self.Xbar   = self.ePos

        eR= np.hypot(self.ePos[0][0],self.ePos[1][0])
        Vn=self.ePos[1][0]
        Ve=self.ePos[3][0]
        V = np.hypot(Vn,Ve)
        self.yawest = 90-(np.arctan2(Vn,Ve) *180/np.pi)
        # print(yawest)
        return V*3.6
    
    def yoyo(self,meas_yaw):
        q = 0.01
        r = 0.01
        z = self.yawest - meas_yaw
        # self.bias = 0.9 * self.residual + 0.1 * ( self.yawest - meas_yaw)
        self.residual =z - self.bias
        self.p = self.p + q
        k = self.p/(self.p + r)
        self.bias = self.bias + k * (z - self.bias)
        self.p = (1-k)* self.p
        # print(f"Yaw bias:{self.bias}, error cov: {self.residual} ")
    