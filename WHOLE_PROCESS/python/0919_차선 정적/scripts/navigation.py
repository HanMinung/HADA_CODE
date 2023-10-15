



import numpy as np
import time 
import math


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