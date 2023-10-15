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
from drawnow import *
from math import sin, cos,asin,sqrt,atan

#IMU REAL TIME PLOT

#TO MORAI OPERATING CODE
class WRITE_DATA(ct.Structure):   # Struct with Lidar
    _fields_ = [ ("Steering", ct.c_double), ("Velocity", ct.c_double),("Gear",ct.c_int)]
# 받기용 구조체  
class READ_DATA(ct.Structure):
    _fields_ = [("Roll",ct.c_double),("Pitch",ct.c_double),("Yaw",ct.c_double),("Acc_x",ct.c_double),("Acc_y",ct.c_double),("Acc_z",ct.c_double),("Latitude",ct.c_double),("Longitude",ct.c_double),("Distance",ct.c_double*761),("Velocity",ct.c_double)]# ,,("Traffic_Flag",ct.c_int*8)]

# class READ_DATA(ct.Structure):
#     _fields_ = [("xCoordinate",ct.c_double*761),("yCoordinate",ct.c_double*761),("dist",ct.c_double*761),("angle",ct.c_double*761)]# ,,("Traffic_Flag",ct.c_int*8)]

global distance, azim ,preDist,L_cone_x, L_cone_y,R_cone_x,R_cone_y
L_cone_x,L_cone_y, R_cone_x, R_cone_y =0,0,0,0
distance = []
azim = np.linspace(-95,95,761)
     
class LIDAR_SM :
    
    # ============================================================================
    # Communication Variable Definition
    # ============================================================================
    def __init__(self):
        self.is_sharedmemory = False



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
        self.wfile_mapping_name_ptr = ct.c_wchar_p("ERP42_smdat_ReadData")
        # self.rfile_mapping_name_ptr = ct.c_wchar_p("Lidar_smdat_ReadData")
        self.rfile_mapping_name_ptr = ct.c_wchar_p("Hils_smdat_ReadData")
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
        self.rmapping_handle = self.OpenFileMapping(self.FILE_MAP_ALL_ACCESS, False, self.rfile_mapping_name_ptr)
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

    def main(self):
        global distance ,preDist

        if self.is_sharedmemory==True:
            read_smdat = READ_DATA()
            rmsg_ptr   = ct.pointer(read_smdat)
            self.memcpy(rmsg_ptr,self.rmapped_view_ptr,self.rbyte_len)
            distance = read_smdat.Distance
            # distance = read_smdat.dist

            

            write_smdat                 = WRITE_DATA()
            write_smdat.Steering        = findSteerAng()
            write_smdat.Velocity        = 18
            write_smdat.Gear            = 4

            wmsg_ptr                    = ct.pointer(write_smdat)
            self.memcpy(self.wmapped_view_ptr, wmsg_ptr, self.wbyte_len)
            # print(f'roll : %.2f, pitch : %.2f, yaw : %.2f'% (roll,pitch,yaw))
            # print(distance)

    def sharedmemory_close(self):
        self.UnmapViewOfFile(self.wmapped_view_ptr)
        self.CloseHandle(self.wmapping_handle)
        self.UnmapViewOfFile(self.rmapped_view_ptr)
        self.CloseHandle(self.rmapping_handle)


def findCircumcenter(x1,y1,x2,y2,x3,y3):
    X = 0.5 * ((y2-y1)*(y2-y3)*(y3-y1)+(x2*x2 - x1*x1)*(y3-y1) -(x3*x3-x1*x1)*(y2-y1) ) /((x2-x1)*(y3-y1) -(x3-x1)*(y2-y1)+0.00000001)
    Y = 0.5 * ((x2-x1)*(x2-x3)*(x3-x1)+(y2*y2 - y1*y1)*(x3-x1) -(y3*y3-y1*y1)*(x2-x1) ) /((y2-y1)*(x3-x1) -(y3-y1)*(x2-x1)+0.00000001)
    return X ,Y

def removeOutliers(dist):
    n =len(dist)
    for i in range(2,n-1):
        if abs(dist[i-1] - dist[i]) >300  and abs(dist[i+1] - dist[i]) >300 :
            dist[i] = 5000
    return dist

def findLocalMin(dist,az):
    n =len(dist)
    rubberX =[]
    rubberY =[]
    flag = 0
    d2r = np.pi/180
    rubStart =0
    rubEnd =0

    for i in range(n-1):
        if dist[i] - dist[i+1] >200 and flag == 0:
            rubStart =i
            flag = 1
        elif dist[i] - dist[i+1] < -200 and flag ==1:
            rubEnd = i
            flag =0
            rubindex = round((rubStart+rubEnd)*0.5)
            x = dist[rubindex] * cos(az[rubindex]*d2r)
            y = dist[rubindex] * sin(az[rubindex]*d2r)
            rubberX.append(x)
            rubberY.append(y)




    return rubberX,rubberY


def polar2xy(dist, az ):
    n = len(az)
    x=np.zeros(n)
    y=np.zeros(n)
    d2r = np.pi/180
    for i in range(n):
        x[i] = dist[i] * cos(az[i]*d2r)
        y[i] = dist[i] * sin(az[i]*d2r)
    return x,y

def distinguishLeftRight(rubberX,rubberY):
    global L_cone_x, L_cone_y,R_cone_x,R_cone_y
    idx_left = (rubberY>0)
    idx_right = (rubberY<0)
    LstartX= rubberX[idx_left] 
    LstartY = rubberY[idx_left] 
    RstartX = rubberX[idx_right] 
    RstartY = rubberY[idx_right] 
    # LeftConeX = [] 
    # LeftConeY = [] 
    # RightConeX = [] 
    # RightConeY = []
    if len(LstartX) > 0 and LstartX[0] <2:
        L_cone_x = LstartX[0] 
        L_cone_y = LstartY[0] 
    else :
        L_cone_x = L_cone_x
        L_cone_y = L_cone_y
    if len(RstartX) > 0 and RstartX[0] <2:
        R_cone_x = RstartX[0] 
        R_cone_y = RstartY[0] 
    else :
        R_cone_x = R_cone_x
        R_cone_y = R_cone_y

    
def showplot():
    x=[]
    y=[]
    nu = removeOutliers(distance)
    x , y = polar2xy(nu,azim)

    rubx ,ruby = findLocalMin(nu,azim)
    
    plt.plot(x,y,'b-')
    # plt.plot(rubx,ruby,'bo')
    npX = np.array(rubx)
    npY = np.array(ruby)
    eucldist=np.zeros_like(npX)
    
    eucldist = npX*npX+npY*npY

    s  = eucldist.argsort()
    sortedX = npX[s]
    sortedY = npY[s]
    leftx,lefty,rightx,righty = distinguishLeftRight(sortedX,sortedY)
    plt.plot(leftx,lefty,'co')
    plt.plot(rightx,righty,'mo')

    if sortedX.size >= 3 :

        targetX ,targetY= findCircumcenter(sortedX[0],sortedY[0],sortedX[1],sortedY[1],sortedX[2],sortedY[2]) 
        plt.plot(targetX,targetY,'ko')

    # plt.legend()
    plt.xlim([0,5000 ])
    plt.ylim([-5000,5000 ])

    plt.grid()


def purePursuit(LoS,lookahead):
    delta = atan(2*sin(LoS)/lookahead)
    return delta

def findSteerAng():
    x=[]
    y=[]
    
    nu = removeOutliers(distance)
    x , y = polar2xy(nu,azim)
    rubx ,ruby = findLocalMin(nu,azim)
    npX = np.array(rubx)
    npY = np.array(ruby)
    eucldist=np.zeros_like(npX)
    
    eucldist = npX*npX+npY*npY

    s  = eucldist.argsort()
    sortedX = npX[s]
    sortedY = npY[s]
    leftx,lefty,rightx,righty = distinguishLeftRight(sortedX,sortedY)
    maxAngle = 28

    if sortedX.size >= 3 :

        targetX ,targetY= findCircumcenter(sortedX[0],sortedY[0],sortedX[1],sortedY[1],sortedX[2],sortedY[2]) 
        LineOfSight = atan(targetY/targetX)
        LookAhead = sqrt(targetX*targetX+targetY*targetY)
        deltaF = purePursuit(LineOfSight,LookAhead) * 180 /math.pi 
        # deltaF =   math.atan(targetY/(targetX+ 0.000000001))
        if deltaF> 28*math.pi/180 :
            deltaF =28.0*math.pi/180
        elif deltaF<-28*math.pi/180:
            deltaF =-28.0*math.pi/180
    else:
        deltaF =0
    # print(deltaF*180/math.pi)
    return deltaF
 




if __name__ == "__main__":
    sim = LIDAR_SM()
    sim.sharedmemory_open()

    time_curr  = 0
    time_cnt   = 0
    time_stime = 0 
    time_ts    = 0.02
    time_final = 5000

    preDist = [5000 for _ in range(761)]

    time_start = time.time()
    while (time_stime < time_final) :
    
        sim.main()
        
        while(1):
            time_curr = time.time()
            time_del = time_curr - time_start - time_stime
            if (time_del > time_ts):
                time_cnt   += 1
                time_stime =  time_cnt*time_ts
                
                break
        drawnow(showplot)

    sim.sharedmemory_close()



