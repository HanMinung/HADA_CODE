import numpy as np
import time 
import math
import ctypes as ct
import ctypes.wintypes as wt
from drawnow import *
from math import sin, cos,asin,sqrt,atan
from sklearn.cluster import DBSCAN
from serial_node import erpSerial

#TO MORAI OPERATING CODE
class WRITE_DATA(ct.Structure):   # Struct with Lidar
    _fields_ = [ ("Steering", ct.c_double), ("Velocity", ct.c_double),("Gear",ct.c_int)]

# class READ_DATA(ct.Structure):# For Real Lidar
#     _fields_ = [("xCoordinate",ct.c_double*761),("yCoordinate",ct.c_double*761),("dist",ct.c_double*761),("angle",ct.c_double*761)]# ,,("Traffic_Flag",ct.c_int*8)]

class READ_DATA(ct.Structure):# For MORAI simulation
    _fields_ = [("Roll",ct.c_double),("Pitch",ct.c_double),("Yaw",ct.c_double),("Acc_x",ct.c_double),("Acc_y",ct.c_double),("Acc_z",ct.c_double),("Latitude",ct.c_double),("Longitude",ct.c_double),("Distance",ct.c_double*761),("Velocity",ct.c_double)]# ,,("Traffic_Flag",ct.c_int*8)]

# Global variables
global distance, azim ,preDist , L_cone_x,L_cone_y, R_cone_x, R_cone_y ,steering_ang

distance = []
# azim = np.linspace(-95,95,761) #real
azim = np.linspace(95,-95,761) #morai simulation
L_cone_x,L_cone_y, R_cone_x, R_cone_y =0,1.5,0,-1.5
steering_ang =0
L1 = np.array([ 0.0, 0.0])
R1 = np.array([ 0.0, 0.0])
L2 = np.array([ 0.0, 0.0])
R2 = np.array([ 0.0, 0.0])

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
        # If Real
        # self.rfile_mapping_name_ptr = ct.c_wchar_p("Lidar_smdat_ReadData")

        # If MORAI
        self.rfile_mapping_name_ptr = ct.c_wchar_p("Hils_smdat_ReadData")

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
        self.rmapping_handle = self.OpenFileMapping(self.FILE_MAP_READ, False, self.rfile_mapping_name_ptr)
        if not self.rmapping_handle:
            print("Could not open file mapping object: {:d}".format(self.GetLastError()))
            raise ct.WinError()

        self.rmapped_view_ptr = self.MapViewOfFile(self.rmapping_handle, self.FILE_MAP_READ, 0, 0, self.rbyte_len)
        if not self.rmapped_view_ptr:
            print("Could not map view of file: {:d}".format(self.GetLastError()))
            self.CloseHandle(self.rmapping_handle)
            raise ct.WinError()
        
        self.is_sharedmemory = True

        ## You can search it to find out the detailed structure of shared memory

    def main(self,cnt):
        global distance ,preDist,initLat,initLon,g_lat,g_lon,g_yaw

        if self.is_sharedmemory==True:
            read_smdat = READ_DATA()
            rmsg_ptr   = ct.pointer(read_smdat)
            self.memcpy(rmsg_ptr,self.rmapped_view_ptr,self.rbyte_len)
            distance = read_smdat.Distance
            for i in range(761):
                if distance[i] >= 5000:
                    distance[i] =0
                
                distance[i] = distance[i]*0.002



            # MORAI control command
            write_smdat                 = WRITE_DATA()
            write_smdat.Steering        = steering_ang
            write_smdat.Velocity        = 40
            write_smdat.Gear            = 4

            wmsg_ptr                    = ct.pointer(write_smdat)
            self.memcpy(self.wmapped_view_ptr, wmsg_ptr, self.wbyte_len)

            



    def sharedmemory_close(self):
        self.UnmapViewOfFile(self.wmapped_view_ptr)
        self.CloseHandle(self.wmapping_handle)
        self.UnmapViewOfFile(self.rmapped_view_ptr)
        self.CloseHandle(self.rmapping_handle)



def remove_outliers(dist):
    n =len(dist)
    for i in range(2,n-1):
        if abs(dist[i-1] - dist[i]) >0.5 and abs(dist[i+1] - dist[i]) >0.5 :
            dist[i] = 0
    return dist


def euc_dist(x1,y1,x2,y2): # distance of two points
    return math.sqrt((x1-x2)**2+(y1-y2)**2)  

def dist_point_line(px,py ,x1,y1,x2,y2):
    
    # line equation
    a = 1
    b =  - (x2 - x1) /(y2 - y1 + 0.0001)
    c = - (a*x1 + b*y1) 

    d = (a * px + b * py +c) / math.hypot(a,b)

    return d

def polar2xy(dist, az ):
    n = len(az)
    x=np.zeros(n)
    y=np.zeros(n)
    d2r = np.pi/180
    for i in range(n):
        x[i] = dist[i] * cos(az[i]*d2r)
        y[i] = dist[i] * sin(az[i]*d2r)
    return x,y


def removeOutliers(dist):
    n =len(dist)
    for i in range(2,n-1):
        if abs(dist[i-1] - dist[i]) >0.5 and abs(dist[i+1] - dist[i]) >0.5 :
            dist[i] = 0
    return dist


def distinguishLeftRight(rubberX,rubberY): #
    global L_cone_x, L_cone_y,R_cone_x,R_cone_y
    
    idx_left = (rubberY>0)
    idx_right = (rubberY<0)
    LstartX= rubberX[idx_left] 
    LstartY = rubberY[idx_left] 
    RstartX = rubberX[idx_right] 
    RstartY = rubberY[idx_right] 

    if len(LstartX) > 0 and len(RstartX) > 0 :
        if LstartX[0] == 0 :
            L1[0] = LstartX[1]
            L1[1] = LstartY[1]
        else:
            L1[0] = LstartX[0]
            L1[1] = LstartY[0]
        if RstartX[0] == 0 :
            R1[0] = RstartX[1]
            R1[1] = RstartY[1]
        else:
            R1[0] = RstartX[0]
            R1[1] = RstartY[0]
        K1 = 1    # distance coefficients
        K2 = 0.7  # heading coefficient
        min_left_cost = np.inf
        min_right_cost = np.inf
        for i in range(len(rubberX)):
            left_dist = euc_dist(L1[0] , L1[1] , rubberX[i] , rubberY [i])
            left_heading = math.fabs(math.atan2 ( rubberY[i] - L1[1] , rubberX[i] - L1[0]))
            # left_edgedist = dist_point_line(rubberX[i] , rubberY [i] , )
            left_cost = K1 * left_dist + K2 * left_heading
            if left_cost < min_left_cost and left_dist > 0.1 and rubberX[i] != 0 :
                min_left_cost = left_cost
                L2[0] = rubberX[i]
                L2[1] = rubberY[i]
            
            right_dist = euc_dist(R1[0] , R1[1] , rubberX[i] , rubberY [i])
            right_heading = math.fabs(math.atan2 ( rubberY[i] - R1[1] , rubberX[i] - R1[0]))
            # right_edgedist = dist_point_line(rubberX[i] , rubberY [i] , )
            right_cost = K1 * right_dist + K2 * right_heading
            if right_cost < min_right_cost and right_dist > 0.1 and rubberX[i] != 0 :
                min_right_cost = right_cost
                R2[0] = rubberX[i]
                R2[1] = rubberY[i]
    


    return L1, L2 , R1, R2


def purePursuit(LoS,lookahead):
    delta =  atan(2*sin(LoS)/lookahead)
    if delta > 28 :
        delta = 28
    elif delta < -28:
        delta = -28
    return delta

def clustering():#X array Y array of Lidar
    
    x=[]
    y=[]
    clusterX = []    # cluster X좌표
    clusterY = []    # cluster Y좌표
    clusterDist = [] # cluster의 차로부터의 거리
    nu = removeOutliers(distance)
    x , y = polar2xy(nu,azim)
    
        
    X = np.stack((x,y), axis = 1)
    cluster = DBSCAN(eps=0.4,min_samples=7).fit(X)    
    labels = cluster.labels_
    cluster_num = max(labels)
    ROI_cluster_num =0
    for i in range(cluster_num+1):   #클러스터 갯수만큼 반복
        points_x = x[labels == i]
        points_y = y[labels == i]
        
        center_x = np.mean(points_x)
        center_y = np.mean(points_y)
        

        clusterDist.append(math.hypot(center_x,center_y))
        clusterX.append(center_x)
        clusterY.append(center_y)
        
    clusterDist = np.array(clusterDist)
    clusterX = np.array(clusterX)
    clusterY = np.array(clusterY)
    s = clusterDist.argsort()
    sortedX = clusterX[s]
    sortedY = clusterY[s]

    return sortedX , sortedY ,cluster_num   

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

def stanley_controller(L1 , L2 , R1, R2):
    p1 = (L1 + R1) * 0.5
    p2 = (L2 + R2) * 0.5
    k =1
    velocity = 6
    track_heading = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    heading_error = normalize_angle(track_heading)
    crosstrack_error = -dist_point_line(0,0,p1[0] , p1[1] ,p2[0],p2[1])
    c_err = np.arctan2(k * crosstrack_error, velocity)
    # print(crosstrack_error)
    # delta = heading_error + c_err
    wx = (p2[0] + p1[0]) *0.5
    wy = (p2[1] + p1[1]) *0.5
    delta = purePursuit(np.arctan2(wy, wx),5)
    # delta = heading_error
    if delta > 28.0 *np.pi/180 :
        delta = 28.0 * np.pi/180
    elif delta < -28.0 *np.pi/180 :
        delta = -28.0 * np.pi/180

    return delta


if __name__ == "__main__":
    sim = LIDAR_SM()
    sim.sharedmemory_open()
    device = 'com4'
    # command  =  erpSerial(device)
    velCMD = 4
   

    time_curr  = 0
    time_cnt   = 0
    time_stime = 0 
    time_ts    = 0.02
    time_final = 5000

    preDist = [5000 for _ in range(761)]

    
    while (time_stime < time_final) :
        time_start = time.time()
        sim.main(time_cnt)
        cX , cY, cN = clustering()
        L1, L2, R1,R2 = distinguishLeftRight(cX, cY)
        plt.cla()
        plt.plot(cX,cY , 'ro',markersize = 4)
        plt.plot(L1[0] , L1[1] ,'yo',markersize = 10)
        plt.plot(L2[0] , L2[1] ,'yo',markersize = 10)
        plt.plot(R1[0] , R1[1] ,'bo',markersize = 10)
        plt.plot(R2[0] , R2[1] ,'bo',markersize = 10)
        plt.xlim([-1,5])
        plt.ylim([-5,5 ])
        plt.grid()
        plt.pause(0.0001)

        # while(1):
        #     time_curr = time.time()
        #     time_del = time_curr - time_start - time_stime
        #     if (time_del > time_ts):
        #         time_cnt   += 1
        #         time_stime =  time_cnt*time_ts
        #        
        #         break
            
 
        # command.send_ctrl_cmd(int(velCMD), int(steering_ang))
        time_cnt   += 1

        time_end = time.time()
        time_del = time_end-time_start
        # print(f'loop time:{time_del:.4f}s')
        print(f'steer cmd:{steering_ang:.4f}deg')


    sim.sharedmemory_close()