import numpy as np
import time 
import math
import ctypes as ct
import ctypes.wintypes as wt
import DBSCAN_Noh as db
from drawnow import *
from math import sin, cos,asin,sqrt,atan
from sklearn.cluster import DBSCAN


class READ_DATA(ct.Structure):# For MORAI simulation
    _fields_ = [("Roll",ct.c_double),("Pitch",ct.c_double),("Yaw",ct.c_double),("Acc_x",ct.c_double),("Acc_y",ct.c_double),("Acc_z",ct.c_double),("Latitude",ct.c_double),("Longitude",ct.c_double),("Distance",ct.c_double*761),("Velocity",ct.c_double)]# ,,("Traffic_Flag",ct.c_int*8)]

global acc_x, acc_y, yaw , lat , lon , LAT2METER ,LON2METER,Rn,Re ,x_buf,y_buf
global distance, azim 
acc_x, acc_y, yaw , lat , lon = 0,0,0,0,0
distance = []
azim = np.linspace(93.5,-96.5,761) #morai simulation
LAT2METER = 110950.59672489;
LON2METER = 90048.170449268 ;
# LON2METER = 110048.170449268 ;
Rn = 6356752;
Re = 6378137;
x_buf =[]
y_buf = []

class READ_DATA(ct.Structure):# For MORAI simulation
    _fields_ = [("Roll",ct.c_double),("Pitch",ct.c_double),("Yaw",ct.c_double),("Acc_x",ct.c_double),("Acc_y",ct.c_double),("Acc_z",ct.c_double),("Latitude",ct.c_double),("Longitude",ct.c_double),("Distance",ct.c_double*761),("Velocity",ct.c_double)]# ,,("Traffic_Flag",ct.c_int*8)]

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
        global acc_x, acc_y, yaw , lat , lon
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


        # If MORAI
        self.rfile_mapping_name_ptr = ct.c_wchar_p("Hils_smdat_ReadData")

  
        self.rbyte_len = ct.sizeof(READ_DATA)    

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
        read_smdat = READ_DATA()
        rmsg_ptr   = ct.pointer(read_smdat)
        self.memcpy(rmsg_ptr,self.rmapped_view_ptr,self.rbyte_len)
        yaw   = read_smdat.Yaw
        lat   = read_smdat.Latitude
        lon   = read_smdat.Longitude
        ## You can search it to find out the detailed structure of shared memory

    def main(self,cnt):
        global acc_x, acc_y, yaw , lat , lon ,distance

        if self.is_sharedmemory==True:
            read_smdat = READ_DATA()
            rmsg_ptr   = ct.pointer(read_smdat)
            self.memcpy(rmsg_ptr,self.rmapped_view_ptr,self.rbyte_len)
            distance = read_smdat.Distance
            for i in range(761):
                if distance[i] >= 5000:
                    distance[i] =0
                
                distance[i] = distance[i]*0.002
            acc_x = read_smdat.Acc_x
            acc_x = read_smdat.Acc_y
            yaw   = read_smdat.Yaw
            lat   = read_smdat.Latitude
            lon   = read_smdat.Longitude


    def sharedmemory_close(self):
        self.UnmapViewOfFile(self.rmapped_view_ptr)
        self.CloseHandle(self.rmapping_handle)


def NE_to_XY(int_lat, int_lon, lat, lon):
    x = (lon-int_lon) * LON2METER
    y = (lat-int_lat) * LAT2METER

    return x,y

def cone_global_pose(cX , cY, my_x, my_y, yaw):
    
    offset =88
    R = np.array([[  np.cos(-(yaw +offset) * np.pi /180)   ,   -np.sin(-(yaw +offset) * np.pi /180)  , -my_x ], 
                  [  np.sin(-(yaw +offset) * np.pi /180)   ,    np.cos(-(yaw +offset) * np.pi /180) ,  -my_y  ] ] )
    x_B = np.array([cX,cY ,1])
    x_B = x_B.T
    g =  R @ x_B
    gx = g[0]
    gy = g[1]

    return gx,gy

def anime_plot():
    plt.plot(x_buf,y_buf,'ro',markersize = 5)
    plt.grid()

def removeOutliers(dist):
    n =len(dist)
    for i in range(2,n-1):
        if abs(dist[i-1] - dist[i]) >0.5 and abs(dist[i+1] - dist[i]) >0.5 :
            dist[i] = 0
    return dist

def polar2xy(dist, az ):
    n = len(az)
    x=np.zeros(n)
    y=np.zeros(n)
    d2r = np.pi/180
    for i in range(n):
        x[i] = dist[i] * cos(az[i]*d2r)
        y[i] = dist[i] * sin(az[i]*d2r)
    return x,y

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
    cX = clusterX[s]
    cY = clusterY[s]

    return cX,cY ,cluster_num   

if __name__ == "__main__":
    sim = LIDAR_SM()
    sim.sharedmemory_open()
    int_lat = lat
    int_lon = lon
    time_curr  = 0
    time_cnt   = 0
    time_stime = 0 
    time_ts    = 0.05
    time_final = 5000
    cone_x_buf =[]
    cone_y_buf =[]
    
    while (time_stime < time_final) :
        time_start = time.time()
        sim.main(time_cnt)
        x,y = NE_to_XY(int_lat, int_lon, lat, lon)
        x_buf.append(x)
        y_buf.append(y)
        cX,cY,cN = clustering()
        for i in range(cN):
            cone_x, cone_y = cone_global_pose(cX[i], cY[i],x,y,yaw)
            cone_x_buf.append(cone_x)
            cone_y_buf.append(cone_y)
        
        plt.cla()
        plt.plot(cone_x_buf,cone_y_buf,'bo',markersize = 5)
        plt.grid()
        plt.pause(0.001)
        # cone_x_buf =[]
        # cone_y_buf =[]
        # drawnow(anime_plot)

        # while(1):
        #     time_curr = time.time()
        #     time_del = time_curr - time_start - time_stime
        #     if (time_del > time_ts):
        #         time_cnt   += 1
        #         time_stime =  time_cnt*time_ts
               
        #         break
            
        # drawnow(showplot)
        time_cnt   += 1

        time_end = time.time()
        time_del = time_end-time_start
        # print(f'loop time:{time_del:.4f}s')


    sim.sharedmemory_close()
    




