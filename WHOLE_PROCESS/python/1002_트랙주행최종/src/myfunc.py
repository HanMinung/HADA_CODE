import numpy as np
import time 
import math
import ctypes as ct
import ctypes.wintypes as wt
import cv2 as cv
from drawnow import *
from math import sin, cos,asin,sqrt,atan
from sklearn.cluster import DBSCAN
from scipy.spatial import Delaunay
from src.serial_node import erpSerial
from scipy.interpolate import interp1d

#TO MORAI OPERATING CODE
mode = "real"
# global distance, azim ,steer_cmd,L_cone_x, L_cone_y,R_cone_x,R_cone_y, test_cnt
class StartCone:
    L_cone_x,L_cone_y, R_cone_x, R_cone_y = 0,1.0,0,-1.0
    L_cnt ,R_cnt =0,0

distance = [] # 라이다 데이터 
steer_cmd = 0
test_cnt =0

class WRITE_DATA(ct.Structure):   # Struct with Lidar
    _fields_ = [ ("Steering", ct.c_double), ("Velocity", ct.c_double),("Gear",ct.c_int)]

if mode == "morai":
    class READ_DATA(ct.Structure):# For MORAI simulation
        _fields_ = [("Roll",ct.c_double),("Pitch",ct.c_double),("Yaw",ct.c_double),("Acc_x",ct.c_double),("Acc_y",ct.c_double),("Acc_z",ct.c_double),("Latitude",ct.c_double),("Longitude",ct.c_double),("Distance",ct.c_double*761),("Velocity",ct.c_double)]# ,,("Traffic_Flag",ct.c_int*8)]
    azim = np.linspace(95,-95,761) #morai simulation

elif mode =="real":
    class READ_DATA(ct.Structure):# For Real Lidar
        _fields_ = [("xCoordinate",ct.c_double*761),("yCoordinate",ct.c_double*761),("dist",ct.c_double*761),("angle",ct.c_double*761)]# ,,("Traffic_Flag",ct.c_int*8)]
    azim = np.linspace(-95,95,761) #real
    



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
        if mode == "real":
            self.rfile_mapping_name_ptr = ct.c_wchar_p("Lidar_smdat_ReadData")
        elif mode == "morai":
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

    def main(self):
        global distance ,preDist,initLat,initLon,g_lat,g_lon,g_yaw,steer_cmd

        if self.is_sharedmemory==True:
            read_smdat = READ_DATA()
            rmsg_ptr   = ct.pointer(read_smdat)
            self.memcpy(rmsg_ptr,self.rmapped_view_ptr,self.rbyte_len)
            if mode == "morai":
                distance = read_smdat.Distance
            elif mode =="real":
                distance = read_smdat.dist
            for i in range(761):
                if distance[i] >=5000:
                    distance[i] =0
                
                distance[i] = distance[i]*0.002


    def send_ctrl_cmd(self,steer):
    
        write_smdat                 = WRITE_DATA()
        write_smdat.Steering        = steer
        write_smdat.Velocity        = 5
        write_smdat.Gear            = 4

        wmsg_ptr                    = ct.pointer(write_smdat)
        self.memcpy(self.wmapped_view_ptr, wmsg_ptr, self.wbyte_len)
    
            



    def sharedmemory_close(self):
        self.UnmapViewOfFile(self.wmapped_view_ptr)
        self.CloseHandle(self.wmapping_handle)
        self.UnmapViewOfFile(self.rmapped_view_ptr)
        self.CloseHandle(self.rmapping_handle)



def removeOutliers(dist):    # 라이다 이상치( 튀는 값) 제거
    n =len(dist)
    for i in range(2,n-1):
        if abs(dist[i-1] - dist[i]) >0.5 and abs(dist[i+1] - dist[i]) >0.5 :#or dist[i] > 7:
            dist[i] = 0
    return dist

def variance_of_cluster(cX,cY): # 클러스터의 분산 계산
    vx = np.var(cX)
    vy = np.var(cY)
    v = np.hypot(vx,vy)

    return v

def polar2xy(dist, az ): # 극좌표계 데이터를 직교좌표계 데이터로 변환
    n = len(az)
    x=np.zeros(n)
    y=np.zeros(n)
    d2r = np.pi/180
    for i in range(n):
        x[i] = dist[i] * cos(az[i]*d2r)
        y[i] = dist[i] * sin(az[i]*d2r)
    return x,y

def dist(x1,y1,x2,y2): # distance of two points
    return math.hypot(x1-x2,y1-y2)

def dist2(p1,p2):
    return math.hypot(p1[0]-p2[0],p1[1]-p2[1])

def vector_angle(v1 , v2):
    dot = np.dot(v1,v2)
    cos_theta = dot / (np.linalg.norm(v1) * np.linalg.norm(v2)+0.000001)
    theta = np.arccos(cos_theta)
    return theta

def distinguishLeftRight(rubberX,rubberY):
    # global L_cone_x, L_cone_y,R_cone_x,R_cone_y
    # L_cone_x +=1
    idx_left = (rubberY>0)
    idx_right = (rubberY<0)
    LstartX = rubberX[idx_left] 
    LstartY = rubberY[idx_left] 
    RstartX = rubberX[idx_right] 
    RstartY = rubberY[idx_right] 
    # LeftConeX  = [] 
    # LeftConeY  = [] 
    # RightConeX = [] 
    # RightConeY = []
    LeftTurn = StartCone.L_cnt - StartCone.R_cnt > 77
    RightTurn = StartCone.L_cnt - StartCone.R_cnt < -77
    
    if len(LstartX) > 0 and  dist(LstartX[0] , LstartY[0],StartCone.L_cone_x,StartCone.L_cone_y ) < 1.5  and LstartX[0] < 1 :
        StartCone.L_cone_x = LstartX[0] 
        StartCone.L_cone_y = LstartY[0] 
        StartCone.L_cnt = 0
        
    # elif  ( math.atan2((RstartY[1] - RstartY[0]) ,(RstartX[1]- RstartX[0])) - math.atan2((StartCone.R_cone_y - RstartY[0]) ,(StartCone.R_cone_x - RstartX[0])) ) > 20 * np.pi / 180 : # 좌회전 코너링 상황일 때 
        '''
        출발 :( math.atan2((RstartY[1] - RstartY[0]) ,(RstartX[1]- RstartX[0])) - math.atan2((R_cone_y - RstartY[0]) ,(R_cone_x - RstartX[0])) ) > 20 * np.pi / 180
        
        '''
    else: #len(RstartX) > 1  and math.atan2((StartCone.R_cone_y - RstartY[0]) ,(StartCone.R_cone_x - RstartX[0])) > 50 * np.pi / 180 :
        StartCone.L_cnt += 1

    if len(RstartX) > 0 and dist(RstartX[0] , RstartY[0],StartCone.R_cone_x,StartCone.R_cone_y ) < 1.5 and RstartX[0] < 1 :
        StartCone.R_cone_x = RstartX[0] 
        StartCone.R_cone_y = RstartY[0] 
        StartCone.R_cnt = 0

    # elif ( math.atan2((LstartY[1] - LstartY[0]) ,(LstartX[1]- LstartX[0])) - math.atan2((StartCone.L_cone_y - LstartY[0]) ,(StartCone.L_cone_x - LstartX[0])) ) < -20 * np.pi / 180 : # 우회전 코너링 상황일 때  :

    else:# len(LstartX) > 1  and math.atan2((StartCone.L_cone_y - LstartY[0]) ,(StartCone.L_cone_x - LstartX[0])) < -50 * np.pi / 180 :
        StartCone.R_cnt += 1


def clustering():#X array Y array of Lidar
    
    x=[]
    y=[]
    clusterX = []    # cluster X좌표
    clusterY = []    # cluster Y좌표
    clusterDist = [] # cluster의 차로부터의 거리
    nu = removeOutliers(distance)
    x , y = polar2xy(nu,azim)
    
        
    X = np.stack((x,y), axis = 1)
    cluster = DBSCAN(eps=0.6,min_samples=5).fit(X)    
    labels = cluster.labels_
    cluster_num = max(labels)
    ROI_cluster_num =0
    for i in range(cluster_num+1):   #클러스터 갯수만큼 반복
        points_x = x[labels == i]
        points_y = y[labels == i]
        X = np.stack((points_x,points_y) , axis= 0)
        cov = np.cov(X)
        var = variance_of_cluster(points_x,points_y)
        
        # print(cov)
        center_x = np.mean(points_x)
        center_y = np.mean(points_y)
        if math.hypot(center_x ,center_y) < 0.001 or var > 0.005 :
            continue
        else:
            clusterDist.append(math.hypot(center_x,center_y))
            clusterX.append(center_x)
            clusterY.append(center_y)
            ROI_cluster_num +=1
        
    clusterDist = np.array(clusterDist)
    clusterX = np.array(clusterX)
    clusterY = np.array(clusterY)
    s = clusterDist.argsort()
    cX = clusterX[s]
    cY = clusterY[s]

    return cX,cY ,ROI_cluster_num   


def generate_path(candid_x, candid_y):

    path            = []   # 경로 점들 저장
    path_indices    = []                  # 경로 점들의 원본 배열 에서의 인덱스 값
    k_dist          = 1
    k_heading       = 1
    next_point_idx =0
    
    if len(candid_x) >0 :
        candid          = np.stack((candid_x,candid_y), axis=1)
        candid_r        = [math.sqrt(candid_x[i]**2 + candid_y[i]**2) for i in range(len(candid_x))]
        point_start_idx = candid_r.index(min(candid_r)) # 가장 가까운 중점 => 경로의 시작점
        # startpoint      = np.stack((candid_x[point_start_idx],candid_y[point_start_idx]), axis=0)
        startpoint  = [(StartCone.L_cone_x +StartCone.R_cone_x)*0.5 ,(StartCone.L_cone_y + StartCone.R_cone_y)*0.5 ] 
        path.append(startpoint) # 시작점 저장
        # path_indices.append(point_start_idx)
        '''
        cost(start , simp[i]) 했을 때 최소가 되는 i (start idx 제외)=> 저장 and pop
        cost(next, simp[i]) 최소  = > 저장 and pop
        한 5 6번 반복
        '''
        candid_indices = list(range(len(candid_x))) # 후보 점들의 인덱스들

        # 1st iteration (시작점 -> 2번째 점)
        e_x = [1,0]
        min_cost = np.inf
        for i in candid_indices:
            if i not in path_indices and vector_angle(e_x,candid[i] - startpoint ) < 75 * np.pi / 180:
                costfun = k_dist * dist2(startpoint ,candid[i])   
                if costfun < min_cost  :
                    next_point_idx = i
                    min_cost = costfun
        path_indices.append(next_point_idx) # 2번째 점 인덱스 저장

        # 2nd iteration
        for n in range(2):
            min_cost = np.inf
            v1 = candid[next_point_idx] - startpoint
            ang =0
            for i in candid_indices:
                if i not in path_indices and (vector_angle(v1,candid[i]-candid[next_point_idx]))< 70 * np.pi / 180: # 이미 
                    costfun = k_dist * dist2(candid[next_point_idx] ,candid[i])
                    if costfun < min_cost: #and math.fabs(vector_angle(v1,candid[i]-candid[next_point_idx])) <70 * np.pi /180:
                        ang = i
                        min_cost = costfun
            next_point_idx =ang
            path_indices.append(next_point_idx) # 2번째 점 인덱스 저장

        for i in path_indices:
            path.append(candid[i])

        path = np.array(path)
    return path


def purePursuit(LoS,lookahead):
    min_lookahead = 2.0 # [m]
    WB = 1.4
    if lookahead < min_lookahead:
        lookahead = min_lookahead
    delta =  - 2 * math.sin(LoS) / lookahead *180/np.pi
    if delta > 28 :
        delta = 28
    elif delta < -28:
        delta = -28
    return delta


class VelCon:
    def __init__(self):
        self.Ts = 0.05 #sample period
        self.meas_vel = 0
        self.Kp = 2
        self.Ki = 0.3
        self.err = 0
        self.i_err = 0
        self.vel_cmd = 0
        
    def velocity_control(self, ref_vel , meas_vel ) :
         # 전 측정값과 5km / h 이상 차이나면 오류로 판단
        # if math.fabs(meas_vel - self.meas_vel) > 5 or meas_vel > 33:
        #     meas_vel = 0
        
        # else:    
        #     self.meas_vel = meas_vel 
        
        err = ref_vel - meas_vel
        self.i_err += err * self.Ts 
        self.i_err = min(max(-20, self.i_err),20)
        vel_cmd = self.Kp * err + self.Ki * self.i_err
        vel_cmd = min(max(vel_cmd , 0) , 20)
        
        return vel_cmd