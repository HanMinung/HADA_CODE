import numpy as np
import time 
import math
import ctypes as ct
import ctypes.wintypes as wt
import DBSCAN_Noh as db
import cv2 as cv
from drawnow import *
from math import sin, cos,asin,sqrt,atan
from sklearn.cluster import DBSCAN
from globVariable import *
from lib.cam_util import UDP_CAM_Parser
import os,json

path = os.path.dirname( os.path.abspath( __file__ ) )

with open(os.path.join(path,("params.json")),'r') as fp :
    params = json.load(fp)

params=params["params"]
user_ip = params["user_ip"]
host_ip = params["host_ip"]
cam_port = params["cam_dst_port"]


params_cam = {
    "localIP": user_ip,
    "localPort": cam_port,
    "hostIP" : host_ip,
    "Block_SIZE": int(65000)
}

#IMU REAL TIME PLOT


#TO MORAI OPERATING CODE
class WRITE_DATA(ct.Structure):   # Struct with Lidar
    _fields_ = [ ("Steering", ct.c_double), ("Velocity", ct.c_double),("Gear",ct.c_int)]

# class READ_DATA(ct.Structure):# For Real Lidar
#     _fields_ = [("xCoordinate",ct.c_double*761),("yCoordinate",ct.c_double*761),("dist",ct.c_double*761),("angle",ct.c_double*761)]# ,,("Traffic_Flag",ct.c_int*8)]

class READ_DATA(ct.Structure):# For MORAI simulation
    _fields_ = [("Roll",ct.c_double),("Pitch",ct.c_double),("Yaw",ct.c_double),("Acc_x",ct.c_double),("Acc_y",ct.c_double),("Acc_z",ct.c_double),("Latitude",ct.c_double),("Longitude",ct.c_double),("Distance",ct.c_double*761),("Velocity",ct.c_double)]# ,,("Traffic_Flag",ct.c_int*8)]

global distance, azim ,preDist ,Rn, Re ,initLat,initLon,g_lat,g_lon,g_yaw , L_cone_x,L_cone_y, R_cone_x, R_cone_y ,steering_ang , R_bound_yaw, L_bound_yaw, track_width
distance = []
# azim = np.linspace(-95,95,761) #real
azim = np.linspace(95,-95,761) #morai simulation
Rn = 6356752  # 극반지름 [m]
Re = 6378137  # 적도반지름 [m]
R_mean = 6367445
lat2meter =  math.pi / 180 * R_mean    
lon2meter = math.cos(37.23882333 * math.pi / 180) * R_mean 
meter2lat = 1/lat2meter
meter2lon = 1/lon2meter
initLat =0
initLon =0
g_lat = 0
g_lon = 0
g_yaw = 0
L_cone_x,L_cone_y, R_cone_x, R_cone_y =0,1.5,0,-1.5
R_bound_yaw = 0
L_bound_yaw = 0
track_width = 2
steering_ang =0
L1 = np.array([ 0.0, 0.0])
R1 = np.array([ 0.0, 0.0])
L2 = np.array([ 0.0, 0.0])
R2 = np.array([ 0.0, 0.0])

global Kp , pError , Ki , iError
iError = 0
Kp = 3
Ki = 0.1
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
            if cnt == 0:
                initLat = read_smdat.Latitude
                initLon = read_smdat.Longitude
            g_lon = read_smdat.Longitude
            g_lat = read_smdat.Latitude
            g_yaw = read_smdat.Yaw



            write_smdat                 = WRITE_DATA()
            write_smdat.Steering        = steering_ang
            write_smdat.Velocity        = 30
            write_smdat.Gear            = 4

            wmsg_ptr                    = ct.pointer(write_smdat)
            self.memcpy(self.wmapped_view_ptr, wmsg_ptr, self.wbyte_len)

            



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
        if abs(dist[i-1] - dist[i]) >0.5 and abs(dist[i+1] - dist[i]) >0.5 :
            dist[i] = 0
    return dist

def findLocalMin(dist,az):
    n =len(dist)
    rubberX =[]
    rubberY =[]
    flag = 0
    d2r = np.pi/180
    rubStart =0
    rubEnd =0
    # for i in range(2,n-1):
    #     if dist[i-1] > dist[i] and dist[i+1] > dist[i]:
    #         x = dist[i] * cos(az[i]*d2r)
    #         y = dist[i] * sin(az[i]*d2r)
    #         rubberX.append(x)
    #         rubberY.append(y)
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

def _dist(x1,y1,x2,y2): # distance of two points
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

def distinguishLeftRight(rubberX,rubberY): # 좀더 발전시킬것 
    global L_cone_x, L_cone_y,R_cone_x,R_cone_y,track_width
    
    idx_left = (rubberY>0)
    idx_right = (rubberY<0)
    LstartX = rubberX[idx_left] 
    LstartY = rubberY[idx_left] 
    RstartX = rubberX[idx_right] 
    RstartY = rubberY[idx_right] 


    if len(LstartX) > 0 and len(RstartX) > 0 :
        if LstartX[0] == 0 :
            L1[0] = LstartX[1]
            L1[1] = LstartY[1]
        elif LstartX[0] < 2:
            L1[0] = LstartX[0]
            L1[1] = LstartY[0]

        if RstartX[0] == 0 :
            R1[0] = RstartX[1]
            R1[1] = RstartY[1]
        elif RstartX[0] <2:
            R1[0] = RstartX[0]
            R1[1] = RstartY[0]
        K1 = 1    # distance coefficients
        K2 = 1  # heading coefficient
        min_left_cost = np.inf
        min_right_cost = np.inf
        l_idx =0
        for i in range(len(rubberX)):
            left_dist = _dist(L1[0] , L1[1] , rubberX[i] , rubberY [i]) # start cone 부터 new cone 까지 거리
            left_heading = math.fabs(math.atan2 ( rubberY[i] - L1[1] , rubberX[i] - L1[0]))
            left_edgedist = dist_point_line(rubberX[i] , rubberY [i] ,R1[0] ,R1[1] , R2[0] , R2[1])
            left_cost = K1 * left_dist + K2 * left_heading-rubberY [i]#+math.fabs(left_edgedist)
            if( left_cost < min_left_cost and 
                left_dist > 0.1 and  
                rubberX[i] != 0 and 
                rubberX[i] > L1[0]  ) :
                min_left_cost = left_cost
                L2[0] = rubberX[i]
                L2[1] = rubberY[i]
                l_idx = i

        for i in range(len(rubberX)):    
            right_dist = _dist(R1[0] , R1[1] , rubberX[i] , rubberY [i])
            right_heading = math.fabs(math.atan2 ( rubberY[i] - R1[1] , rubberX[i] - R1[0]))
            right_edgedist = dist_point_line(rubberX[i] , rubberY [i] ,L1[0] ,L1[1] , L2[0] , L2[1] )
            right_cost = K1 * right_dist + K2 * right_heading + rubberY [i] #+math.fabs(right_edgedist)
            if right_cost < min_right_cost and right_dist > 0.1 and rubberX[i] != 0 and  i != l_idx and rubberX[i] > R1[0]:
                min_right_cost = right_cost
                R2[0] = rubberX[i]
                R2[1] = rubberY[i]


        

    


    return L1, L2 , R1, R2


    # LeftConeX = [] 
    # LeftConeY = [] 
    # RightConeX = [] 
    # RightConeY = []
    
    # if( len(LstartX) > 1 and len(RstartX) > 1 
    #     and  LstartX[0] < 3 
    #     and math.atan2(  LstartY[1] - LstartY[0] , LstartX[1] - LstartX[0]) < 60
    #     and _dist(LstartX[0],LstartY[0],RstartX[0],RstartY[0]) > 1.5 ):
    #     # track_width = _dist( L_cone_x,L_cone_y,R_cone_x,R_cone_y ) 
    #     L_cone_x = LstartX[0] 
    #     L_cone_y = LstartY[0] 
    # else :
    #     L_cone_x = L_cone_x
    #     L_cone_y = L_cone_y

    # if (len(RstartX) > 1 and len(LstartX) > 1 
    #     and  RstartX[0] < 3 
    #     and math.atan2(  RstartY[1] - RstartY[0] , RstartX[1] - RstartX[0]) < 60
    #     and _dist(LstartX[0],LstartY[0],RstartX[0],RstartY[0]) > 1.5):

    #     R_cone_x = RstartX[0] 
    #     R_cone_y = RstartY[0] 

    # else :
    #     R_cone_x = R_cone_x
    #     R_cone_y = R_cone_y


 






    
    # return L_cone_x, L_cone_y,R_cone_x,R_cone_y



def getGlobalPosition(px,py,lat,lon,yaw): # 물체의 NE좌표계 절대위치 반환
    alpha = math.atan2(py,px)
    pLat = lat + math.hypot(px,py) * math.cos((yaw-alpha) * math.pi / 180) * meter2lat
    pLon = lon + math.hypot(px,py) * math.sin((yaw-alpha) * math.pi / 180) * meter2lon
    return pLat,pLon

def purePursuit(LoS,lookahead):
    delta =  atan(2*sin(LoS)/lookahead)
    if delta > 28 :
        delta = 28
    elif delta < -28:
        delta = -28
    return delta

def PNG(LOS , lookahead):
    pass

def impact_angle_gui(eta ,rng):
    C1 =1.4
    C2= 0.14
    K1 = C1 * exp(-C2*(1-rng))
    gamDotc = K1 * C2 * sin(eta) + (2.0+K1) * (-4*sin(eta)/rng);
    delta = gamDotc * 1.4 / 4;
    return delta
def nohs_dynamic_window(clusterX, clusterY, clusterN):
    steering_angle = 0 # 전역변수
    wheelbase = 1.04
    car_width = 2.5
    look_ahead = 1.5
    hit = 0
    inbound_hit = []
    

    for delta in range(-28, 29 ): # 1도 해상도로 모든 조향각에 대해서 충돌 안하는 조향각 찾음
        if math.fabs(delta) > 0:
            R = wheelbase / math.fabs(math.sin(delta * math.pi / 180))
        
        theta = look_ahead / R
        
        for i in range(clusterN):
            x = clusterX[i]
            y = clusterY[i]
            d = math.sqrt(math.hypot(x,y))
            # 차량의 충돌범위 이내에 클러스터(라바콘) 가 있을 경우  hit 추가
            # 가장 적은 hit 
            if ((delta < 0) and ( x > 0) and              
               ( y < -1 / math.tan(theta) * x + R ) and 
               ( x**2 + (y - R)**2 < ( R - car_width*0.5)**2) and 
               ( x**2 + (y - R)**2 < ( R + car_width*0.5)**2)):
                hit += 1 / d
            elif ((delta ==0) and ( x > 0) and 
                 (x < look_ahead) and 
                 (math.fabs(y) < car_width * 0.5)):
                hit += 1 / d
            elif ((delta > 0) and ( x > 0) and              
               ( y >  1 / math.tan(theta) * x - R ) and 
               ( x**2 + (y + R)**2 < ( R - car_width*0.5)**2) and 
               ( x**2 + (y + R)**2 < ( R + car_width*0.5)**2)):
                hit += 1 / d
            
        inbound_hit.append(hit)
        hit = 0

    return inbound_hit
                    


               
        

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

def get_steer_cmd(sortedX , sortedY , cluster_num):
    global steering_ang , Ki,Kp,iError
    if  cluster_num >0 :
        distinguishLeftRight(sortedX ,sortedY)
        plt.cla()
        plt.plot(sortedX,sortedY,'ro',markersize = 2)
        plt.plot(L_cone_x,L_cone_y,'yo',markersize = 20)
        plt.plot(R_cone_x,R_cone_y,'bo',markersize = 20)
        plt.xlim([-1,5])
        plt.ylim([-5,5 ])
        plt.grid()
        plt.pause(0.001)
        midX = (L_cone_x + R_cone_x)*0.5
        midY = (L_cone_y + R_cone_y)*0.5
        err = -midY
        line_of_sight = -math.atan2(midY,midX)
        look_ahead = math.sqrt(math.hypot(midX,midY))
        # # # # PURE PURSUIT 
        steering_ang = purePursuit(line_of_sight,2) *math.pi /180
        
        # # # # PID CONTROL
        # iError = iError + err
        # steering_ang = Kp * err
        # print(steering_ang)
        # if steering_ang > 28:
        #     steering_ang = 28
        # elif steering_ang < -28:
        #     steering_ang = -28
        # steering_ang =steering_ang * 180 /math.pi

        # steering_ang = line_of_sight * 180 /math.pi 
        # print(f"left cone:{len(leftX)} ,rightt cone:{len(rightX)} ")
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

global k_p , k_i,p_err,i_err
k_p = 1
k_i = 10
p_err = 0
i_err =0
def stanley_controller(L1 , L2 , R1, R2):
    global k_p , k_i,p_err,i_err
    p1 = (L1 + R1) * 0.5
    p2 = (L2 + R2) * 0.5
    k =1
    velocity = 6
    track_heading = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    heading_error = normalize_angle(track_heading)
    crosstrack_error = -dist_point_line(0,0,p1[0] , p1[1] ,p2[0],p2[1])
    c_err = np.arctan2(k * crosstrack_error, velocity)
    # print(crosstrack_error)
    # delta =  3* crosstrack_error
    wx = (p2[0] + p1[0]) *0.5
    wy = (p2[1] + p1[1]) *0.5

    # eta = np.arctan2(p1[1],p1[0])
    # rng = math.hypot(p1[0],p1[1])
    # delta = impact_angle_gui(eta ,rng )

    i_err += crosstrack_error * 0.1
    delta = k_p * crosstrack_error +k_i * i_err
    delta = purePursuit(np.arctan2(p2[1], p2[0]),10)
    # delta = heading_error
    if delta > 28.0 *np.pi/180 :
        delta = 28.0 * np.pi/180
    elif delta < -28.0 *np.pi/180 :
        delta = -28.0 * np.pi/180

    return delta


    


class PROJECTION :
    
    def __init__(self) :
        
        self.projectionX = []
        self.projectionY = []
        self.lidarX      = []
        self.lidarY      = []
        self.candidates  = []
        self.avgDist     = 0
        self.distance    = 0
        self.distances   = []
        self.distVal     = 0
        self.maxDiff     = 20
        
    



    def lidarCamProjection(self, frame ) :                                 

        self.projectionX = []
        self.projectionY = []

        cX , cY , cN = clustering()
    
        # cX , cY = polar2xy(distance,azim)                      
        # cN = 761
        # plt.cla()
        # plt.plot(cX,cY,'ro',markersize = 2)
        # plt.xlim([-1,5])
        # plt.ylim([-5,5 ])
        # plt.grid()
        # plt.pause(0.001)

        for i in range(cN):

            XY_im  = 0
            XY_cam = 0

            lx = -cY[i]
            ly = cX[i]
            lz = 0
            Cz = ly + realRecede

            XY_cam = np.array([[lx],[ly],[lz],[1]])
            XY_im = 1/Cz * intMat @ extMat @ XY_cam

            xx = int(XY_im[0])
            yy = int(XY_im[1])

            self.projectionX.append(xx)
            self.projectionY.append(yy)

            cv.circle(frame, (round(xx) ,round(yy)), 8, colorYellow)
            


if __name__ == "__main__":
    sim = LIDAR_SM()
    lp = PROJECTION()
    sim.sharedmemory_open()
    frame = np.zeros((480 , 640 , 3) , np.uint8) # projection image
    zeroframe = np.zeros((480 , 640 , 3) , np.uint8) # projection image
    time_curr  = 0
    time_cnt   = 0
    time_stime = 0 
    time_ts    = 0.02
    time_final = 5000

    preDist = [5000 for _ in range(761)]
    # udp_cam = UDP_CAM_Parser(ip=params_cam["hostIP"], port=params_cam["localPort"], params_cam=params_cam)

   


    
    
    while (time_stime < time_final) :
        # if udp_cam.is_img==True :
        #     frame = udp_cam.raw_img
        time_start = time.time()
        sim.main(time_cnt)
        # lp.lidarCamProjection(frame)
        # cv.imshow("ang",frame)
        # np.copyto(frame,zeroframe)
        # drawnow(clustering)
        cX , cY , cN = clustering()
        L1, L2, R1,R2 = distinguishLeftRight(cX, cY)
        p1 = (L1 + R1) * 0.5
        p2 = (L2 + R2) * 0.5
        midline = np.stack((p1, p2), axis= 1)
        plt.cla()
        plt.plot(cX,cY , 'ro',markersize = 4)
        plt.plot(L1[0] , L1[1] ,'yo',markersize = 10)
        plt.plot(L2[0] , L2[1] ,'yo',markersize = 10)
        plt.plot(R1[0] , R1[1] ,'bo',markersize = 10)
        plt.plot(R2[0] , R2[1] ,'bo',markersize = 10)
        plt.plot(midline[0] , midline[1] , 'k-',markersize = 4)
        plt.xlim([-1,10])
        plt.ylim([-10,10 ])
        plt.grid()
        plt.pause(0.0001)
        steering_ang = -stanley_controller(L1, L2, R1, R2) * 180 /np.pi
        # print(steering_ang)

        # while(1):
        #     time_curr = time.time()
        #     time_del = time_curr - time_start - time_stime
        #     if (time_del > time_ts):
        #         time_cnt   += 1
        #         time_stime =  time_cnt*time_ts
        #        
        #         break
            
        # drawnow(showplot)
        time_cnt   += 1

        time_end = time.time()
        time_del = time_end-time_start
        # print(f'loop time:{time_del:.4f}s')
        # if cv.waitKey(1) & 0xFF == ord('q'):
        #     break

    sim.sharedmemory_close()



