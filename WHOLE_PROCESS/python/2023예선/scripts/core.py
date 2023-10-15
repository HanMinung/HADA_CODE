from scripts.dataio_for_all import *
from scripts.sensor.gps import *
from scripts.sensor.imu import *
from scripts.sensor.lidar import *
from scripts.guidance.guidance import *
from scripts.guidance.local_path_planning import *
from scripts.cam_tongshin import *
from scripts.sensor.Encoder import *
from scripts.navigation import *
import time
from project_val import *
from scripts.serial_node import *
import keyboard


class WayMaker:  # 웨이포인트 읽고, xy좌표계 변환 후, 3차 스플라인
    def __init__(self, filname):
        self.wp_file_name = filname  # "waypoint/HADA3BONSEONV4.csv"
        self.wp_data = read_csv_file(self.wp_file_name)
        self.wp_x, self.wp_y = waypoint_to_xy(self.wp_data)
        
        try:
            self.cx, self.cy, _,_,_ = calc_spline_course(self.wp_x, self.wp_y, ds=0.1)
        except Exception:
            self.cx, self.cy = self.wp_x, self.wp_y
        
        self.Trajectory = TargetCourse(self.cx, self.cy)


class RealTime:
    def __init__(self):
        self.time_cnt = 0
        self.simtime = 0
        self.Ts = 0.01
        self.time_curr = 0
        self.time_prev = 0



class DataCommunication:  # 공유메모리 열기,센서데이터 수신 , 조종 명령 송신
    def __init__(self, mode: int):
        self.mode: int = mode
        self.gpsqual = 0
        self.yawbyus = 0
        #parallel park
        # self.init_heading = -151.8139403464505
        self.init_heading =  INIT_HEADING_AT_KCITY  # 초기헤딩값
        self.init_heading = 40
        if mode == MOD_REAL:  # 실제 주행환경일떄
            device = 'com4'  # 시리얼 포트 넘버
            self.command = erpSerial(device)  # 시리얼 통신 인스턴스
            self.GPS_REAL = GPS_SM()  # GPS 데이터 수신 공유메모리 인스턴스
            self.GPS_REAL.sharedmemory_open()  # 메모리맵파일 생성
            self.IMU_REAL = IMU_SM()  # IMU 데이터 수신 공유메모리 인스턴스
            self.IMU_REAL.sharedmemory_open()
            self.LIDAR_REAL = lidarSM()
            self.LIDAR_REAL.Lidar_SMopen()
            # self.ERP_IO = EncoderSM()


        elif mode == MOD_MORAI:  # MORAI 주행환경일때
            # self.morai_com = planner()
            self.morai_com = MORAI_SM()  # MORAI 공유메모리 인스턴스
            self.morai_com.sharedmemory_open()

    def get_att_data(self):
        lat: float = 0.
        lon: float = 0.
        yaw: float = 0.

        if self.mode == MOD_REAL:  # 실제 주행환경일떄 자세데이터 수신 (위경도, 헤딩)
            lat, lon, h, self.gpsqual = self.GPS_REAL.recv_data()
            yaw = self.IMU_REAL.recv_data() + self.init_heading
            # if yaw>180:
            #     yaw = 360 - yaw
            # elif yaw < -180:
            #     yaw = yaw + 360
        elif self.mode == MOD_MORAI:  # MORAI 주행환경일떄 자세데이터 수신 (위경도, 헤딩)
            # lat,lon,yaw = self.morai_com.lat , self.morai_com.lon,self.morai_com.yaw
            lat, lon, yaw, _ = self.morai_com.recv_data_MORAI()
        x, y, yaw = my_xy_pos(lat, lon, yaw)
        
        return lat, lon, x, y, yaw

    def get_lidar_data(self) -> tuple[np.ndarray, float]:  # 2d lidar 점군 데이타 수신
        azim: np.ndarray = np.ndarray([])
        dist: float = 0.
        if self.mode == MOD_MORAI:
            azim = np.linspace(95, -95, 761)
            _, _, _, dist = self.morai_com.recv_data_MORAI()
        if self.mode == MOD_REAL:
            azim = np.linspace(-95, 95, 761)
            dist = self.LIDAR_REAL.importData()

        return azim, dist
    
    def get_velocity(self):# 0921 
        # vel = self.ERP_IO.recv_velocity()# 0921 
        
        # ROLL BACKED AT 0929-21:47
        vel = self.command.speed / 10
        return vel
        
    def command_to_vehicle(self, gear, steer, velocity, brake):
        if self.mode == MOD_REAL:
            # ROLL BACKED AT 0929-21:47
            self.command.send_ctrl_cmd(gear, velocity, steer, brake)
            # self.ERP_IO.send_ctrl_cmd(gear,steer,velocity,brake)   # 0921 
        elif self.mode == MOD_MORAI:
            self.morai_com.cmd_to_MORAI(gear, velocity, steer, brake)

    # def __del__(self):
    #     if self.mode == MOD_REAL:
    #         self.GPS_REAL.sharedmemory_close()

    #     elif self.mode == MOD_MORAI:
    #         self.morai_com.sharedmemory_close()
    #     print("Program Terminated.")


class Mission_YeahSon:  # 예선미션
    # 각 미션마다 주행속도 사전 정의
    # 가이던스 주행시 속도
    # 각 미션마다 웨이포인트 할당

    '''
    예선주행
    1. 가이던스
    2. 러버콘 유턴구역
    3. 터널(정적, 동적, 통신음영)

    '''

    # 카메라 필요시 카메라 프로세스에 1 넘김
    # 카메라 필요없으면 0 
    def __init__(self, cx, cy):
        
        # 유턴 미션구간 추가
        
        self.Uturn = [[37.24016170148196, 126.77522934561435], # 유턴 시작점
                      [ 37.24086929494119,  126.77537375586974]] # 유턴 종료점
        self.Uturn_idx = self.calculate_mission_idx(cx, cy, self.Uturn)
        
        self.Tunnel = [[37.23890473342678,  126.77522321682333]] # 머지포인트 
        self.Tunnel_idx = self.calculate_mission_idx(cx, cy, self.Tunnel)
        self.Tunnel_flag = 0
        
    def calculate_mission_idx(self, cx, cy, mission_wp):  # 가장 가까운 웨이포인트의 인덱스 반환
        mission_indices = []
        wp_x, wp_y = waypoint_to_xy(mission_wp)
        for i in range(len(mission_wp)):
            dx = [wp_x[i] - icx for icx in cx]
            dy = [wp_y[i] - icy for icy in cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            mission_indices.append(ind)
        return mission_indices

    def current_mission(self, my_ind):
        '''
        if 유턴시작: return MISSION_DELIVERY_A
        
        if 터널시작 : return MISSION_DELIVERY_B
        '''
        # 배달미션
        if (( my_ind > self.Tunnel_idx[0] ) ): # 차선인식
            self.Tunnel_flag =  YEAH_TUNNEL  # GPS음영지역 통과시 위치정보가 이상하게 바뀌면서 웨이포인트 인덱스가 바뀔수 있는 경우
        
        if self.Tunnel_flag ==  YEAH_TUNNEL:
            return YEAH_TUNNEL
        elif  self.Uturn_idx[0] < my_ind < self.Uturn_idx[1]:
            return YEAH_UTURN
        else:
            return MISSION_NONE

    def get_mission_vel(self, my_ind):  # 미션마다 속도
        pass



def isEnd() -> bool:
    if keyboard.is_pressed('e'):
        return True
    else:
        return False


class Mission_Gal_YeahSon:  # 갈상예선 시나리오
    # 각 미션마다 주행속도 사전 정의
    # 가이던스 주행시 속도
    # 각 미션마다 웨이포인트 할당

    '''
    예선주행
    1. 가이던스
    2. 러버콘 유턴구역
    3. 터널(정적, 동적, 통신음영)

    '''

    # 카메라 필요시 카메라 프로세스에 1 넘김
    # 카메라 필요없으면 0 
    def __init__(self, cx, cy):
        
        # 유턴 미션구간 추가
        
        self.Uturn = [[ 36.10345230922429,  129.3860679915525], # 유턴 시작점
                      [  36.103668282861854,  129.3860245647902 ]] # 유턴 종료점
        self.Uturn_idx = self.calculate_mission_idx(cx, cy, self.Uturn)
        
        self.Tunnel = [[ 36.102466399511826,  129.38552606607246]] # 머지포인트 
        self.Tunnel_idx = self.calculate_mission_idx(cx, cy, self.Tunnel)
        self.Tunnel_flag = 0
        
    def calculate_mission_idx(self, cx, cy, mission_wp):  # 가장 가까운 웨이포인트의 인덱스 반환
        mission_indices = []
        wp_x, wp_y = waypoint_to_xy(mission_wp)
        for i in range(len(mission_wp)):
            dx = [wp_x[i] - icx for icx in cx]
            dy = [wp_y[i] - icy for icy in cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            mission_indices.append(ind)
        return mission_indices

    def current_mission(self, my_ind):
        '''
        if 유턴시작: return MISSION_DELIVERY_A
        
        if 터널시작 : return MISSION_DELIVERY_B
        '''
        # 배달미션
        if (( my_ind > self.Tunnel_idx[0] ) ): # 차선인식
            self.Tunnel_flag =  YEAH_TUNNEL  # GPS음영지역 통과시 위치정보가 이상하게 바뀌면서 웨이포인트 인덱스가 바뀔수 있는 경우
        
        if self.Tunnel_flag ==  YEAH_TUNNEL:
            return YEAH_TUNNEL
        elif  self.Uturn_idx[0] < my_ind < self.Uturn_idx[1]:
            return YEAH_UTURN
        else:
            return MISSION_NONE

    def get_mission_vel(self, my_ind):  # 미션마다 속도
        pass



def isEnd() -> bool:
    if keyboard.is_pressed('e'):
        return True
    else:
        return False
