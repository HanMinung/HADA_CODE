from scripts.dataio_for_all import *
from scripts.guidance.parapark import *
from scripts.guidance.delivery import *
from scripts.sensor.gps import *
from scripts.sensor.imu import *
from scripts.sensor.lidar import *
from scripts.serial_node import *
from scripts.cam_tongshin import *

from project_val import *
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
        self.init_heading = INIT_HEADING_AT_KCITY  # 초기헤딩값
        if mode == MOD_REAL:  # 실제 주행환경일떄
            device = 'com4'  # 시리얼 포트 넘버
            imu_port = "com20"  # imu 포트 넘버 
            self.command = erpSerial(device)  # 시리얼 통신 인스턴스
            self.GPS_REAL = GPS_SM()  # GPS 데이터 수신 공유메모리 인스턴스
            self.GPS_REAL.sharedmemory_open()  # 메모리맵파일 생성
            self.IMU_REAL = IMU_SM()  # IMU 데이터 수신 공유메모리 인스턴스
            self.IMU_REAL.sharedmemory_open()
            self.LIDAR_REAL = lidarSM()
            self.LIDAR_REAL.Lidar_SMopen()


        elif mode == MOD_MORAI:  # MORAI 주행환경일때
            # self.morai_com = planner()
            self.morai_com = MORAI_SM()  # MORAI 공유메모리 인스턴스
            self.morai_com.sharedmemory_open()

    def get_att_data(self):
        lat: float = 0.
        lon: float = 0.
        yaw: float = 0.

        if self.mode == MOD_REAL:  # 실제 주행환경일떄 자세데이터 수신 (위경도, 헤딩)
            lat, lon, h, GPS_qual = self.GPS_REAL.recv_data()
            yaw = self.IMU_REAL.recv_data() + self.init_heading

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

    def command_to_vehicle(self, gear, steer, velocity, brake):
        if self.mode == MOD_REAL:
            self.command.send_ctrl_cmd(gear, velocity, steer, brake)
        elif self.mode == MOD_MORAI:
            self.morai_com.cmd_to_MORAI(gear, velocity, steer, brake)

    # def __del__(self):
    #     if self.mode == MOD_REAL:
    #         self.GPS_REAL.sharedmemory_close()

    #     elif self.mode == MOD_MORAI:
    #         self.morai_com.sharedmemory_close()
    #     print("Program Terminated.")


class Mission:  #
    # 각 미션마다 주행속도 사전 정의
    # 가이던스 주행시 속도
    # 각 미션마다 웨이포인트 할당

    '''
    본선 미션
    1. 배달A 픽업
    2. 교차로 좌회전(비신호)
    3. 정적장애물 (소형)
    4. 교차로 우회전 (비신호)
    5. 교차로 좌회전 (신호)
    6. 교차로 직진 (신호)
    7. 정적장애물 (대형)
    8. 교차로 직진(신호)
    9. 배달B  
    10. 11. 교차로 좌회전(신호)
    12. 교차로 우회전(정지 후 출발)
    13.14. 교차로 직진(신호)
    15. 평행주차
    '''

    # 카메라 필요시 카메라 프로세스에 1 넘김
    # 카메라 필요없으면 0 
    def __init__(self, cx, cy):
        self.Delivery = [[37.2391121194106, 126.773079989008],  # 배달 픽업 시작
                         [37.2394904945581, 126.773326927361],  # 배달 픽업 끝
                         [37.2416866298285, 126.774517021633],  # 배달 드랍 시작
                         [37.2419420816915, 126.774571352587]]  # 배달 드랍 끝
        self.StaticObs = [[37.2397891426220, 126.773425652935],  # 소형 정적 시작
                          [37.2399343828844, 126.773094804741],  # 소형 정적 끝
                          [37.2407966484577, 126.774198068529],  # 대형 정적 시작
                          [37.2411601656377, 126.774413785686]]  # 대형 정적 끝
        self.TrafficLeft = [[37.2403017109888, 126.773345985601],  # 신호 좌회전 1 시작
                            [37.2401647119396, 126.773641060366],  # 신호 좌회전 1 끝
                            [37.2422629564403, 126.774463653281],  # 신호 좌회전 2 시작
                            [37.2425666230187, 126.774456261229],  # 신호 좌회전 2 끝
                            [37.2427901452489, 126.774144656705],  # 신호 좌회전 3 시작
                            [37.2427761181610, 126.773852587704]]  # 신호 좌회전 3 끝
        self.TrafficRight = [[]]
        self.TrafficStraight = [[37.2402068016252, 126.773800525134],  # 신호 직진 1 시작
                                [37.2404349818779, 126.773950205060],  # 신호 직진 1 끝
                                [37.2412103450348, 126.774437954826],  # 신호 직진 2 시작
                                [37.2415639635850, 126.774519089899],  # 신호 직진 2 끝
                                [37.2410313494839, 126.774234627389],  # 신호 직진 3 시작
                                [37.2406780000744, 126.774018152642],  # 신호 직진 3 끝
                                [37.2405308852286, 126.773943045604],  # 신호 직진 4 시작
                                [37.2402487182569, 126.773769319668]]  # 신호 직진 4 끝
        self.Park = [[]]

        self.Delivery_idx = self.calculate_mission_idx(cx, cy, self.Delivery)
        self.StaticObs_idx = self.calculate_mission_idx(cx, cy, self.StaticObs)
        self.TrafficStraight_idx = self.calculate_mission_idx(cx, cy, self.TrafficStraight)
        self.TrafficLeft_idx = self.calculate_mission_idx(cx, cy, self.TrafficLeft)

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

        # 배달픽업
        if (self.Delivery_idx[0] < my_ind < self.Delivery_idx[1]) :
            return MISSION_DELIVERY_A
        # 배달드랍
        elif(self.Delivery_idx[2] < my_ind < self.Delivery_idx[3]):
            return MISSION_DELIVERY_B

        # 정적장애물
        elif ((self.StaticObs_idx[0] < my_ind < self.StaticObs_idx[1]) or
              (self.StaticObs_idx[2] < my_ind < self.StaticObs_idx[3])):
            # print("정적")
            return MISSION_STATIC

        # 신호 직진
        elif ((self.TrafficStraight_idx[0] < my_ind < self.TrafficStraight_idx[1]) or
              (self.TrafficStraight_idx[2] < my_ind < self.TrafficStraight_idx[3]) or
              (self.TrafficStraight_idx[4] < my_ind < self.TrafficStraight_idx[5]) or
              (self.TrafficStraight_idx[6] < my_ind < self.TrafficStraight_idx[7])):
            # print("직진")
            return MISSION_STRAIGHT

        # 신호 좌회전  
        elif ((self.TrafficLeft_idx[0] < my_ind < self.TrafficLeft_idx[1]) or
              (self.TrafficLeft_idx[2] < my_ind < self.TrafficLeft_idx[3]) or
              (self.TrafficLeft_idx[4] < my_ind < self.TrafficLeft_idx[5])):
            # print("좌회전")
            return MISSION_TURNLEFT
        
        
        else:
            # print("가이던스")
            return MISSION_NONE

    def get_mission_vel(self, my_ind):  # 미션마다 속도
        pass


class Mission_GalSang:  #
    # 각 미션마다 주행속도 사전 정의
    # 가이던스 주행시 속도
    # 각 미션마다 웨이포인트 할당

    '''
    갈상 테스트
    1. 배달A 픽업
    2. 교차로 좌회전(비신호)
    3. 정적장애물 (소형)
    4. 교차로 우회전 (비신호)
    5. 교차로 좌회전 (신호)
    6. 교차로 직진 (신호)
    7. 정적장애물 (대형)
    8. 교차로 직진(신호)
    9. 배달B  
    10. 11. 교차로 좌회전(신호)
    12. 교차로 우회전(정지 후 출발)
    13.14. 교차로 직진(신호)
    15. 평행주차
    '''

    # 카메라 필요시 카메라 프로세스에 1 넘김
    # 카메라 필요없으면 0 
    def __init__(self, cx, cy):
        self.Delivery = [[36.10234131333333, 129.3845896883333],  # 배달 픽업 시작
                         [36.10251945365194,  129.3845020898018],  # 배달 픽업 끝
                         [36.102403943525225,  129.38442088085762] ,  # 배달 드랍 시작
                         [36.10215670410554,  129.3846770858146]]  # 배달 드랍 끝
        


        self.Delivery_idx = self.calculate_mission_idx(cx, cy, self.Delivery)


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
        if 배달픽업 : return MISSION_DELIVERY_A
        
        if 배달드랍 : return MISSION_DELIVERY_B
        '''
        # 배달미션
        if ((self.Delivery_idx[0] < my_ind < self.Delivery_idx[1]) ): # 배달 픽업
            return MISSION_DELIVERY_A
        
        if ((self.Delivery_idx[2] < my_ind < self.Delivery_idx[3])): # 배달 드롭
            return MISSION_DELIVERY_B

        else:
            
            return MISSION_NONE

    def get_mission_vel(self, my_ind):  # 미션마다 속도
        pass


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
        
        
        self.Uturn = [[]]
        self.Uturn_idx = self.calculate_mission_idx(cx, cy, self.Uturn)
        self.Tunnel = [[]]
        self.Tunnel_idx = self.calculate_mission_idx(cx, cy, self.Tunnel)

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
        if ((self.Delivery_idx[0] < my_ind < self.Delivery_idx[1]) ): # 배달 픽업
            return MISSION_DELIVERY_A
        
        if ((self.Delivery_idx[2] < my_ind < self.Delivery_idx[3])): # 배달 드롭
            return MISSION_DELIVERY_B

        else:
            
            return MISSION_NONE

    def get_mission_vel(self, my_ind):  # 미션마다 속도
        pass



def isEnd() -> bool:
    if keyboard.is_pressed('e'):
        return True
    else:
        return False
