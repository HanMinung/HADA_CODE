import numpy as np
import time
from scripts.guidance.guidance import *
from sklearn.cluster import DBSCAN
from scripts.cam_tongshin import *

# import HADA.lidar3d_py as lidar3d

log_occ_prob = [0, 0, 0, 0, 0, 0]  # 주차공간 점유도 확률 (로그)
occ_prob = [0, 0, 0, 0, 0, 0]  # 주차공간 점유도 확률


def removeOutliers(dist):
    n = len(dist)
    for i in range(2, n - 1):
        if abs(dist[i - 1] - dist[i]) > 0.5 and abs(dist[i + 1] - dist[i]) > 0.5 or dist[i] == 0:
            dist[i] = 20
    for i in range(761):
        if dist[i] >= 20 :
            dist[i] = 20

    return dist


def lidar_normalize(pc):
    pc = pc / 5000

    return pc


def polar2xy(dist, az):
    n = len(az)
    x = np.zeros(n)
    y = np.zeros(n)
    d2r = np.pi / 180
    for i in range(n):
        x[i] = dist[i] * math.cos(az[i] * d2r)
        y[i] = dist[i] * math.sin(az[i] * d2r)
    return x, y


def vff_path_planning(pc, steer_cmd):
    sigma = 10  # deg
    extend_obs = 1  # 확장할 크기
    pc = removeOutliers(pc)
    pc_normed = lidar_normalize(pc)
    azim = np.linspace(95, -95, 761)
    extend_data = np.copy(pc_normed)
    sff = [np.exp(-(x - (-steer_cmd * 180 / np.pi)) ** 2 / (2 * sigma)) for x in azim]
    # sff =[np.exp(-(x)**2 / (2 * sigma)) for x in  azim]
    # extend_data = gaussian_filter1d(pc_normed,30) *2 -1
    w = 0.17
    w_steer = 0.7
    for i in range(len(pc)):
        esize = 30  # round((extend_obs * 180)/(np.pi * pc[i]))
        for j in range(i - esize, i + esize):
            if j >= 0 and j < len(pc):
                if extend_data[j] > pc_normed[i]:
                    extend_data[j] = pc_normed[i]
    # extend_data = gaussian_filter1d(extend_data , 60) 
    # extend_data = moving_average(extend_data)
    iff = [w * sff[i] + (1 - w) * extend_data[i] for i in range(761)]
    # iff = gaussian_filter1d(iff,50)
    ind = np.argmax(iff)
    delta = (ind - 380) * 0.25 * np.pi / 180
    delta = w_steer * delta + (1 - w_steer) * steer_cmd
    delta = max(min(delta, 28 * np.pi / 180), -28 * np.pi / 180)
    # print(delta)   
    return extend_data, iff, delta


def vector_force_field(azim, pc, steer_cmd):
    # F_target : attraction force of target point
    # F_obs    : repulsive force of obstacle
    # F_result : sum of Ft and Fo

    # 척력 계산 :  각 센서값의 거리에 반비례 F = 1/(r+1) 
    # 타겟 인력 계산
    # azim = np.linspace(95,-95,761) # morai
    pc = removeOutliers(pc)  # 0~ 10 [m] range 
    x, y = polar2xy(pc, azim)
    k_obs = 0.2
    k_car = 0.2
    k_target = -0.2
    obstacle_vec = [0, 0]
    target_vec = [0, 1]
    car_vec = [0, 0]

    for i in range(761) : 
        
        if azim[i] > -35 and azim[i] < 35 and pc[i] < 3.5 and pc[i] > 0: # +- 40 도 이내의 점군에 대해서 척력계산
            obstacle_vec[0] += k_obs * (50 / pc[i]) * y[i]
            
        elif pc[i] < 2 and pc[i] > 0:                                  # +- 40도 밖의 점군에 대해서 척력 계산
            obstacle_vec[0] += k_obs * (1 / pc[i]) * y[i]

    # obstacle_vec[0] = min(max(-28 , obstacle_vec[0]), 28)
    w = 0.7
    car_vec[0] = 1 * (steer_cmd) + (1 - w) * obstacle_vec[0]
    car_vec[0] = min(max(-28, car_vec[0]), 28)
    car_vec[1] = k_car * (target_vec[1] + obstacle_vec[1])

    return car_vec[0]


def affine_transform(theta, x, y, px, py):  # 전역좌표계 상의 정점 -> 동체좌표계로 변환
    affine_x = math.cos(theta) * (x - px) + math.sin(theta) * (y - py)
    affine_y = -math.sin(theta) * (x - px) + math.cos(theta) * (y - py)
    return affine_x, affine_y


def parking_space_detection(pc, azim, px, py, yaw):
    # binary bayes algorithm
    parking_space = [[37.23932911, 126.7732837, 88.01],
                     [37.23935263, 126.7733001, 88.01],
                     [37.23937558, 126.7733138, 88.01],
                     [37.23939926, 126.7733309, 88.01],
                     [37.23942181, 126.7733439, 88.01],
                     [37.23944591, 126.773359, 88.01]]
    theta = (yaw) * np.pi / 180
    park_width = 2.5
    park_depth = 2.36
    park_x, park_y = waypoint_to_xy(parking_space)
    park_enter_x = [park_x[i] - park_depth * math.cos((90 - 88.01) * np.pi / 180) for i in range(6)]
    park_enter_y = [park_y[i] - park_depth * math.sin((90 - 88.01) * np.pi / 180) for i in range(6)]

    park_body_x = []
    park_body_y = []
    park_enter_body_x = []
    park_enter_body_y = []
    pc = removeOutliers(pc)
    pc_x, pc_y = polar2xy(pc, azim)

    p_occ = 0.65  # 점유 확률

    l_occ = math.log10(p_occ / (1 - p_occ))
    l_free = -l_occ
    obs_hit = 0
    # 주차구역을 전역좌표계에서 동체좌표계로 변환
    # 내 전역좌표 = (px,py) ,전역좌표계상의 임의의 점 (x_g , y_g)
    # 회전 고려 안하고 좌표만 변환 ( x_g - px, y_g - py)
    # yaw 가 30도라면?
    # [cos(theta)  sin(theta)]  [x_g - px]
    # [-sin(theta) cos(theta)]  [y_g - py]
    # 반시계방향 양수 각도라면 -theta 가되어야함

    for i in range(len(park_x)):
        rotated_x, rotated_y = affine_transform(theta, park_x[i], park_y[i], px, py)
        rotated_enter_x, rotated_enter_y = affine_transform(theta, park_enter_x[i], park_enter_y[i], px, py)
        park_body_x.append(rotated_x)
        park_body_y.append(rotated_y)
        park_enter_body_x.append(rotated_enter_x)
        park_enter_body_y.append(rotated_enter_y)

    # 주차구역 진입점과 반경 1미터 내에 라이다 측정값이 있으면 제외
    # 761개 중 0.5도 resolution
    for i in range(6):  # 6개 주차구역
        dx = [park_enter_body_x[i] - icx for icx in pc_x]
        dy = [park_enter_body_y[i] - icy for icy in pc_y]
        d = np.hypot(dx, dy)  # 각 주차구역 진입점에서 라이다 측정값 까지 거리
        for j in range(761):
            if d[j] < 1:
                obs_hit += 1

        log_occ_prob[i] += obs_hit * l_occ  # 주차구역이 점유되어있다고 판단되면 점군의 개수만큼 로그 오즈를 더함
        log_occ_prob[i] = min(log_occ_prob[i], 10)
        occ_prob[i] = 1 - 1 / (1 + math.exp(log_occ_prob[i]))  # 확률 복구 ( -inf ~ inf -> 0 ~ 1)

        obs_hit = 0
    print(occ_prob)

    return park_body_x, park_body_y, park_enter_body_x, park_enter_body_y, pc_x, pc_y

    # 라이다 데이터와 동시에 출력


class TangPark:  # 사선주차 클래스
    def __init__(self):
        self.is_park_fix = 0
        self.stop_flag = 0
        self.park_success = 0
        self.back_to_track = 0
        self.parking_index = 0
        self.park_end = 0
        self.gear = 0
        self.wait_cnt = 0
        self.park_start = [37.23928831, 126.77319631]
        self.parking_space = [[37.23932911, 126.7732837, 88.01],  # 시작점에서 가까운순서
                              [37.23935263, 126.7733001, 88.01],
                              [37.23937558, 126.7733138, 88.01],
                              [37.23939926, 126.7733309, 88.01],
                              [37.23942181, 126.7733439, 88.01],
                              [37.23944591, 126.773359, 88.01]]

        self.park_checkpoint = [[37.23933218, 126.7732236],
                                [37.2393569, 126.7732402],
                                [37.23938235, 126.7732571],
                                [37.23940585, 126.7732722],
                                [37.2394291, 126.7732884],
                                [37.23945285, 126.7733017]]
        # self.park_checkpoint = [ [37.23933432,	126.7731994],
        #                         [37.239361,	126.7732175],
        #                         [37.23938401,	126.7732338],
        #                         [37.23940604,	126.7732489],
        #                         [37.23942978,	126.773267],
        #                         [37.23945548	,126.7732864]]

        self.park_wp_x, self.park_wp_y = [], []  # 주차 가이던스 웨이포인트
        self.park_depth = 2.36  # 사선주차공간 세로길이 절반 [m]
        self.park_width = 2.5
        self.park_x, self.park_y = waypoint_to_xy(self.parking_space)  # 주차구역 xy좌표계 상의 위치
        self.park_enter_x = [self.park_x[i] - self.park_depth * math.cos((90 - 88.01) * np.pi / 180) for i in
                             range(6)]  # 주차 구역 입구 중점
        self.park_enter_y = [self.park_y[i] - self.park_depth * math.sin((90 - 88.01) * np.pi / 180) for i in range(6)]
        self.park_extended_x = [self.park_x[i] - 6 * math.cos((90 - 88.01) * np.pi / 180) for i in
                                range(6)]  # 연장된 점 ( 주차구역 중심을 지나는 주차방향 직선과 차선 중심선의 교점)
        self.park_extended_y = [self.park_y[i] - 6 * math.sin((90 - 88.01) * np.pi / 180) for i in range(6)]
        self.park_checkpoint_x, self.park_checkpoint_y = waypoint_to_xy(self.park_checkpoint)  # 주차구역 감지했을떄 멈출 포인트
        self.p_occ = 0.65  # 점유 확률 p(occ | z)
        self.log_occ_prob = [0, 0, 0, 0, 0, 0]  # 주차공간 점유도 확률 (로그)
        self.occ_prob = [0, 0, 0, 0, 0, 0]  # 주차공간 점유도 확률
        self.l_occ = math.log10(self.p_occ / (1 - self.p_occ))  # 점유 로그 오즈 (log odds)
        self.wp_idx = 0

    def check_parking_space(self, px, py, yaw, pc, azim):  # 동체의 x,y,yaw
        park_body_x = []  # 동체좌표계 상의 주차공간 중점
        park_body_y = []
        park_enter_body_x = []  # 동체좌표계 상의 주차공간 입구 중점
        park_enter_body_y = []
        obs_hit = 0
        pc = removeOutliers(pc)
        pc_x, pc_y = polar2xy(pc, azim)
        theta = (yaw) * np.pi / 180

        # self.log_occ_prob = [0,0,0,0,0,0] # 주차공간 점유도 확률 (로그)
        # self.occ_prob = [ 0,0,0,0,0,0]  # 주차공간 점유도 확률 
        for i in range(6):  # 주차공간을 전역좌표계 ->동체좌표계로 변환 ( 라이다 데이터와 비교할 수 있게)
            rotated_x, rotated_y = affine_transform(theta, self.park_x[i], self.park_y[i], px, py)
            rotated_enter_x, rotated_enter_y = affine_transform(theta, self.park_enter_x[i], self.park_enter_y[i], px,
                                                                py)
            park_body_x.append(rotated_x)
            park_body_y.append(rotated_y)
            park_enter_body_x.append(rotated_enter_x)
            park_enter_body_y.append(rotated_enter_y)

        # 주차구역 진입점과 반경 1미터 내에 라이다 측정값이 있으면 제외

        for i in range(6):  # 6개 주차구역
            dx = [park_enter_body_x[i] - icx for icx in pc_x]
            dy = [park_enter_body_y[i] - icy for icy in pc_y]
            d = np.hypot(dx, dy)  # 각 주차구역 진입점에서 라이다 측정값 까지 거리
            for j in range(761):
                if d[j] < 1:
                    obs_hit += 1

            # 점유 판단 기준: 주차구역 입구 중점으로부터 1미터 반경에 물체가 감지되면 점유
            self.log_occ_prob[i] += obs_hit * self.l_occ  # 주차구역이 점유되어있다고 판단되면 점군의 개수만큼 로그 오즈를 더함
            if math.fabs(math.atan2(park_enter_body_y[i], park_enter_body_x[
                i])) < 95 * np.pi / 180 and obs_hit < 2:  # and np.hypot(px - self.park_x[i] ,py - self.park_y[i] ) <10:
                self.log_occ_prob[i] -= self.l_occ
            self.log_occ_prob[i] = min(self.log_occ_prob[i], 10)
            self.occ_prob[i] = 1 - 1 / (1 + math.exp(self.log_occ_prob[i]))  # 확률 복구 ( -inf ~ inf -> 0 ~ 1)

            obs_hit = 0

        # 
        # 각 주차구역마다 체크포인트 지정
        # 체크포인트에서 주차구역 판단했을떄 비어있으면 멈춤
        # park fix 플래그 켬 -> 주차 진행
        dx_park = [px - icx for icx in self.park_checkpoint_x]
        dy_park = [py - icy for icy in self.park_checkpoint_y]
        d_park = np.hypot(dx_park, dy_park)  # 내 위치와 주차구역 체크 포인트 사이 거리
        park_ind = np.argmin(d_park)  # 가장 가까운 체크포인트의 인덱스
        # print(self.occ_prob[park_ind])   
        # if check_min_dist < 1 and occ_prob[check_index] < 0.5 => stopflag and parkfix flag on
        if d_park[park_ind] < 1.1 and self.occ_prob[park_ind] < 0.1:
            self.stop_flag = 1
            self.is_park_fix = 1
            self.parking_index = park_ind
            # 주차 웨이포인트 생성
            self.park_wp_x = [self.park_extended_x[park_ind], self.park_enter_x[park_ind], self.park_x[park_ind]]
            self.park_wp_y = [self.park_extended_y[park_ind], self.park_enter_y[park_ind], self.park_y[park_ind]]
            print(park_ind)

        # 처음에 서서 0,1,2 번 체크 => 없으면 중간으로 가서 3,4,5 체크 yhp    
        return park_body_x, park_body_y, park_enter_body_x, park_enter_body_y, pc_x, pc_y

    def do_parking(self, px, py, yaw):

        # 뉴 주차 가이던스 웨이포인트 생성
        wp_x = self.park_wp_x
        wp_y = self.park_wp_y
        # print(f"wp_index: {self.wp_idx}") 
        Lf = 2

        dist_wp_car = np.hypot((wp_x[self.wp_idx] - px), (wp_y[self.wp_idx] - py))

        if (dist_wp_car < 0.8) and self.wp_idx < 2 and self.park_success == 0:  # wp 갱신
            self.wp_idx = self.wp_idx + 2

            if self.wp_idx > 2:
                self.wp_idx = 2

        if (np.hypot((wp_x[2] - px), (wp_y[2] - py)) < 1.2):  # 주차 성공 판단
            self.stop_flag = 1
            self.park_success = 1  # 주차 성공 플래그 
        else:
            self.stop_flag = 0
            # wp 인덱스 업데이트
        # 가이던스 수행
        gamma = yaw * np.pi / 180
        los = math.atan2(wp_y[self.wp_idx] - py, wp_x[self.wp_idx] - px) * np.pi / 180
        eta = gamma - los

        c1 = 2 * 1.1 * 0.095
        c2 = 0.095 ** 2
        vel = 5
        tgo = vel / Lf
        alpha = -(math.atan2(wp_y[self.wp_idx] - py, wp_x[self.wp_idx] - px) - yaw * np.pi / 180)
        # eta = gamma - los
        Kp = 1.0
        # delta = 2 * 1.04 * math.sin(alpha) / Lf
        delta = 5 * Kp * math.sin(eta) / Lf * 180 / np.pi
        # delta = math.atan2(2 * WB * math.sin(alpha) / Lf ,1)*180/ np.pi 
        # delta = min(max(delta,-28),28)
        # 주차 구역 들어가면 멈춤
        # plt.cla()
        # plt.plot(wp_x,wp_y,'bo')
        # plt.plot(px,py, 'ko')

        # 10초 셈

        # 다시 후진으로  똑같은 경로로 나옴
        if self.park_success and self.back_to_track:
            self.gear = 2
            self.stop_flag = 0
            alpha = -(math.atan2(wp_y[0] - py, wp_x[0] - px) - yaw * np.pi / 180)
            # eta = gamma - los
            Kp = 1.0
            # delta = 2 * 1.04 * math.sin(alpha) / Lf
            if alpha > 90 * np.pi / 180:
                alpha = np.pi - alpha
            elif alpha < - 90 * np.pi / 180:
                alpha = alpha + np.pi
            # delta =2 * Kp * math.sin(alpha) / Lf *180/ np.pi 
            delta = 0
            if np.hypot(wp_y[0] - py, wp_x[0] - px) < 1.4:  # 주차 미션 종료
                self.stop_flag = 1
                self.gear = 0
                self.park_end = 1
                print("Packing Success")

        # 오른쪽으로 꺾다가 다시 발산하는데 좌표계 한번 봐야할듯
        # 가이던스 방법 바꿔도 바뀌는거 같음

        return delta

    def parking_routine(self, x, y, yaw, dist, azim, cmd_steer, cmd_velocity):
        if self.park_end == 0:
            if self.is_park_fix == 0:
                park_x, park_y, park_enter_x, park_enter_y, pcx, pcy = self.check_parking_space(x, y, yaw, dist,
                                                                                                azim)  # 여기서 stop flag 한번 켜짐
                # data_IO.command_to_vehicle(cmd_gear ,  cmd_steer, cmd_velocity, 0)
            # send command to ERP
            elif self.is_park_fix and self.wait_cnt < 50:  # 주차 구역이 정해지면 멈춤 & 2초 대기 & stop flag 꺼짐
                # data_IO.command_to_vehicle(cmd_gear ,  cmd_steer, 0, 100)
                self.wait_cnt += 1
            elif self.is_park_fix and self.wait_cnt >= 50:
                cmd_steer = self.do_parking(x, y, yaw)  # 주차 성공하면 stop flag 켜짐

                if self.park_success == 1 and self.wait_cnt < 100:  #
                    self.wait_cnt += 1

                elif self.wait_cnt >= 100 and self.park_success == 1:
                    self.back_to_track = 1
                    cmd_velocity = 10

            return cmd_steer, cmd_velocity
        elif self.park_end == 1:
            self.stop_flag = 0
            cmd_velocity = 14
            return cmd_steer, cmd_velocity

def clustering(lidar_data, azimu):#X array Y array of Lidar
    
    x=[]
    y=[]
    clusterX = []    # cluster X좌표
    clusterY = []    # cluster Y좌표
    clusterDist = [] # cluster의 차로부터의 거리
    nu = removeOutliers(lidar_data)
    x , y = polar2xy(nu,azimu)
    
        
    X = np.stack((x,y), axis = 1)
    cluster = DBSCAN(eps=0.4,min_samples=7).fit(X)    
    labels = cluster.labels_
    cluster_num = max(labels)
    ROI_cluster_num =0
    for i in range(cluster_num+1):   #클러스터 갯수만큼 반복
        points_x = x[labels == i]
        points_y = y[labels == i]
        X = np.stack((points_x,points_y) , axis= 0)
        cov = np.cov(X)
        # var = variance_of_cluster(points_x,points_y)
        
        # print(cov)
        center_x = np.mean(points_x)
        center_y = np.mean(points_y)
        if math.hypot(center_x ,center_y) < 0.001:# or var > 0.005 :
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

class UTURN:
    def __init__(self) -> None:
        self.state = 0 
        self.time_start = 0
        self.stop_x , self.stop_y = 0,0
        self.new_tar_x , self.new_tar_y = 0,0 
    '''
    1 장애물 감지
    2. 웨이포인트 생성
    3. 장애물 주행
    4. 복귀
    '''
    
    def uturn(self,azim,dis ,x,y,traj,yaw,org_cmd  ):
        self.stopit(azim,dis)
        self.make_wp( x,y,traj)
        gearcmd, velcmd, steercmd, brakecmd = self.gogo(x,y,yaw,azim,dis,org_cmd)
       
        
        return gearcmd, velcmd, steercmd, brakecmd
        
    def stopit(self,azim,dis ):
        # 1차선의 점유도 계싼해서 막혀있는 곳 기준으로 웨이포인트 새로 생성
        # 1차선 길게 linspace 해서 각 점마다 반경 1미터 내 점유도 계산
        # 가장 인덱스 적은 점 기준으로 웨이포인트 생성
        check_point = [[37.24025968945677, 126.77522905465027 ], #start
                       [37.241561602187474,126.77517869901426]] #end
                
        maxdist = max(dis)
        dis = [0 if i == maxdist else i for i in dis ] #최대거리 0으로 초기화
        cX,cY,cN = clustering(dis,azim)
        for i in range(cN):
            if   -0.7 < cY[i] < 0.7 and 1 < cX[i] < 6 and self.state == 0:
                print("stopped")
                self.state = 1
        
    def make_wp(self , x,y,traj ): 
        third_line_lon =  126.775376626 # 3차선 경도
        third_line_x =  (126.775376626 - INIT_LON) * LON2METER
        if self.state ==1:
            # make wp
            self.stop_x = x # 내 현재 위치
            self.stop_y = y
            # 3차선에서 내 위도에 해당하는 웨이포인트 인덱스를 찾기
            dx = [third_line_x - icx for icx in traj.cx]
            dy = [self.stop_y - icy for icy in traj.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)           
            self.state = 2
            print("WP made")
            self.new_tar_x , self.new_tar_y = traj.cx[ind] ,traj.cy[ind]
    
    def gogo(self,x,y,yaw, azim,dis, org_cmd) :
        d = np.hypot(self.new_tar_x-x,self.new_tar_y-y)  
        if self.state ==2:
            org_cmd[2] = impact_Angle_steer_control(x,y,yaw , self.new_tar_x , self.new_tar_y)
            org_cmd[2] = vector_force_field(azim , dis,org_cmd[2])
            if d < 3:
              self.state = 3  
        elif self.state == 3   :
            print("back to guidance")

        return org_cmd[0],org_cmd[1],org_cmd[2],org_cmd[3] # gear vel steer brake
                   


# static obstacle class - minwoong add ( 9.16 ) 

class STATIC :
    
    def __init__(self) :
        
        self.static_mission_flag   =   False
        self.ref_yaw_off           =   None
        self.ref_yaw_org           =   None
        self.prev_yaw              =   0
        
        self.start_cnt             =   0
        self.end_cnt               =   0
        
        # self.cnt                   =   0
    
    # org cmd : steer, velocity
    def static_obs(self, azim, dist, yaw, org_cmd) :
        
        # print(self.static_mission_flag)
        
        # self.cnt += 1
        
        cluster_num, drum_num  =  0, 0
        roi_height             =  10
        
        roi_height = self.set_clustering_roi()        
        
        steer_cmd, vel_cmd = org_cmd[0], org_cmd[1]
        
        cluster_num = self.clustering(azim, dist, roi_height)
        
        drum_num = recv_data( RUBBER_TO_STATIC, 1 )
        
        self.check_static(cluster_num, drum_num)
        
        print(roi_height)
        
        # algorithm operates only when static_flag is "True"
        if self.static_mission_flag :
            
            vel_cmd = 6.0
            
            self.get_ref_yaw(yaw)
            
            yaw_cmd_steer = self.yaw_control(yaw, cluster_num)
            
            steer_cmd = vector_force_field(azim, dist, yaw_cmd_steer)
            
            # if self.cnt % 10 == 0 :
                
            # print(f"yaw : {round(yaw_cmd_steer, 2)},    static : {round(steer_cmd, 2)}\n\n")
            # print(self.static_mission_flag)
                        
            self.check_termination(cluster_num)
        
        return steer_cmd, vel_cmd
            
    
    def set_clustering_roi(self) :
        
        # print(self.static_mission_flag)
        
        if self.static_mission_flag  :  return 6
        
        else  :  return 10
    
    
    # get number of cluster in roi range [ setting : roi_x & roi_y ]
    def clustering(self, azim, dist, roi_height) :
        
        cluster_num   =   0
        epsilon       =   0.25
        min_sample    =   3
        roi_width     =   2.0
        
        dist = removeOutliers(dist)
        
        x_pos, y_pos = polar2xy(dist, azim)
        
        mask = (abs(x_pos) < roi_height) & (abs(y_pos) < roi_width)
        
        filtered_x_pos = x_pos[mask]
        filtered_y_pos = y_pos[mask]    
        
        stack = np.stack((filtered_x_pos, filtered_y_pos), axis = 1)
        
        if stack.shape[0] > 0:
            
            cluster = DBSCAN(eps = epsilon, min_samples = min_sample).fit(stack)
        
            labels = cluster.labels_
        
            cluster_num = max(labels)
            
        if cluster_num == 0 : return 0
        
        else                : return cluster_num     
    
              
    
    def check_static(self, cluster_num, drum_num) :
        
        if cluster_num >= 2 and drum_num >= 2 : self.start_cnt += 1
            
        if self.start_cnt >= 10 : self.static_mission_flag = True

    
    
    def get_ref_yaw(self, yaw) :
        
        yaw_offset = -5
        
        if self.ref_yaw_off is None : 
            
            self.ref_yaw_off   =  yaw + yaw_offset
            
            self.ref_yaw_org   =  yaw - yaw_offset
            
    
    def yaw_control(self, yaw, cluster_num) :
        
        gain        =   0.8
        tolerance   =   30.0
        
        if abs(yaw - self.ref_yaw_off) < tolerance :
            
            self.prev_yaw = yaw
            
        elif abs(yaw - self.ref_yaw_off) >= tolerance :
            
            yaw = self.prev_yaw
        
        if cluster_num == 0 : error = self.ref_yaw_org - yaw
        
        else : error = self.ref_yaw_off - yaw
        
        # if self.cnt % 10 == 0 :
        
        #     print("\n\n")
        #     print(f"reference yaw : {round(self.ref_yaw, 2)}" )
        #     print(f"current yaw   : {round(yaw, 2)}")
        #     print(f"prev yaw      : {round(self.prev_yaw, 2)}")
        #     print(f"error         : {round(error, 2)}")
        
        return gain * error
    
    
    def check_termination(self, cluster_num) :
        
        if cluster_num == 0    :  
            
            self.end_cnt += 1
        
        if self.end_cnt >= 50  :  
            
            self.start_cnt = 0
            self.static_mission_flag = False
        
        

# dynamic obstacle class - minwoong add ( 9.16 ) 

class DYNAMIC :

    def __init__(self) :

        self.driver = lidar3d.driver()
        self.driver.run()

        self.brake_time = None
        self.is_certain = 0
    
    
    def dynamic_obs(self) :

        lidar_channel = 8

        data = self.driver.get_channel_data(lidar_channel)

        lidar_data = np.copy(data)                  # do not use original data --> copy data to lidar_data

        cluster_n = self.clustering(lidar_data)

        brakecmd = self.control_platform(cluster_n)

        # plt.cla()
        # plt.plot(lidar_data[0], lidar_data[1], 'bo', markersize = 5)
        # plt.plot(cluster_x, cluster_y, 'ro', markersize = 8)
        # plt.grid(True)
        # plt.xlim(-4, 4)
        # plt.ylim(0, 5)
        # plt.pause(0.001)

        return brakecmd

        
    
    def dbScan(self, x_data, y_data) :

        cluster_x = []
        cluster_y = []

        epsilon   = 0.25
        minSample = 3

        roi_x     = 1.5
        roi_y     = 3.0

        stack   = np.stack((x_data, y_data), axis = 1)
        
        if stack.shape[0] > 0:
            
            cluster = DBSCAN(eps = epsilon, min_samples = minSample).fit(stack)
        
            labels = cluster.labels_
        
            cluster_n = max(labels)

            for Idx in range(cluster_n) :

                cluster_x_pos = np.mean(np.array(x_data)[np.array(labels) == Idx])
                cluster_y_pos = np.mean(np.array(y_data)[np.array(labels) == Idx])

                if abs(cluster_x_pos) < roi_x and abs(cluster_y_pos) < roi_y :

                    cluster_x.append(cluster_x_pos)
                    cluster_y.append(cluster_y_pos)

        return cluster_x, cluster_y

    
    def clustering(self, lidar_data) :

        try :

            cluster_x, cluster_y = self.dbScan(lidar_data[0], lidar_data[1])

            return len(cluster_x)
        
        except ValueError as e : 

            cluster_x, cluster_y = [], []

            return len(cluster_x)


    def control_platform(self, cluster_n) :

        cnt_thresh   = 100
        restart_time = 2.0 
        brake_cmd    = 0

        if cluster_n >= 1 : self.is_certain += 1
            
        if self.is_certain >= cnt_thresh :

            brake_cmd = 200
            
            if self.brake_time is None : self.brake_time = time.time()

            if self.brake_time is not None : 

                if time.time() - self.brake_time >= restart_time : 

                    self.is_certain = 0
                    brake_cmd       = 0
                    self.brake_time = None
                    
            
        return brake_cmd