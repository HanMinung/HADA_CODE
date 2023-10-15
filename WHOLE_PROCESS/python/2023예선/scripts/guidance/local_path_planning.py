import numpy as np
from scripts.guidance.guidance import *
from sklearn.cluster import DBSCAN
from scripts.cam_tongshin import *
from scripts.guidance.calibration_var import *
import matplotlib.pyplot as plt
import time
import HADA.lidar3d_py as lidar3d


def removeOutliers(dist):
    n = len(dist)
    for i in range(2, n - 1):
        if abs(dist[i - 1] - dist[i]) > 0.5 and abs(dist[i + 1] - dist[i]) > 0.5 or dist[i] == 0:
            dist[i] = 10
    for i in range(761):
        if dist[i] >= 10 :
            dist[i] = 10

        
            

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

    for i in range(761):
        if azim[i] > -40 and azim[i] < 40 and pc[i] < 4 and pc[i] > 0: # +- 40 도 이내의 점군에 대해서 척력계산
            obstacle_vec[0] += k_obs * (40 / pc[i]) * y[i]
            
        elif pc[i] < 2 and pc[i] > 0:                                  # +- 40도 밖의 점군에 대해서 척력 계산
            obstacle_vec[0] += k_obs * (10 / pc[i]) * y[i]

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




def clustering(lidar_data, azimu):#X array Y array of Lidar
    
    x=[]
    y=[]
    clusterX = []    # cluster X좌표
    clusterY = []    # cluster Y좌표
    clusterDist = [] # cluster의 차로부터의 거리
    nu = removeOutliers(lidar_data)
    x , y = polar2xy(nu,azimu)
    
        
    X = np.stack((x,y), axis = 1)
    cluster = DBSCAN(eps= 0.5, min_samples=3).fit(X)    
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



def inter1d(start_point, end_point , n_point):


    # 보간된 점들을 저장할 리스트 생성
    interpolated_points = []

    # 보간할 간격 계산
    dx = (end_point[0] - start_point[0]) / n_point
    dy = (end_point[1] - start_point[1]) / n_point

    # 보간 실행
    for i in range(n_point + 1):
        x = start_point[0] + i * dx
        y = start_point[1] + i * dy
        interpolated_points.append((x, y))


    return interpolated_points
    #이 코드는 시작점과 끝점을 정의하고, 보간할 점의 개수를 설정한 후, 
    # 보간된 점을 계산하여 interpolated_points 리스트에 저장하고 출력합니다.
    # 이렇게 하면 start_point에서 시작하여 end_point로 끝나는 선분을 n등분한 보간된 점들을 얻을 수 있습니다.

def get_min_idx(my_list):
    min_index = float('inf')  # 무한대로 초기화
    for index, value in enumerate(my_list):
        if value >= 0.95:
            min_index = min(min_index, index)

    # 결과 출력
    if min_index != float('inf'):
        # print(f"1 이상의 원소 중에서 가장 낮은 인덱스: {min_index}")
        return min_index
    else:
        # print("1 이상의 원소가 없습니다.")
        return False


                
class UTURN_2:
    def __init__(self) -> None:
        self.state = 0 
        self.time_start = 0
        self.stop_x , self.stop_y = 0,0
        self.new_tar_x , self.new_tar_y = 0,0 
        self.occ_prob = [0 for i in range(70)]
        self.log_occ_prob = [0 for i in range(70)]
        self.l_occ = math.log10(0.65 / (1 - 0.65))  
        self.check_point = [[ 37.24032725934774,  126.7752246277481], #start
                            [ 37.24165282432525, 126.77517420173152 ]]
        self.blocked_idx = False
        
        lx,ly = waypoint_to_xy(self.check_point) # 막혀있는지 판단할 시작점과 종점의 xy좌표
        grid = inter1d([lx[0],ly[0]], [lx[1],ly[1]] , 70 )
        grid = np.array(grid)
        self.grid = grid.T
        
    def uturn(self,azim,dis ,x,y,traj,yaw,org_cmd  ):
        self.stopit(azim,dis,x,y,yaw)
        self.make_wp( x,y,traj)
        gearcmd, velcmd, steercmd, brakecmd = self.gogo(x,y,yaw,azim,dis,org_cmd)
       
        
        return gearcmd, velcmd, steercmd, brakecmd
        
    def stopit(self,azim,dis,px,py,yaw  ):
        '''
        유턴 알고리듬
        1차선 중심선을 잇는 직선 과
        라이다 센서값의 거리가 일정 이하 차이나면
        '''
        rot_grid_x = []
        rot_grid_y = []
        obs_hit = 0
        pc = removeOutliers(dis)
        pc_x, pc_y = polar2xy(pc, azim)

        theta = (90 - yaw) * np.pi / 180
        # plt.cla()
        # plt.plot(traj.cx, traj.cy , "ko")
        # plt.plot(grid[0],grid[1],'bo',markersize = 3)
        # plt.pause(0.001)   
        # disarray = [get_point_line_dist(i) for ]
        
        if self.state == 0:
            # 점유도 체크
            for Idx in range(70):
                
                rotated_enter_x, rotated_enter_y = affine_transform(theta, self.grid[0][Idx],
                                                                    self.grid[1][Idx], px + 1.6 * cos(theta), py+ 1.6 * sin(theta))

                rot_grid_x.append(rotated_enter_x)
                rot_grid_y.append(rotated_enter_y)
            for Idx in range(70):

                dx = [rot_grid_x[Idx] - icx for icx in pc_x]
                dy = [rot_grid_y[Idx] - icy for icy in pc_y]

                d = np.hypot(dx, dy)  # 각 그리드에서 라이다 측정값 까지 거리
                for Icx in range(761):
                    if d[Icx] < 0.5:
                        obs_hit += 1
        
                # 점유 판단 기준: 그리드 점으로부터 1미터 반경에 물체가 감지되면 점유
                if( math.fabs(math.atan2(rot_grid_y[Idx], rot_grid_x[Idx])) < 95 * np.pi / 180 and np.hypot(px - self.grid[0][Idx] ,py - self.grid[1][Idx] ) <9):
                    self.log_occ_prob[Idx] += obs_hit * self.l_occ  # 주차구역이 점유되어있다고 판단되면 점군의 개수만큼 로그 오즈를 더함
                
                elif( math.fabs(math.atan2(rot_grid_y[Idx], rot_grid_x[Idx])) < 95 * np.pi / 180 
                    and obs_hit < 2 and np.hypot(px - self.grid[0][Idx] ,py - self.grid[1][Idx] ) <9):
                    self.log_occ_prob[Idx] -= self.l_occ
                self.log_occ_prob[Idx] = min(self.log_occ_prob[Idx], 10)
                self.occ_prob[Idx] = 1 - 1 / (1 + math.exp(self.log_occ_prob[Idx]))  # 확률 복구 ( -inf ~ inf -> 0 ~ 1)
                
                obs_hit = 0       
            self.blocked_idx = get_min_idx(self.occ_prob)    
            
            if self.blocked_idx != False:
                self.state = 1
                print("Construction Detected")
        # plt.plot(pc_x,pc_y,'bo')
        # plt.plot(rot_grid_x,rot_grid_y, 'ko')
        # plt.grid()
        # plt.xlim(-1,10)
        # plt.ylim(-10,10)
        # plt.bar(range(70), self.occ_prob)
        # plt.pause(0.0001)
    
        
    def make_wp(self , x,y,traj ): 
        third_line_lon =  126.775376626 # 3차선 경도
        third_line_x =  ( 126.7753921047 - INIT_LON) * LON2METER
        if self.state == 1:
            # make wp
            self.stop_x = x # 내 현재 위치
            self.stop_y = self.grid[1][self.blocked_idx] 
            # 3차선에서 내 위도에 해당하는 웨이포인트 인덱스를 찾기
            dx = [third_line_x - icx for icx in traj.cx]
            dy = [self.stop_y - icy for icy in traj.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)           
            self.state = 2
            print("WP made")
            self.new_tar_x , self.new_tar_y = traj.cx[ind] ,traj.cy[ind]
        # plt.cla()
        # plt.plot(self.stop_x,self.stop_y,'co',markersize = 15)    
        # plt.plot(x,y,'ko')
        # plt.pause(0.0001)
        
    def gogo(self,x,y,yaw, azim, dis, org_cmd) :
        d = np.hypot(self.new_tar_x-x,self.new_tar_y-y)  
        third_line_x =  (126.77533192975078 - INIT_LON) * LON2METER
        # print(d)
        if self.state == 2:
            org_cmd[2] = impact_Angle_steer_control(x,y,yaw , self.new_tar_x , self.new_tar_y)
            org_cmd[2] = vector_force_field(azim , dis ,org_cmd[2])
            # print(org_cmd)
            if third_line_x -2.5 <  x < third_line_x +2.5 :
                self.state = 3  
        elif self.state == 3   :
            print("back to guidance")

        return org_cmd[0],org_cmd[1],org_cmd[2],org_cmd[3] # gear vel steer brake
                           
class UTURN_GALSANG:
    def __init__(self) -> None:
        self.state = 0 
        self.time_start = 0
        self.stop_x , self.stop_y = 0,0
        self.new_tar_x , self.new_tar_y = 0,0 
        self.n_grid = 40
        self.occ_prob = [0 for i in range(self.n_grid)]
        self.log_occ_prob = [0 for i in range(self.n_grid)]
        self.l_occ = math.log10(0.65 / (1 - 0.65))  
        self.check_point = [[ 36.10363629031556,  129.38610409010167], #start
                            [  36.103966853165936,  129.38624454718527]]
        self.blocked_idx = False
        
        lx,ly = waypoint_to_xy(self.check_point) # 막혀있는지 판단할 시작점과 종점의 xy좌표
        grid = inter1d([lx[0],ly[0]], [lx[1],ly[1]] , self.n_grid )
        grid = np.array(grid)
        self.grid = grid.T
        
    def uturn(self,azim,dis ,x,y,traj,yaw,org_cmd  ):
        self.stopit(azim,dis,x,y,yaw,traj)
        self.make_wp( x,y,traj)
        gearcmd, velcmd, steercmd, brakecmd = self.gogo(x,y,yaw,azim,dis,org_cmd)
       
        
        return gearcmd, velcmd, steercmd, brakecmd
        
    def stopit(self,azim,dis,px,py,yaw ,traj ):
        '''
        유턴 알고리듬
        1차선 중심선을 잇는 직선 과
        라이다 센서값의 거리가 일정 이하 차이나면
        '''
        rot_grid_x = []
        rot_grid_y = []
        obs_hit = 0
        pc = removeOutliers(dis)
        pc_x, pc_y = polar2xy(pc, azim)

        theta = (90 - yaw) * np.pi / 180
        plt.cla()
        # plt.plot(traj.cx, traj.cy , "ko")
        # plt.plot(self.grid[0],self.grid[1],'bo',markersize = 3)
        # plt.pause(0.001)   
        # disarray = [get_point_line_dist(i) for ]
        
        if self.state == 0:
            # 점유도 체크
            for Idx in range(self.n_grid):
                
                rotated_enter_x, rotated_enter_y = affine_transform(theta, self.grid[0][Idx],
                                                                    self.grid[1][Idx], px + 1.6 * cos(theta), py+ 1.6 * sin(theta))

                rot_grid_x.append(rotated_enter_x)
                rot_grid_y.append(rotated_enter_y)
            for Idx in range(self.n_grid):

                dx = [rot_grid_x[Idx] - icx for icx in pc_x]
                dy = [rot_grid_y[Idx] - icy for icy in pc_y]

                d = np.hypot(dx, dy)  # 각 그리드에서 라이다 측정값 까지 거리
                for Icx in range(761):
                    if d[Icx] < 1:
                        obs_hit += 1
        
                # 점유 판단 기준: 그리드 점으로부터 1미터 반경에 물체가 감지되면 점유
                if( math.fabs(math.atan2(rot_grid_y[Idx], rot_grid_x[Idx])) < 95 * np.pi / 180 and np.hypot(px - self.grid[0][Idx] ,py - self.grid[1][Idx] ) <9):
                    self.log_occ_prob[Idx] += obs_hit * self.l_occ  # 주차구역이 점유되어있다고 판단되면 점군의 개수만큼 로그 오즈를 더함
                
                elif( math.fabs(math.atan2(rot_grid_y[Idx], rot_grid_x[Idx])) < 95 * np.pi / 180 
                    and obs_hit < 2 and np.hypot(px - self.grid[0][Idx] ,py - self.grid[1][Idx] ) <9):
                    self.log_occ_prob[Idx] -= self.l_occ
                self.log_occ_prob[Idx] = min(self.log_occ_prob[Idx], 10)
                self.occ_prob[Idx] = 1 - 1 / (1 + math.exp(self.log_occ_prob[Idx]))  # 확률 복구 ( -inf ~ inf -> 0 ~ 1)
                
                obs_hit = 0       
            self.blocked_idx = get_min_idx(self.occ_prob)    
            
            if self.blocked_idx != False:
                self.state = 1
                print("Construction Detected")
        # plt.plot(pc_x,pc_y,'bo')
        # plt.plot(rot_grid_x,rot_grid_y, 'ko')
        # plt.grid()
        # plt.xlim(-1,10)
        # plt.ylim(-10,10)
        plt.bar(range(40), self.occ_prob)
        plt.pause(0.0001)
        # print(self.occ_prob)
        
    def make_wp(self , x,y,traj ): 
        third_line_lon =  126.775376626 # 3차선 경도
        third_line_x =  (126.77533192975078 - INIT_LON) * LON2METER
        if self.state == 1:
            # make wp
            self.stop_x = x # 내 현재 위치
            self.stop_y = self.grid[1][self.blocked_idx] 
       
            self.state = 2
            print("WP made")
            self.new_tar_x = self.stop_x - 4
            self.new_tar_y = self.stop_y + 4 * math.tan(17 * np.pi / 180)
        # plt.cla()
        # plt.plot(self.stop_x,self.stop_y,'co',markersize = 15)    
        # plt.plot(x,y,'ko')
        # plt.pause(0.0001)
        
    def gogo(self,x,y,yaw, azim, dis, org_cmd) :
        d = np.hypot(self.new_tar_x-x,self.new_tar_y-y)  
        third_line_x =  (126.77533192975078 - INIT_LON) * LON2METER
        # print(d)
        next_line = [[36.103683590764604, 129.38610413345637],
                     [36.1039840956056, 129.38622702725112]]
        nlx,nly = waypoint_to_xy(next_line)
        m = (nly[1] - nly[0])/(nlx[1] - nlx[0])  # 직선의 기울기
        c =  nly[0] - m * nlx[0]
        if self.state == 2:
            org_cmd[2] = impact_Angle_steer_control(x,y,yaw , self.new_tar_x , self.new_tar_y)
            org_cmd[2] = vector_force_field(azim , dis ,org_cmd[2])
            # print(org_cmd)
            if y > (m * x + c ) : # 다음 차선에 있으면
                self.state = 3  
        elif self.state == 3   :
            print("back to guidance")

        return org_cmd[0],org_cmd[1],org_cmd[2],org_cmd[3] # gear vel steer brake
    
        
class STATIC():
    
    def __init__(self) :
        
        self.static_mission_flag:int   =   False
        self.object_center:list        =   []

        self.sm_send_flag          =   open_send_shm( STATIC_FALG_TO_LANE  , 4)
        self.sm_send_steer         =   open_send_shm( STATIC_STEER_TO_LANE , 4)
        
        self.print_cnt             =   0
        self.start_cnt             =   0
        self.end_cnt               =   0
        self.end_limit : int       =   50

        self.send_static_flag: int


    def static_obs(self, lidar_dist) :

        object_dist     = 1000
        pixel_x_thresh  = 10.0
        dist_thresh     = 10.0

        object_dist = self.get_object_dist(pixel_x_thresh, lidar_dist)

        self.check_static(object_dist, dist_thresh)

        self.send_static_flag = YEAH_TUNNEL
        
        if self.static_mission_flag and self.end_cnt <= self.end_limit :
                        
            self.send_static_flag = YEAH_TUNNEL_STATIC 

            self.check_termination(self.object_center)

        send_data(self.sm_send_flag,  (1,1) , [self.send_static_flag])

        self.print_cnt += 1

        if self.print_cnt % 20 == 0 :

            print("------------------------------------------------\n")
            print(f"OBJECT CENTER : {self.object_center      }           ")
            print(f"OBJECT DIST   : {round(object_dist,2)    }[m]        ")
            print(f"STATIC FLAG   : {self.static_mission_flag}           ")
            print(f"END CNT       : {self.end_cnt}")

        return self.end_cnt, self.send_static_flag
            

    def get_object_dist(self, pixel_x_thresh, lidar_dist) :
        
        azim                       = np.linspace(-8.5 ,182.5 ,761)
        min_diff_idx               = 0
        object_dist                = 1000
        x_pos, y_pos               = [], []
        # projection_x, projection_y = [], []
        pixel_coor                 = 0
        world_coor                 = 0
            
        self.object_center = recv_list( RUBBER_TO_STATIC, 2 )

        x_pos, y_pos = polar2xy(lidar_dist, azim)

        if self.object_center[0] != 0  and self.object_center[1] != 0 :

            for Idx in range(len(azim)):

                scaling_factor = y_pos[Idx] + real_recede

                world_coor = np.array([[x_pos[Idx]], [y_pos[Idx]], [0], [1]])
                pixel_coor = 1/scaling_factor * wide_intrinsic_mat @ extrinsic_mat @ world_coor

                pixel_coor_U = int(pixel_coor[0])
                # pixel_coor_V = int(pixel_coor[1])

                # projection_x.append(pixel_coor_U)
                # projection_y.append(pixel_coor_V)

                if abs(self.object_center[0] - pixel_coor_U) < pixel_x_thresh and self.object_center[1] > self.object_center[1] - 10 : min_diff_idx = Idx

            object_dist = math.sqrt(x_pos[min_diff_idx] **2 + y_pos[min_diff_idx] **2)

            if object_dist < 2.0 : object_dist = 1000

            # plt.cla()
            # plt.plot(projection_x, projection_y, 'bo', markersize = 3)
            # plt.plot(object_center[0], object_center[1], 'ro', markersize = 10)
            # plt.plot(projection_x[min_diff_idx], projection_y[min_diff_idx], 'go', markersize = 5)
            # plt.xlim([0, 640])
            # plt.ylim([0, 480])
            # plt.grid(True)
            # plt.pause(0.0001)
        
        else : object_dist = 1000

        return object_dist
              
    
    def check_static(self, object_dist, dist_thresh) :

        if object_dist <= dist_thresh : self.start_cnt += 1
            
        if self.start_cnt >= 3 : self.static_mission_flag = True
    
    
    def check_termination(self, object_center) :
        
        if object_center[0] == 0 and object_center[1] == 0  : self.end_cnt += 1

        if  10 <= self.end_cnt <= self.end_limit :

            self.start_cnt = 0
            self.send_static_flag = YEAH_TUNNEL_STATIC_DONE
        
        if self.end_cnt >= self.end_limit :

            self.static_mission_flag = False
    
        


class DYNAMIC :

    def __init__(self) :

        self.driver = lidar3d.driver()
        self.driver.run()

        self.velcmd      = 0
        self.brake_time  = None
        self.is_certain  = 0
        self.print_cnt   = 0
    
    
    def dynamic_obs(self, static_flag, cmd_velocity) :

        self.print_cnt += 1

        dynamic_flag  =  False
        brakecmd      =  0
        self.velcmd   =  cmd_velocity

        if static_flag == YEAH_TUNNEL_STATIC or static_flag == YEAH_TUNNEL_STATIC_DONE : 
            
            dynamic_flag = False

        else : dynamic_flag = True

        if dynamic_flag is False : brakecmd = 0

        else :

            lidar_channel = 9

            data = self.driver.get_channel_data(lidar_channel)

            lidar_data = np.copy(data)                  # do not use original data --> copy data to lidar_data

            cluster_n = self.clustering(lidar_data)

            brakecmd = self.control_platform(cluster_n, cmd_velocity)

            # plt.cla()
            # plt.plot(lidar_data[0], lidar_data[1], 'bo', markersize = 5)
            # plt.plot(cluster_x, cluster_y, 'ro', markersize = 8)
            # plt.grid(True)
            # plt.xlim(-4, 4)
            # plt.ylim(0, 5)
            # plt.pause(0.001)

        if self.print_cnt % 20 == 0 : 

            print(f"DYNAMIC 구동여부  : {dynamic_flag} \n")

        return self.velcmd, brakecmd
        
        
    
    def dbScan(self, x_data, y_data) :

        cluster_x = []
        cluster_y = []

        epsilon   = 0.25
        minSample = 3

        roi_x     = 0.75
        roi_y     = 4.0

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

            # plt.cla()
            # plt.plot(cluster_x, cluster_y, 'ro')
            # plt.xlim([0, 3])
            # plt.ylim([-1.5, 1.5])
            # plt.grid(True)
            # plt.pause(0.0001)

            return len(cluster_x)
        
        except ValueError as e : 

            cluster_x, cluster_y = [], []

            return len(cluster_x)


    def control_platform(self, cluster_n, cmd_velocity) :

        cnt_thresh   = 2
        restart_time = 1.0 
        brake_cmd    = 0

        if cluster_n >= 1 : self.is_certain += 1
            
        elif cluster_n == 0 and self.is_certain >= 0 : self.is_certain -= 1
        
        if self.is_certain >= cnt_thresh :

            self.velcmd = 0
            brake_cmd   = 200
            
            if self.brake_time is None : self.brake_time = time.time()

            if self.brake_time is not None : 

                if time.time() - self.brake_time >= restart_time : 

                    self.is_certain = 0
                    brake_cmd       = 0
                    self.velcmd     = cmd_velocity
                    self.brake_time = None
            
        return brake_cmd