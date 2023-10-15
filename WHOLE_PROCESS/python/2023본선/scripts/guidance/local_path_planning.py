import numpy as np
import time
import matplotlib.pyplot as plt
from scripts.guidance.guidance import *
from sklearn.cluster import DBSCAN
from scripts.cam_tongshin import *


# import HADA.lidar3d_py as lidar3d

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
            obstacle_vec[0] += k_obs * (20 / pc[i]) * y[i]
            
        elif pc[i] < 2 and pc[i] > 0:                                  # +- 40도 밖의 점군에 대해서 척력 계산
            obstacle_vec[0] += k_obs * (6 / pc[i]) * y[i]

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




    def __init__(self) :

        self.driver = lidar3d.driver()
        self.driver.run()

        self.brake_time = None
        self.is_certain = 0
    
    
    def dynamic_obs(self, static_mission_flag) :

        # algorithm operates only when static flag if False
        if static_mission_flag : return 0

        else :

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

        cnt_thresh   = 10
        restart_time = 1.0 
        brake_cmd    = 0

        if cluster_n >= 1 : self.is_certain += 1
            
        elif cluster_n == 0 and self.is_certain >= 0 : self.is_certain -= 1
        
        if self.is_certain >= cnt_thresh :

            brake_cmd = 200
            
            if self.brake_time is None : self.brake_time = time.time()

            if self.brake_time is not None : 

                if time.time() - self.brake_time >= restart_time : 

                    self.is_certain = 0
                    brake_cmd       = 0
                    self.brake_time = None
            
        return brake_cmd