from src.myfunc import  *
from src.navigation import *
from src.gps import *







# def distinguishLeftRight(rubberX,rubberY):
#     global L_cone_x, L_cone_y,R_cone_x,R_cone_y
#     idx_left = (rubberY>0)
#     idx_right = (rubberY<0)
#     LstartX = rubberX[idx_left] 
#     LstartY = rubberY[idx_left] 
#     RstartX = rubberX[idx_right] 
#     RstartY = rubberY[idx_right] 
#     # LeftConeX = [] 
#     # LeftConeY = [] 
#     # RightConeX = [] 
#     # RightConeY = []
#     if len(LstartX) > 0 and  _dist(LstartX[0] , LstartY[0],L_cone_x,L_cone_y )<2.0 :
#         L_cone_x = LstartX[0] 
#         L_cone_y = LstartY[0] 
#     else :
#         L_cone_x = L_cone_x
#         L_cone_y = L_cone_y
#     if len(RstartX) > 0 and _dist(RstartX[0] , RstartY[0],R_cone_x,R_cone_y )<2.0:
#         R_cone_x = RstartX[0] 
#         R_cone_y = RstartY[0] 
#     else :
#         R_cone_x = R_cone_x
#         R_cone_y = R_cone_y

# def get_cubic_spline(points):
#     # Extract x and y coordinates from the points
#     x_coords = [point[0] for point in points]
#     y_coords = [point[1] for point in points]
    
#     # Create a CubicSpline object
#     spline = interp1d(x_coords, y_coords ,kind = 'linear' )
    
#     return spline

# def clustering():#X array Y array of Lidar
    
#     x=[]
#     y=[]
#     clusterX = []    # cluster X좌표
#     clusterY = []    # cluster Y좌표
#     clusterDist = [] # cluster의 차로부터의 거리
#     nu = removeOutliers(distance)
#     x , y = polar2xy(nu,azim)
    
        
#     X = np.stack((x,y), axis = 1)
#     cluster = DBSCAN(eps=0.4,min_samples=7).fit(X)    
#     labels = cluster.labels_
#     cluster_num = max(labels)
#     ROI_cluster_num =0
#     for i in range(cluster_num+1):   #클러스터 갯수만큼 반복
#         points_x = x[labels == i]
#         points_y = y[labels == i]
#         X = np.stack((points_x,points_y) , axis= 0)
#         cov = np.cov(X)
#         var = variance_of_cluster(points_x,points_y)
        
#         # print(cov)
#         center_x = np.mean(points_x)
#         center_y = np.mean(points_y)
#         if math.hypot(center_x ,center_y) < 0.001:# or var > 0.005 :
#             continue
#         else:
#             clusterDist.append(math.hypot(center_x,center_y))
#             clusterX.append(center_x)
#             clusterY.append(center_y)
#             ROI_cluster_num +=1
        
#     clusterDist = np.array(clusterDist)
#     clusterX = np.array(clusterX)
#     clusterY = np.array(clusterY)
#     s = clusterDist.argsort()
#     cX = clusterX[s]
#     cY = clusterY[s]

#     return cX,cY ,ROI_cluster_num   

# def _dist(x1,y1,x2,y2): # distance of two points
#     return math.hypot(x1-x2,y1-y2)

# def dist2(p1,p2):
#     return math.hypot(p1[0]-p2[0],p1[1]-p2[1])

# def vector_angle(v1 , v2):
#     dot = np.dot(v1,v2)
#     cos_theta = dot / (np.linalg.norm(v1) * np.linalg.norm(v2)+0.000001)
#     theta = np.arccos(cos_theta)
#     return theta

# def get_start_cone(cX,cY):
#     idx_left = (cY>0)
#     idx_right = (cY<0)
#     LstartX = cX[idx_left] 
#     LstartY = cY[idx_left] 
#     RstartX = cX[idx_right] 
#     RstartY = cY[idx_right] 
#     left_start = np.hstack((LstartX[0],LstartY[0]))
#     right_start = np.hstack((RstartX[0],RstartY[0]))
#     return left_start,right_start

# def generate_path(candid_x, candid_y):

#     path            = []   # 경로 점들 저장
#     path_indices    = []                  # 경로 점들의 원본 배열 에서의 인덱스 값
#     k_dist          = 1
#     k_heading       = 1
#     next_point_idx =0
    
#     if len(candid_x) >0 :
#         candid          = np.stack((candid_x,candid_y), axis=1)
#         candid_r        = [math.sqrt(candid_x[i]**2 + candid_y[i]**2) for i in range(len(candid_x))]
#         point_start_idx = candid_r.index(min(candid_r)) # 가장 가까운 중점 => 경로의 시작점
#         # startpoint      = np.stack((candid_x[point_start_idx],candid_y[point_start_idx]), axis=0)
#         startpoint  = [(L_cone_x + R_cone_x)*0.5 ,(L_cone_y + R_cone_y)*0.5 ] 
#         # path.append(startpoint) # 시작점 저장
#         # path_indices.append(point_start_idx)
#         '''
#         cost(start , simp[i]) 했을 때 최소가 되는 i (start idx 제외)=> 저장 and pop
#         cost(next, simp[i]) 최소  = > 저장 and pop
#         한 5 6번 반복
#         '''
#         candid_indices = list(range(len(candid_x))) # 후보 점들의 인덱스들

#         # 1st iteration (시작점 -> 2번째 점)
#         e_x = [1,0]
#         min_cost = np.inf
#         for i in candid_indices:
#             if i not in path_indices and vector_angle(e_x,candid[i] - startpoint ) < 55 * np.pi / 180:
#                 costfun = k_dist * dist2(startpoint ,candid[i])   
#                 if costfun < min_cost  :
#                     next_point_idx = i
#                     min_cost = costfun
#         path_indices.append(next_point_idx) # 2번째 점 인덱스 저장

#         # 2nd iteration
#         for n in range(2):
#             min_cost = np.inf
#             v1 = candid[next_point_idx] - startpoint
#             ang =0
#             for i in candid_indices:
#                 if i not in path_indices and (vector_angle(v1,candid[i]-candid[next_point_idx]))< 70 * np.pi / 180: # 이미 
#                     costfun = k_dist * dist2(candid[next_point_idx] ,candid[i])
#                     if costfun < min_cost: #and math.fabs(vector_angle(v1,candid[i]-candid[next_point_idx])) <70 * np.pi /180:
#                         ang = i
#                         min_cost = costfun
#             next_point_idx =ang
#             path_indices.append(next_point_idx) # 2번째 점 인덱스 저장

#         for i in path_indices:
#             path.append(candid[i])

#         path = np.array(path)
#     return path

# def purePursuit(LoS,lookahead):
#     min_lookahead = 2.0 # [m]
#     WB = 1.4
#     if lookahead < min_lookahead:
#         lookahead = min_lookahead
#     delta =  - 1 * math.sin(LoS) / lookahead *180/np.pi
#     if delta > 28 :
#         delta = 28
#     elif delta < -28:
#         delta = -28
#     return delta
steer_cmd_prev = 0
if __name__ == "__main__":
    
    sim = LIDAR_SM()
    gps = GPS_SM()
    gps.sharedmemory_open()
    lat,lon,_,_ = gps.recv_data()
    vfeel = VelFilter(lat,lon)
    sim.sharedmemory_open()

    if mode == "real":
        device = 'com4'
        command  =  erpSerial(device)
    # velCMD = 4

    time_curr  = 0
    time_cnt   = 0
    time_stime = 0 
    time_ts    = 0.02
    time_final = 5000

    meas_vel = 0
    ref_vel = 6
    vc = VelCon()
    while (time_stime < time_final) :
        time_start = time.time()
        sim.main()

        cX , cY , cN = clustering()
        distinguishLeftRight(cX, cY)
        
        cX = np.hstack(([StartCone.L_cone_x,StartCone.R_cone_x],cX))
        cY = np.hstack(([StartCone.L_cone_y,StartCone.R_cone_y],cY))
        points = np.stack((cX,cY), axis=1)
        # print(L_cone_x)
        
        path_candidate_x = []     # 경로 후보 점
        path_candidate_y = []
        # print(points)
        lat,lon,_,_ = gps.recv_data()
        meas_vel = vfeel.main(lat,lon)
        print(f"속도추정:{meas_vel :.2f}")
        if cN >= 3:
            
            try:
                tri = Delaunay(points) # Delaunay 삼각분할 알고리듬
                simps = tri.simplices # 각 삼각형들 구성하는 점들의 인덱스 값 (n x 3 행렬)
                # print(len(simps))
                
                for i in range(0, len(simps)): # 경로점의 후보 점 생성(=> 들로네 삼각분할 삼각형들의 중점)
                    node1_idx = simps[i][0]
                    node2_idx = simps[i][1]
                    node3_idx = simps[i][2] 
                    # 후보 중점 계산할 때 너무 길이가 짧은 변의 중점 제외
                    if dist(cX[node1_idx] ,cY[node1_idx] ,cX[node2_idx],cY[node2_idx]) > 1.5 :
                        p1_x,p1_y = (cX[node1_idx]+cX[node2_idx])*0.5 , (cY[node1_idx]+cY[node2_idx])*0.5
                        path_candidate_x.append(p1_x)
                        path_candidate_y.append(p1_y)
                    if dist(cX[node1_idx] ,cY[node1_idx] ,cX[node3_idx],cY[node3_idx]) > 1.5 :
                        p2_x,p2_y = (cX[node1_idx]+cX[node3_idx])*0.5 , (cY[node1_idx]+cY[node3_idx])*0.5
                        path_candidate_x.append(p2_x)
                        path_candidate_y.append(p2_y)
                    if dist(cX[node2_idx] ,cY[node2_idx] ,cX[node2_idx],cY[node2_idx]) > 1.5 :
                        p3_x,p3_y = (cX[node2_idx]+cX[node3_idx])*0.5 , (cY[node2_idx]+cY[node3_idx])*0.5
                        path_candidate_x.append(p3_x)
                        path_candidate_y.append(p3_y)
                    # path_candidate_x.extend([p1_x,p2_x,p3_x])
                    # path_candidate_y.extend([p1_y,p2_y,p3_y])
                path = generate_path(path_candidate_x,path_candidate_y) # 경로 계획
   
                plt.cla()
                plt.triplot(points[:,0], points[:,1], tri.simplices)
                plt.plot( points[:,0], points[:,1], 'o')
                # plt.plot(-1 * path_candidate_y,path_candidate_x , 'yo')
                plt.plot(path[:,0],path[:,1],'k-')
                plt.xlim([-1,8])
                plt.ylim([-6,6 ])
                plt.grid()
                # plt.plot(xaray,yaray,'k-')
                plt.pause(0.001)
                
                target_point = path[1]  # 경로 점에서 4번째 점을 목표 지점으로 설정 
                los = np.arctan2(target_point[1], target_point[0]) # 목표지점 시선각
                lookahead = math.hypot(target_point[0], target_point[1]) 
                steer_cmd =  purePursuit(los,2)
                # if StartCone.L_cnt - StartCone.R_cnt > 10:
                #     steer_cmd = -28
                #     print("Left")
                # elif StartCone.L_cnt - StartCone.R_cnt < -10:
                #     steer_cmd = 28
                #     print("Right")
                # print(steer_cmd)


                

                sim.send_ctrl_cmd(steer_cmd)
                # steer_cmd = los * 180/ np.pi
                # steer_cmd_prev = steer_cmd
                # For ERP Serial
                if mode == "real":
                    vel_cmd = vc.velocity_control(ref_vel ,meas_vel)
                    command.send_ctrl_cmd(round(vel_cmd) , round(steer_cmd))

            except ValueError:
                pass
        
        
        

        # plt.plot(cX,cY,'bo',markersize = 5)

        # plt.grid()
        # plt.pause(0.0001)

        cX , cY =[], []

        time_end = time.time()
        # while time.time() - time_start < 0.05:
        #     pass
        time_del = time_end-time_start
        # fps =1 / time_del
        # print(f'FPS:{fps:.4f}')


    sim.sharedmemory_close()



