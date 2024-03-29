import numpy as np
# ----------------------------------------------------------

# ----------------------------------------------------------

# ----------------------------------------------------------

# ----------------------------------------------------------

from math import *
from sympy import sqrt
import time

from scripts.guidance.local_path_planning import *
from scripts.guidance.guidance import *
import matplotlib.pyplot as plt

class UNIT:

    def __init__(self):
        self.D2R = pi / 180
        self.R2D = 180 / pi
        self.KM2MS = 0.27777  # km/h  to  m/s

        self.LAT2METER = 110950.59672489
        self.LON2METER =  5159243.427952315 * np.pi / 180

    def RAD2DEG(self, val):
        return val * self.R2D

    def DEG2RAD(self, val):
        return val * self.D2R


class PARALLEL:
    """
        [ options require for modification in the experiment ]
        
        - parking lot stop waypoint
        - parking lot index extracting waypoint
        - start position way point
        - stop position way point
        - location threshold
        - gear command
        
    """

    def __init__(self):

        self.unit = UNIT()

        self.lidar_dist = []
        self.azimuth = []

        # lat, lon information of three parking lots
        # self.parking_lot = [[37.239486594906836, 126.77322108146119],
        #                     [37.23944825489258, 126.77319724779235],
        #                     [37.239404275285665, 126.77316920483779]]

        self.parking_lot = [[36.104234115000004  ,129.38709665833332],
                            [36.10422132149203,  129.38715155798405 ],
                            [36.104206736669944,  129.3872052416568]]

        # reference point of checking each parking lot
        # self.park_enter = [[37.239461805547805, 126.77321552077697],
        #                    [37.23942346283424, 126.7731902783902],
        #                    [37.23937835692698, 126.77316223882387]]
        
        self.park_enter = [[36.104234766666664, 129.38719452333333],
                           [36.10422763236186,  129.38717395422344],
                           [36.10421192198761,  129.38722760382336]]
        

        # referecne point of getting empty space index & get stop position
        # self.park_checkpoint = [[37.2393862467553, 126.773199775928],
        #                         [37.2393483444278, 126.773168930525],
        #                         [37.2393084420170, 126.773142338551]]
        

        self.park_checkpoint = [[36.10423160919496,  129.38720044328727],
                                [36.10422045627737,  129.38725145519857],
                                [36.104205871411196,  129.38730513884965]]
        

        self.park_enter_x, self.park_enter_y = waypoint_to_xy(self.park_enter)
        self.p_occ = 0.65  # 점유 확률 p(occ | z)
        self.log_occ_prob = [0, 0, 0]  # 주차공간 점유도 확률 (로그)
        self.occ_prob = [0, 0, 0]  # 주차공간 점유도 확률
        self.l_occ = math.log10(self.p_occ / (1 - self.p_occ))  # 점유 로그 오즈 (log odds)
        self.park_checkpoint_x, self.park_checkpoint_y = waypoint_to_xy(self.park_checkpoint)  # 주차구역 감지했을떄 멈출 포인트

        self.empty_idx = 0

        self.is_mission_start = False
        self.is_ready_planning = False
        self.is_ready_parking = False
        self.is_total_stop = False
        self.is_parking_finish = False
        self.is_waiting_finish = False
        self.is_escape_finish = False

        self.S = 0
        self.H = 0

        self.delta = 0
        # self.loc_thresh = 2.5
        self.loc_thresh = 4.3


        self.park_cnt = 0
        self.print_cnt = 0

        self.x_list = None
        self.y_list = None
        self.Guididx = 3
        self.alphaOffset= 0.8

        self.waiting_start_time = None


    def parallel_parking(self, x, y, latitude, longtitude, yaw, azimuth, lidar_data, org_cmd):

        gear, vel, steer, brake = 0, 0, 0, 0

        self.check_parallel_start(latitude, longtitude)  # check mission start with waypoint infomrmation

        if self.is_mission_start :

            self.azimuth, self.lidar_dist = np.copy(azimuth), np.copy(lidar_data)  # lidar data receive

            self.check_parking_space(x, y, latitude, longtitude, yaw, lidar_data, azimuth)

            self.make_trajectory(longtitude, latitude)  # parking trajectory generation

        if self.is_ready_parking and not self.is_parking_finish:

            self.Rimpact_Angle_steer_control(longtitude, latitude, yaw)  # reverse gear guidance

        elif self.is_parking_finish:

            self.impact_Angle_steer_control(longtitude, latitude, yaw)

        gear, vel, steer, brake = self.control_platform(org_cmd)  # contorl gear, vel, steer, brake

        self.print_result()  # print algorithm state

        return gear, vel, steer, brake
    

    def control_platform(self, org_cmd):

        gearcmd, velcmd, steercmd, brakecmd = 0, 0, 0, 0

        if not self.is_mission_start and not self.is_ready_planning :

            gearcmd, velcmd, steercmd, brakecmd = org_cmd[0], org_cmd[1], org_cmd[2], org_cmd[3]

        if self.is_mission_start and not self.is_ready_planning:

            # gearcmd, velcmd, steercmd, brakecmd = org_cmd[0], 4, org_cmd[2] * self.unit.R2D, org_cmd[3]
            
            # real
            gearcmd, velcmd, steercmd, brakecmd = org_cmd[0], 7, org_cmd[2], org_cmd[3]

        if self.is_mission_start and self.is_ready_parking and not self.is_total_stop:

            if self.waiting_start_time is not None:

                if time.time() - self.waiting_start_time >= 1.0: self.is_total_stop = True

            gearcmd, velcmd, steercmd, brakecmd = 1, 0, 0, 200

        if self.is_ready_parking and self.is_total_stop and self.is_total_stop:

            # gearcmd, velcmd, steercmd, brakecmd = 2, 4, self.unit.R2D * self.delta, 0

            gearcmd, velcmd, steercmd, brakecmd = 2, 4,  self.delta, 0

        if self.is_parking_finish and not self.is_ready_parking and not self.is_escape_finish:

            gearcmd, velcmd, steercmd, brakecmd = 1, 0, 0, 200

            if time.time() - self.waiting_start_time >= 3.0:  
            
                self.is_waiting_finish = True 

            if self.is_waiting_finish:  # after waiting 3 sec : escape parking space

                # gearcmd, velcmd, steercmd, brakecmd = 4, 4, self.unit.R2D * self.delta, 0

                gearcmd, velcmd, steercmd, brakecmd = 0, 6, self.delta, 0


        if self.is_escape_finish == True:  # after all process of parking is finished

            # gearcmd, velcmd, steercmd, brakecmd = org_cmd[0], org_cmd[1], org_cmd[2] * self.unit.R2D, org_cmd[3]
            gearcmd, velcmd, steercmd, brakecmd = org_cmd[0], org_cmd[1], org_cmd[2] , org_cmd[3]

        return gearcmd, velcmd, steercmd, brakecmd

    def check_parallel_start(self, latitude, longtitude):

        # real
        # mission_start_pos = [37.23953631117929, 126.77330404809946] 

        mission_start_pos = [36.10427039140663, 129.3870628346891] 

        lat_lon_diff = np.hypot((latitude - mission_start_pos[0]) * self.unit.LAT2METER,
                                (longtitude - mission_start_pos[1]) * self.unit.LON2METER)

        if lat_lon_diff <= self.loc_thresh:  self.is_mission_start = True

    def check_parking_space(self, px, py, lat, lon, yaw, pc, azim):

        if not self.is_ready_planning and not self.is_ready_parking:

            park_enter_body_x = []
            park_enter_body_y = []
            obs_hit = 0
            pc = removeOutliers(pc)
            pc_x, pc_y = polar2xy(pc, azim)
            theta = (90 - yaw) * np.pi / 180

            for Idx in range(3):
                rotated_enter_x, rotated_enter_y = affine_transform(theta, self.park_enter_x[Idx],
                                                                    self.park_enter_y[Idx], px + 1.6 * cos(theta), py+ 1.6 * sin(theta))

                park_enter_body_x.append(rotated_enter_x)
                park_enter_body_y.append(rotated_enter_y)

            for Idx in range(3):

                dx = [park_enter_body_x[Idx] - icx for icx in pc_x]
                dy = [park_enter_body_y[Idx] - icy for icy in pc_y]

                d = np.hypot(dx, dy)  # 각 주차구역 진입점에서 라이다 측정값 까지 거리
                for Icx in range(761):
                    if d[Icx] < 1:
                        obs_hit += 1

                # 점유 판단 기준: 주차구역 입구 중점으로부터 1미터 반경에 물체가 감지되면 점유
                self.log_occ_prob[Idx] += obs_hit * self.l_occ  # 주차구역이 점유되어있다고 판단되면 점군의 개수만큼 로그 오즈를 더함
                if math.fabs(math.atan2(park_enter_body_y[Idx], park_enter_body_x[
                    Idx])) < 95 * np.pi / 180 and obs_hit < 2:  # and np.hypot(px - self.park_x[i] ,py - self.park_y[i] ) <10:
                    self.log_occ_prob[Idx] -= self.l_occ
                self.log_occ_prob[Idx] = min(self.log_occ_prob[Idx], 10)
                self.occ_prob[Idx] = 1 - 1 / (1 + math.exp(self.log_occ_prob[Idx]))  # 확률 복구 ( -inf ~ inf -> 0 ~ 1)

                obs_hit = 0

            # 각 주차구역마다 체크포인트 지정
            # 체크포인트에서 주차구역 판단했을떄 비어있으면 멈춤
            # park fix 플래그 켬 -> 주차 진행
            dx_park = [px - icx for icx in self.park_checkpoint_x]
            dy_park = [py - icy for icy in self.park_checkpoint_y]
            d_park = np.hypot(dx_park, dy_park)  # 내 위치와 주차구역 체크 포인트 사이 거리

            for Idx in range(3):

                abs_loc_diff = abs(np.hypot((lat - self.park_checkpoint[Idx][0]) * LAT2METER,
                                            (lon - self.park_checkpoint[Idx][1]) * LON2METER))

                if abs_loc_diff < (self.loc_thresh - 0.3) and self.occ_prob[Idx] < 0.1:
                    self.empty_idx = Idx + 1
                    self.is_ready_planning = True

            # print(abs(np.hypot((lat - self.park_checkpoint[0][0]) * LAT2METER,
            #                                 (lon - self.park_checkpoint[0][1]) * LON2METER)))
            # plt.cla()
            # plt.plot(pc_x,pc_y, "bo")
            # plt.plot(park_enter_body_x , park_enter_body_y , "ro", markersize = 10)
            # plt.xlim(-2,10)
            # plt.ylim(-10,10)
            # plt.grid()
            # plt.pause(0.001)
    """
        path planning
    """

    def bisection_method(self, a, b, tolerance, max_iterations):

        if (a - sqrt((self.S / 2) ** 2 + (self.H / 2 - (self.H - a)) ** 2)) * (
                b - sqrt((self.S / 2) ** 2 + (self.H / 2 - (self.H - b)) ** 2)) >= 0:
            raise ValueError(
                "Function values at a and b should have opposite signs, ensuring a unique root in the interval.")
            c = 3

        for _ in range(max_iterations):

            c = (a + b) / 2

            if abs((c - sqrt((self.S / 2) ** 2 + (self.H / 2 - (self.H - c)) ** 2))) < tolerance:
                return c

            if (a - sqrt((self.S / 2) ** 2 + (self.H / 2 - (self.H - a)) ** 2)) * (
                    c - sqrt((self.S / 2) ** 2 + (self.H / 2 - (self.H - c)) ** 2)) < 0:

                b = c

            else:
                a = c

        return c

    def make_trajectory(self, longtitude, latitude):

        self.gear = 1  # parking gear and path planning
        #parking_heading = -2.258591636
        parking_heading = 108*np.pi/180
        if self.is_ready_planning:

            midpoint_x = (longtitude + self.parking_lot[self.empty_idx - 1][1]) / 2.0
            midpoint_y = (latitude + self.parking_lot[self.empty_idx - 1][0]) / 2.0

            lat_diff = (latitude - self.parking_lot[self.empty_idx - 1][0]) * self.unit.LAT2METER
            lon_diff = (longtitude - self.parking_lot[self.empty_idx - 1][1]) * self.unit.LON2METER

            angleP2E = atan2(lon_diff,lat_diff)  # azimuth in Radian

            L = sqrt((lat_diff) ** 2 + (lon_diff) ** 2)
            self.S = abs(L * cos(parking_heading - angleP2E))
            self.H = abs(L * sin(parking_heading - angleP2E))
            # self.H = abs(L * cos(parking_heading - angleP2E))
            # self.S = abs(L * sin(parking_heading - angleP2E))

            Rmin = abs(self.bisection_method(-15, 15, 0.00001, 100))
            ParaNormal = parking_heading - pi / 2
            if ParaNormal >= np.pi:
                ParaNormal -= 2*np.pi
            elif ParaNormal <= -np.pi:
                ParaNormal += 2*np.pi

            O1 = np.zeros((2, 1))
            O2 = np.zeros((2, 1))

            O1[0][0] = latitude   - Rmin * cos(ParaNormal) / self.unit.LAT2METER
            O1[1][0] = longtitude - Rmin * sin(ParaNormal) / self.unit.LON2METER
            O2[0][0] = self.parking_lot[self.empty_idx - 1][0] + Rmin * cos(ParaNormal) / self.unit.LAT2METER
            O2[1][0] = self.parking_lot[self.empty_idx - 1][1] + Rmin * sin(ParaNormal) / self.unit.LON2METER

            O1_2_Mid = atan2(midpoint_y - O1[0][0], midpoint_x - O1[1][0])
            alpha = acos((self.S ** 2 - self.H ** 2) / (self.S ** 2 + self.H ** 2))
             
            O2_2_Mid = atan2(midpoint_x - O2[1][0],midpoint_y - O2[0][0])
            O2_2_Stop= atan2(self.parking_lot[self.empty_idx - 1][1] - O2[1][0],self.parking_lot[self.empty_idx - 1][0]- O2[0][0])
            if O2_2_Mid >= np.pi:
                O2_2_Mid -=2*np.pi
            elif O2_2_Mid <= -np.pi:
                O2_2_Mid += 2*np.pi
            if O2_2_Stop >= np.pi:
                O2_2_Stop -= 2*np.pi
            elif O2_2_Stop <= -np.pi:
                O2_2_Stop += 2*np.pi

            theta_1 = np.linspace(O1_2_Mid - alpha*self.alphaOffset, O1_2_Mid, 4)
            theta_2 = np.linspace(O2_2_Mid,  O2_2_Mid-alpha*(2.2-self.alphaOffset), 4)

            X_O1 = np.zeros((4, 1))
            Y_O1 = np.zeros((4, 1))
            X_O2 = np.zeros((4, 1))
            Y_O2 = np.zeros((4, 1))

            for Idx in range(4):
                X_O1[Idx][0] = O1[1][0] + Rmin * cos(theta_1[Idx]) / self.unit.LON2METER
                Y_O1[Idx][0] = O1[0][0] + Rmin * sin(theta_1[Idx]) / self.unit.LAT2METER
                X_O2[Idx][0] = O2[1][0] - Rmin * cos(theta_2[Idx]) / self.unit.LON2METER 
                Y_O2[Idx][0] = O2[0][0] - Rmin * sin(theta_2[Idx]) / self.unit.LAT2METER

            X1 = X_O1.astype('float64')
            X2 = X_O2.astype('float64')
            Y1 = Y_O1.astype('float64')
            Y2 = Y_O2.astype('float64')

            x1_list = X1.flatten().tolist()
            x2_list = X2.flatten().tolist()
            y1_list = Y1.flatten().tolist()
            y2_list = Y2.flatten().tolist()

            self.x_list = x1_list + x2_list
            self.y_list = y1_list + y2_list
            # print(f"|   - latitude     : {latitude}")
            # print(f"|   - longtidude   : {longtitude}")
            # print(f"|   - midpointX     : {midpoint_x}")
            # print(f"|   - midpointY     : {midpoint_y}")
            # print(f" O1 {O1}")
            # print(f" O2 {O2}")        
            # print(f"|   - S     : {self.S}")
            # print(f"|   - H     : {self.H}")
            # print(f"|   - Rmin  : {Rmin}")
            # print(f"|   - alpha  : {alpha}")
            # print(self.x_list)
            # print(self.y_list)

            self.is_ready_planning = False
            self.is_ready_parking = True

            self.waiting_start_time = time.time()

    def Rimpact_Angle_steer_control(self, lon, lat, yaw):
        #Real
        #parking_heading = -2.258591636
        parking_heading = 108*np.pi/180
        vel = 5
        gamma = (- yaw) * np.pi / 180 - np.pi/2

        dY = (self.y_list[self.Guididx] - lat) * self.unit.LAT2METER
        dX = (self.x_list[self.Guididx] - lon) * self.unit.LON2METER

        los = atan2(dY, dX)
        eta = gamma - los
        Lf = hypot(dX, dY)

        gamdotc = -3 * vel * sin(eta) / Lf
        self.delta = (gamdotc * 1.4 / vel * 180/np.pi)
        self.delta = min(max(self.delta, -28), 28)

        if Lf < 1.0 and (self.Guididx + 1) < len(self.x_list):
            self.Guididx = self.Guididx + 1
            if self.Guididx == 3:
                self.Guididx += 1

        # if self.Guididx == 7: #self.Guididx + 1 <= len(self.x_list) and self.Guididx + 1 > len(self.x_list) - 2:
        #     #if abs(yaw*np.pi/180 - parking_heading) % (2 * pi) <0.5:     #0.32:
        #         # print("존잘진")
        #     self.park_cnt += 1
        #     self.velcmd = 0
        #     self.brakecmd = 200   
        #     self.Guididx = self.Guididx

        if 5<=self.Guididx:
            if abs(yaw*np.pi/180 - parking_heading) % (2 * pi) <0.14:     #0.32:
                # print("존잘진")
                self.park_cnt += 1
                self.velcmd = 0
                self.brakecmd = 200   
                self.Guididx = self.Guididx            
        print(self.Guididx)
        if self.park_cnt >= 30:
            self.Guididx -= 3

            self.is_total_stop = False
            self.is_parking_finish = True
            self.is_ready_parking = False

            self.waiting_start_time = time.time()
        # traj = np.column_stack([self.y_list,self.x_list])
        # xl,yl = waypoint_to_xy(traj)
        # plt.cla()
        # plt.plot(xl, yl, 'ko', markersize = 3)
        # plt.plot(xl[self.Guididx]  , yl[self.Guididx] , 'yo', markersize = 7)
        # plt.plot((lon - INIT_LON) * self.unit.LON2METER ,(lat - INIT_LAT) * self.unit.LAT2METER , "mo")
        # plt.xlim((lon - INIT_LON) * self.unit.LON2METER - 10 , (lon - INIT_LON) * self.unit.LON2METER +10)
        # plt.ylim((lat - INIT_LAT) * self.unit.LAT2METER -10, (lat - INIT_LAT) * self.unit.LAT2METER+10)
        # plt.grid()
        
        # plt.pause(0.0001)

    def impact_Angle_steer_control(self, lon, lat, yaw):

        vel = 5
        gamma = (90 - yaw) * np.pi / 180 #(- yaw) * np.pi / 180 - np.pi/2

        dY = (self.y_list[self.Guididx] - lat) * self.unit.LAT2METER
        dX = (self.x_list[self.Guididx] - lon) * self.unit.LON2METER
        los = atan2(dY,dX)
        eta = gamma - los
        Lf = hypot(dY,dX)

        gamdotc = 3 * vel * sin(eta) / Lf
        self.delta = gamdotc * 1.4 / vel * 180 /np.pi
        self.delta = min(max(self.delta, -28), 28)

        if Lf < 0.8 and (self.Guididx) >= 0: 
            self.Guididx -= 1
            if self.Guididx == 4:
                self.Guididx -= 1
        
        if self.Guididx == 2 :
            self.is_parking_finish = False
            self.is_ready_parking = False
            self.is_escape_finish = True
        
        traj = np.column_stack([self.y_list,self.x_list])
        xl,yl = waypoint_to_xy(traj)
        # plt.cla()
        # plt.plot(xl, yl, 'ko', markersize = 3)
        # plt.plot(xl[self.Guididx]  , yl[self.Guididx] , 'yo', markersize = 7)
        # plt.plot((lon - INIT_LON) * self.unit.LON2METER ,(lat - INIT_LAT) * self.unit.LAT2METER , "mo")
        # plt.xlim((lon - INIT_LON) * self.unit.LON2METER - 10 , (lon - INIT_LON) * self.unit.LON2METER +10)
        # plt.ylim((lat - INIT_LAT) * self.unit.LAT2METER -10, (lat - INIT_LAT) * self.unit.LAT2METER+10)
        # plt.grid()
        
        # plt.pause(0.0001)

    def print_result(self):

        self.print_cnt += 1

        if self.print_cnt % 100 == 0:
            print("\n----------------------------------------------------")
            print(f"|   - empty space idx     : {self.empty_idx}")
            print(f"|   - mission start flag  : {self.is_mission_start}")
            print(f"|   - ---                 : {self.occ_prob}")
            print(f"|   - ready parking flag  : {self.is_ready_parking}")
            print(f"|   - total stop flag     : {self.is_total_stop}")
            print(f"|   - parking finish flag : {self.is_parking_finish}")
            print(f"|   - waiting state       : {self.is_waiting_finish}")
