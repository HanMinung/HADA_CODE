from project_val import *
import csv
from scripts.guidance.cubic_spline_planner import *

# Parameters
k = 0.1  # look forward gain
Lfc = 2.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 2.9  # [m] wheel base of vehicle
filename = "waypoint/WP_TRACK0.csv"

LAT2METER = 110950.59672489
# LON2METER = 90048.170449268  # at K city


LON2METER = 5159243.427952315 * np.pi / 180  # at HGU


def read_csv_file(file_path):
    data = []
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file, delimiter=',')
        for row in csv_reader:
            data.append([float(row[0]), float(row[1])])
    return data


def waypoint_to_xy(data):  # EN to XY
    # INIT_LON = 126.7
    # INIT_LAT = 37.2
    try:
        wp_x = [(row[1] - INIT_LON) * LON2METER for row in data]
        wp_y = [(row[0] - INIT_LAT) * LAT2METER for row in data]
    except TypeError:
        wp_x = (data[1] - INIT_LON) * LON2METER
        wp_y = (data[0] - INIT_LAT) * LAT2METER
    return wp_x, wp_y


def my_xy_pos(_lat: float, _lon: float, _yaw: float) -> tuple[float, float, float]:
    """
    위도 경도값 기반으로 m 좌표계로 변환
    """
    yaw: float = _yaw
    x: float = (_lon - INIT_LON) * LON2METER
    y: float = (_lat - INIT_LAT) * LAT2METER

    return x, y, yaw


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None
        self.my_curr_index = 0

    def search_target_index(self, px, py):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [px - icx for icx in self.cx]
            dy = [py - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            dx = [px - icx for icx in self.cx]
            dy = [py - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            # if math.fabs(ind - self.old_nearest_point_index) > 30:
            #     self.old_nearest_point_index = self.old_nearest_point_index
            #     # self.old_nearest_point_index +=1 
            # else:
            #     self.old_nearest_point_index = ind
                
            # ind = self.old_nearest_point_index
            # distance_this_index = np.hypot(self.cx[ind] - px,self.cy[ind] -py)

            # while (ind + 1) < len(self.cx):

            #     distance_next_index = np.hypot(self.cx[ind + 1] - px, self.cy[ind + 1] - py)
            #     if distance_this_index < distance_next_index:
            #         break
            #     ind = ind + 1 if (ind + 1) < len(self.cx) else ind
            # #     distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        self.my_curr_index = self.old_nearest_point_index
        # Lf = k * state.v + Lfc  # update look ah
        
        # lead distance
        Lf = 4.5
        
        # search look ahead target point index
        while Lf > np.hypot(self.cx[self.old_nearest_point_index] - px, self.cy[self.old_nearest_point_index] - py):
            if (self.old_nearest_point_index + 1) >= len(self.cx):
                break  # not exceed goal
            self.old_nearest_point_index += 1

        # print(self.old_nearest_point_index)
        # self.old_nearest_point_index = ind
        return self.old_nearest_point_index, Lf


class RealTime():
    def __init__(self):
        self.time_cnt = 0
        self.simtime = 0
        self.Ts = 0.01
        self.time_curr = 0
        self.time_prev = 0


def pure_pursuit_steer_control(px, py, yaw, trajectory, pind):
    ind, Lf = trajectory.search_target_index(px, py)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1
    gamma = (90 - yaw) * np.pi / 180
    los = math.atan2(ty - py, tx - px)
    # alpha = -(math.atan2(ty - py, tx - px) - yaw * np.pi /180)
    eta = gamma - los
    Kp = 3
    # delta = 2 * WB * math.sin(alpha) / Lf
    delta = -2 * Kp * math.sin(eta) / Lf  # *180/ np.pi
    delta = min(max(delta, -28), 28)
    return delta, ind, los, eta


def impact_Angle_steer_control(px, py, yaw, tx, ty):
    # ind, Lf = trajectory.search_target_index(px,py)
    # if pind >= ind:
    #     ind = pind

    # if ind < len(trajectory.cx):
    #     tx = trajectory.cx[ind]
    #     ty = trajectory.cy[ind]
    # else:  # toward goal
    #     tx = trajectory.cx[-1]
    #     ty = trajectory.cy[-1]
    #     ind = len(trajectory.cx) - 1
    Lf = 3
    gamma =  (90 - yaw) * np.pi / 180
    los = math.atan2(ty - py, tx - px)
    # alpha = -(math.atan2(ty - py, tx - px) - yaw * np.pi /180)
    #print(gamma*180/np.pi)
    eta = gamma - los
    c1 = 2 * 1.1 * 0.095
    c2 = 0.095 ** 2
    vel = 8
    tgo = vel / Lf
    Kp = 1
    
    gamdotc = c1 * c2 * math.sin(eta) + (c1 + 1.0) * vel * math.sin(eta) / Lf
    # lamdot = vel * math.sin( eta ) / Lf
    # gamdotc = -(c1 * lamdot + c2 * (los - cyaw[pind]  )) / tgo + 2*lamdot
    # delta = gamdotc * 1.4 / vel * 180 / np.pi
    delta = 2 * Kp * math.sin(eta) / Lf *180/ np.pi 

    delta = min(max(delta, -28), 28)

    return delta


def stanley_control(px, py, yaw, cyaw, trajectory, pind):
    ind, Lf = trajectory.search_target_index(px, py)
    cx, cy = trajectory.cx, trajectory.cy
    # if pind >= ind:
    #     ind = pind

    # if ind < len(trajectory.cx):
    #     tx = trajectory.cx[ind]
    #     ty = trajectory.cy[ind]
    # else:  # toward goal
    #     tx = trajectory.cx[-1]
    #     ty = trajectory.cy[-1]
    #     ind = len(trajectory.cx) - 1

    dx = [px - icx for icx in cx]
    dy = [py - icy for icy in cy]
    d = np.hypot(dx, dy)
    ind = np.argmin(d)
    K = 0.1
    vel = 20
    gamma = yaw * np.pi / 180
    los = math.atan2(cy[ind] - py, cx[ind] - px)
    # alpha = -(math.atan2(ty - py, tx - px) - yaw * np.pi /180)
    eta = gamma - los

    psi = yaw - cyaw[ind] * 180 / np.pi
    cross_track_err = np.hypot(cx[ind] - px, cy[ind] - py) * math.sin(eta)
    delta = psi * 0.5 + math.atan2(K * cross_track_err, vel)
    # delta = cross_track_err * K
    delta = min(max(delta, -28), 28)
    # print(cross_track_err)
    return delta, ind


class VelCon:
    def __init__(self):
        self.Ts = 0.05 #sample period
        self.meas_vel = 0
        self.Kp = 2
        self.Ki = 0.1
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
# if __name__ == '__main__':
#     mode = "morai"
#     rt = RealTime()
#     data = read_csv_file(filename)  # read wp data
#     #=====================================================================REAL==========================================================================
#     if  mode == "real": 
#         device = 'com4'
#         command  =  erpSerial(device)
#         GPS_REAL = GPS_SM()
#         GPS_REAL.sharedmemory_open()
#         IMU_REAL = IMU_SM()
#         IMU_REAL.sharedmemory_open()

#         wp_x , wp_y = waypoint_to_xy(data) # convert wp data to xy coordinate
#         cx, cy, cyaw, ck, s = calc_spline_course(wp_x, wp_y, ds=0.1) # calculate continuous path
#         target_course = TargetCourse(cx,cy)

#         #Real GPS,IMU 초기값 받기
#         init_heading = -121 # 초기헤딩
#         lat,lon,h,GPS_qual=GPS_REAL.recv_data()
#         yaw = (IMU_REAL.recv_data()  + init_heading)            #초기만 앵글,pi/2는 함수안에서 더해줌 
#         x,y,yaw = my_xy_pos(lat,lon,yaw,data[0][0],data[0][1])
#         target_ind, _ = target_course.search_target_index(x,y)

#     #=====================================================================MORAI==========================================================================    
#     elif mode == "morai" :   
#     #MORAI
#         sim = planner()
#         wp_x , wp_y = waypoint_to_xy(data) # convert wp data to xy coordinate
#         cx, cy, cyaw, ck, s = calc_spline_course(wp_x, wp_y, ds=0.1) # calculate continuous path
#         target_course = TargetCourse(cx,cy)
#         lat,lon,yaw = sim.recv_data_MORAI()
#         x,y,yaw = my_xy_pos(lat,lon,yaw,data[0][0],data[0][1])
#         target_ind, _ = target_course.search_target_index(x,y)


#     exportdata = []

#     while True :
#         rt.time_prev = time.time()

#         lat,lon,yaw = sim.recv_data_MORAI()
#                 #Real GPS,IMU 초기값 받기
#         # init_heading = -121 # 초기헤딩
#         # lat,lon,h,GPS_qual=GPS_REAL.recv_data()
#         # target_ind, _ = target_course.search_target_index(x,y)
#         # yaw = (IMU_REAL.recv_data()  + init_heading) 

#         x,y,yaw = my_xy_pos(lat,lon,yaw,data[0][0],data[0][1])
#         # delta, target_ind ,los,eta = pure_pursuit_steer_control(x,y,yaw, target_course, target_ind)
#         delta, target_ind ,los,eta = impact_Angle_steer_control(x,y,yaw,cyaw, target_course, target_ind)
#         # delta, target_ind = stanley_control(x,y,yaw,cyaw, target_course, target_ind)
#         tx = cx[target_ind]
#         ty = cy[target_ind]
#         velcmd = 20
#         vel_curvature = velocity_control(target_ind,ck,velcmd)

#         print(delta) 
#         if mode == "morai":
#             sim.cmd_to_MORAI(2,4,velcmd ,delta )
#         elif mode =="real":
#             command.send_ctrl_cmd(velcmd , round(delta) , 0)
#         plt.cla()
#         # plt.plot(wp_x,wp_y,'k*')
#         plt.plot(cx,cy,'b-')
#         plt.plot(x,y,'yo',markersize =8)
#         plt.plot(tx,ty,'kx')
#         # # plt.xlim((x-5,x+5))
#         # # plt.ylim((y-5,y+5))
#         plt.grid()
#         plt.pause(0.0001)


#         rt.time_cnt +=1
#         rt.simtime = rt.time_cnt * rt.Ts
#         exportdata.append([delta, los , yaw, eta,rt.simtime , x,y,tx,ty] )

#         if keyboard.is_pressed('q') :# or target_ind == (len(cx) -1):
#             break
#         while (time.time() - rt.time_prev )< rt.Ts:
#             continue


#     #Export Data
#     nowdate = datetime.datetime.now()
#     exportfile = nowdate.strftime("%Y-%m-%d_%H%M") + "guidance_exp.csv" 


#     with open(exportfile , 'w', newline='') as file:
#         # CSV writer 생성
#         csv_writer = csv.writer(file)

#         # 각 행을 CSV 파일에 작성
#         for row in exportdata:
#             csv_writer.writerow(row)
