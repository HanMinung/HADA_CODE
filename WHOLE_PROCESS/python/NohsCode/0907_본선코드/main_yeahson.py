####################################
# Main Code For Mission Driving    #
# 2023-08-23                       #
# Author : NKH                     #
####################################

import datetime

from scripts.core import *


def plot(waymaker: WayMaker):
    plt.cla()
    # plt.plot(park_x,park_y , "bo")
    # plt.plot(park_enter_x,park_enter_y,"ko")
    # plt.plot(pcx,pcy,'yo')
    plt.plot(waymaker.cx, waymaker.cy, "bo", markersize = 1)
    plt.plot(waymaker.cx[mission_master.Delivery_idx[0]:mission_master.Delivery_idx[1]],
             waymaker.cy[mission_master.Delivery_idx[0]:mission_master.Delivery_idx[1]], "ro", markersize = 3)  # 배달 구간
    plt.plot(waymaker.cx[mission_master.Delivery_idx[2]:mission_master.Delivery_idx[3]],
             waymaker.cy[mission_master.Delivery_idx[2]:mission_master.Delivery_idx[3]], "ro", markersize = 3)  # 배달 구간
    
    plt.plot(waymaker.cx[mission_master.TrafficLeft_idx[0]:mission_master.TrafficLeft_idx[1]],
             waymaker.cy[mission_master.TrafficLeft_idx[0]:mission_master.TrafficLeft_idx[1]], "mo",
             markersize = 3)  # 좌회전
    plt.plot(waymaker.cx[mission_master.TrafficLeft_idx[2]:mission_master.TrafficLeft_idx[3]],
             waymaker.cy[mission_master.TrafficLeft_idx[2]:mission_master.TrafficLeft_idx[3]], "mo", markersize = 3)
    plt.plot(waymaker.cx[mission_master.TrafficLeft_idx[4]:mission_master.TrafficLeft_idx[5]],
             waymaker.cy[mission_master.TrafficLeft_idx[4]:mission_master.TrafficLeft_idx[5]], "mo", markersize = 3)
    
    plt.plot(waymaker.cx[mission_master.TrafficStraight_idx[0]:mission_master.TrafficStraight_idx[1]],
             waymaker.cy[mission_master.TrafficStraight_idx[0]:mission_master.TrafficStraight_idx[1]], "yo",
             markersize = 3)  # 직진
    plt.plot(waymaker.cx[mission_master.TrafficStraight_idx[2]:mission_master.TrafficStraight_idx[3]],
             waymaker.cy[mission_master.TrafficStraight_idx[2]:mission_master.TrafficStraight_idx[3]], "yo",
             markersize = 3)
    plt.plot(waymaker.cx[mission_master.TrafficStraight_idx[4]:mission_master.TrafficStraight_idx[5]],
             waymaker.cy[mission_master.TrafficStraight_idx[4]:mission_master.TrafficStraight_idx[5]], "yo",
             markersize = 3)
    plt.plot(waymaker.cx[mission_master.TrafficStraight_idx[6]:mission_master.TrafficStraight_idx[7]],
             waymaker.cy[mission_master.TrafficStraight_idx[6]:mission_master.TrafficStraight_idx[7]], "yo",
             markersize = 3)
    
    plt.plot(waymaker.cx[mission_master.StaticObs_idx[0]:mission_master.StaticObs_idx[1]],
             waymaker.cy[mission_master.StaticObs_idx[0]:mission_master.StaticObs_idx[1]], "go", markersize = 3)  # 좌회전
    plt.plot(waymaker.cx[mission_master.StaticObs_idx[2]:mission_master.StaticObs_idx[3]],
             waymaker.cy[mission_master.StaticObs_idx[2]:mission_master.StaticObs_idx[3]], "go", markersize = 3)
    
    plt.plot(x, y, "ko")
    plt.plot(tx, ty, "yo")
    # plt.xlim((x-10 ,x+10))
    # plt.ylim((y-10 ,y+10))
    plt.grid()
    plt.pause(0.0001)
    # ----------------------------------PLOT------------------------------------#   
    # plt.cla()    
    # plt.grid()
    # plt.plot(park_enter_body_x, park_enter_body_y , "bo")
    # plt.plot(pc_x , pc_y , "yo")
    # plt.xlim((-10 ,10))
    # plt.ylim((-10 ,10))
    # plt.pause(0.0001)
    # time.sleep(0.05)


def galplot(waymaker: WayMaker):
    plt.cla()
    plt.grid()
    plt.plot(waymaker.cx, waymaker.cy, "ko")
    plt.plot(waymaker.cx[mission_master.Delivery_idx[0]:mission_master.Delivery_idx[1]],
             waymaker.cy[mission_master.Delivery_idx[0]:mission_master.Delivery_idx[1]], "yo", markersize = 3)  # 배달 구간
    plt.plot(waymaker.cx[mission_master.Delivery_idx[2]:mission_master.Delivery_idx[3]],
             waymaker.cy[mission_master.Delivery_idx[2]:mission_master.Delivery_idx[3]], "go", markersize = 3)  # 배달 구간
    
    plt.plot(x, y, "go", markersize = 15)
    # plt.plot([x,x+3*math.sin(yaw*np.pi/180)] ,[ y,y+3*math.cos(yaw*np.pi/180) ] ,"g-" , linewidth = 5)
    # # plt.plot(x+3*math.sin(yaw*np.pi/180), y+3*math.cos(yaw*np.pi/180), "ko")
    plt.plot(tx, ty, "yo")
    
    plt.pause(0.0001)


if __name__ == "__main__":
    
    Wa = WayMaker("waypoint/갈상p턴.csv")
    data_IO = DataCommunication(MOD_REAL)
    cmd_gear = 0
    cmd_steer = 0
    cmd_velocity = 4
    cmd_brake = 0
    
    lat, lon, x, y, yaw = data_IO.get_att_data()
    target_ind, _ = Wa.Trajectory.search_target_index(x, y)
    
    mission_master = Mission_GalSang(Wa.cx, Wa.cy)  # for galsang
    # mission_master = Mission(Wa.cx, Wa.cy) # for bonsun
    para = PARALLEL()
    # cam = Camera_IO()
    
    vel_ctrl = VelCon()
    mission_flag: int = MISSION_NONE
    cam_brake_flag: int
    
    gps_buf = []
    while not (isEnd()):
        time_pre = time.time()
        # import data
        lat, lon, x, y, yaw = data_IO.get_att_data()
        meas_speed = data_IO.command.speed / 10
        # print(f"Lat: {lat:.6f} Lon: {lon:.6f} Yaw: {yaw:.2f}")
        # azim, dist = data_IO.get_lidar_data()
        
        # calculate guidance command
        target_ind, _ = Wa.Trajectory.search_target_index(x, y)
        tx = Wa.cx[target_ind]
        ty = Wa.cy[target_ind]
        cmd_steer = impact_Angle_steer_control(x, y, yaw, tx, ty)
        
        mission_flag: int = mission_master.current_mission(Wa.Trajectory.my_curr_index)
        
        if mission_flag == YEAH_UTURN:
            pass
        elif mission_flag == YEAH_TUNNEL:
            pass
        
        # cam.send_flag(mission_flag)
        # cam_brake_flag = cam.recv_flag()
        
        # if 정적장애물
        # cmd_steer = vector_force_field(azim , dist,cmd_steer)
        
        # VELOCITY CONTROL
        vel_ctrl.vel_cmd = vel_ctrl.velocity_control(cmd_velocity, meas_speed)
        print(cmd_gear, cmd_steer, vel_ctrl.vel_cmd, cmd_brake, meas_speed)
        
        # COMMAND CONTROL INPUT to ERP
        data_IO.command_to_vehicle(cmd_gear, cmd_steer, vel_ctrl.vel_cmd, cmd_brake)
        
        # ----------------------------------PLOT------------------------------------#
        
        # galplot(Wa)
        # time.sleep(0.5)
        
        while (time.time() - time_pre) < 0.05:
            pass
    # 주행 데이터 저장
    # Export Data
    nowdate = datetime.datetime.now()
    exportfile = nowdate.strftime("%Y-%m-%d_%H%M") + "guidance_exp.csv"
    
    with open(exportfile, 'w', newline = '') as file:
        # CSV writer 생성
        csv_writer = csv.writer(file)
        
        # 각 행을 CSV 파일에 작성
        for row in gps_buf:
            csv_writer.writerow(row)
    
    # 할거 : 현재 미션 인지하고 , 수행
