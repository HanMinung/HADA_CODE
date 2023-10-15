####################################
# Main Code For Mission Driving    #
# 2023-08-23                       #
# Author : NKH                     #
####################################

from scripts.core import *
import matplotlib.pyplot as plt

import datetime
def  galplot(waymaker: WayMaker):
    plt.cla()    
    plt.grid()
    plt.plot(waymaker.cx, waymaker.cy , "ko")
    plt.plot(waymaker.cx[mission_master.Delivery_idx[0]:mission_master.Delivery_idx[1]],
             waymaker.cy[mission_master.Delivery_idx[0]:mission_master.Delivery_idx[1]], "yo", markersize=3)  # 배달 구간
    plt.plot(waymaker.cx[mission_master.Delivery_idx[2]:mission_master.Delivery_idx[3]],
             waymaker.cy[mission_master.Delivery_idx[2]:mission_master.Delivery_idx[3]], "go", markersize=3)  # 배달 구간

    plt.plot(x, y, "go" , markersize = 15)
    # plt.plot([x,x+3*math.sin(yaw*np.pi/180)] ,[ y,y+3*math.cos(yaw*np.pi/180) ] ,"g-" , linewidth = 5)
    # # plt.plot(x+3*math.sin(yaw*np.pi/180), y+3*math.cos(yaw*np.pi/180), "ko")
    plt.plot(tx, ty, "ro",markersize =5)

    plt.pause(0.0001)
if __name__ == "__main__":

    Wa = WayMaker("waypoint/갈상p턴.csv")
    data_IO = DataCommunication(MOD_REAL)
    ref_cmd_gear = 0
    ref_cmd_steer = 0
    ref_cmd_velocity = 3
    ref_cmd_brake = 0

    lat, lon, x, y, yaw = data_IO.get_att_data()
    target_ind, _ = Wa.Trajectory.search_target_index(x, y)
    
    mission_master = Mission_GalSang(Wa.cx, Wa.cy) # for galsang
    # mission_master = Mission(Wa.cx, Wa.cy) # for bonsun
    para = PARALLEL()
    deliv = DELIVERY()
    # cam = Camera_IO()
    mission_flag = 0  #
    cam_brake_flag = 0
    vel_ctrl = VelCon()
    # gps_buf =[]
    while not (isEnd()):
        time_pre = time.time()
        # import data
        lat, lon, x, y, yaw = data_IO.get_att_data()
        azim, diss = data_IO.get_lidar_data()
        # speed = data_IO.command.speed / 10
        # print(f"Lat: {lat:.6f} Lon: {lon:.6f} Yaw: {yaw:.2f}")
        
        # # calculate guidance command
        target_ind, _ = Wa.Trajectory.search_target_index(x, y)
        tx = Wa.cx[target_ind]
        ty = Wa.cy[target_ind]
        ref_cmd_steer = impact_Angle_steer_control(x, y, yaw, tx, ty)

        # print(lat, lon)

        mission_flag = mission_master.current_mission(Wa.Trajectory.my_curr_index)

        cmd_gear, cmd_velocity, cmd_steer, cmd_brake = deliv.delivery_mission(mission_flag, [ref_cmd_gear, ref_cmd_velocity, ref_cmd_steer, ref_cmd_brake])
        # cmd_brake = gorani_sibal(azim,diss)
        ref_cmd_steer = vector_force_field(azim,diss, cmd_steer)
        # print("---------------------------------------")
        # print(cmd_gear, cmd_steer, cmd_velocity, cmd_brake)
        meas_speed = data_IO.command.speed / 10
        vel_ctrl.vel_cmd = vel_ctrl.velocity_control(ref_cmd_velocity,meas_speed)

        # print(vel_ctrl.vel_cmd, cmd_velocity,meas_speed)
        data_IO.command_to_vehicle(cmd_gear, ref_cmd_steer ,vel_ctrl.vel_cmd, cmd_brake)

        # ----------------------------------PLOT------------------------------------#
        # galplot(Wa)
        # time.sleep(0.5)
        while (time.time() - time_pre) < 0.05:
            pass

    # 주행 데이터 저장
    # Export Data
    # nowdate = datetime.datetime.now()
    # exportfile = nowdate.strftime("%Y-%m-%d_%H%M") + "guidance_exp.csv" 

    # with open(exportfile , 'w', newline='') as file:
    #     # CSV writer 생성
    #     csv_writer = csv.writer(file)

    #     # 각 행을 CSV 파일에 작성
    #     for row in gps_buf:
    #         csv_writer.writerow(row)

    # 할거 : 현재 미션 인지하고 , 수행

    
    

    
