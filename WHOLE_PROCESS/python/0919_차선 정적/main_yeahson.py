####################################
# Main Code For Mission Driving    #
# 2023-08-23                       #
# Author : NKH                     #
####################################
'''
        if Uturn
        
        1. 러버콘을 충돌하지 않으면서 톨게이트를 통과해야 함
        2. 러버콘을 회피할 때 차선 지역 이탈하면 안됨
        3. 공사장 표지판이 있음( 인식할 지는 선택)
'''
        
       
'''
        if Tunnel
        
        1. 카메라 -> 메인 : 조향각, 속도
        2. yaw 제어: 정적 장애물 회피로 인해 차선을 벗어났을 때 다시 복귀하기 위함
        3. 정적 장애물 회피  ( 2D Lidar ) -> vff
        4. 동적 장애물 급정거( 3D Lidar ) -> 일정 범위 들어왔을 때 트리거 (플래그)
        
'''

from scripts.core import *
import matplotlib.pyplot as plt
import datetime


def  galplot(waymaker: WayMaker):
    plt.cla()    
    plt.grid()
    plt.plot(waymaker.cx, waymaker.cy , "ko")
    # plt.plot(waymaker.cx[mission_master.Delivery_idx[0]:mission_master.Delivery_idx[1]],
    #          waymaker.cy[mission_master.Delivery_idx[0]:mission_master.Delivery_idx[1]], "yo", markersize=3)  # 배달 구간
    # plt.plot(waymaker.cx[mission_master.Delivery_idx[2]:mission_master.Delivery_idx[3]],
    #          waymaker.cy[mission_master.Delivery_idx[2]:mission_master.Delivery_idx[3]], "go", markersize=3)  # 배달 구간

    plt.plot(x, y, "go" , markersize = 15)
    # plt.plot([x,x+3*math.sin(yaw*np.pi/180)] ,[ y,y+3*math.cos(yaw*np.pi/180) ] ,"g-" , linewidth = 5)
    # # plt.plot(x+3*math.sin(yaw*np.pi/180), y+3*math.cos(yaw*np.pi/180), "ko")
    plt.plot(tx, ty, "yo")

    plt.pause(0.0001)

if __name__ == "__main__":

    Wa = WayMaker("2023-09-16_0007guidance_exp.csv")
    data_IO = DataCommunication(MOD_REAL)
    cmd_gear = 0
    cmd_steer = 0
    cmd_velocity = 10
    cmd_brake = 0

    lat, lon, x, y, yaw = data_IO.get_att_data()
    target_ind, _ = Wa.Trajectory.search_target_index(x, y)
    
    # mission_master = Mission_GalSang(Wa.cx, Wa.cy) # for galsang
    # mission_master = Mission(Wa.cx, Wa.cy) # for bonsun
    mission_master = Mission_YeahSon(Wa.cx, Wa.cy) #for yeason
    uturn = UTURN()
    static = STATIC()
    # dynamic = DYNAMIC()

    cam = Camera_IO()

    vel_ctrl = VelCon()
    mission_flag = 0  #
    cam_brake_flag = 0


    while not (isEnd()):
        time_pre = time.time()
        # -----------------------------------------------------------#
        #                         IMPORT DATA                        #
        # -----------------------------------------------------------#
        lat, lon, x, y, yaw = data_IO.get_att_data()
        if data_IO.mode == MOD_REAL: meas_speed = data_IO.command.speed / 10
        # print(f"Lat: {lat:.6f} Lon: {lon:.6f} Yaw: {yaw:.2f}")
        azim, dis = data_IO.get_lidar_data()
        # dis = removeOutliers(dis)



        # -----------------------------------------------------------#
        #                           GUIDANCE                         #
        # -----------------------------------------------------------#
        target_ind, _ = Wa.Trajectory.search_target_index(x, y)
        tx = Wa.cx[target_ind]
        ty = Wa.cy[target_ind]
        cmd_steer = impact_Angle_steer_control(x, y, yaw, tx, ty)
        
        
        
        # -----------------------------------------------------------#
        #                           MISSION                          #
        # -----------------------------------------------------------#
        mission_flag = mission_master.current_mission(Wa.Trajectory.my_curr_index)

        # cmd_steer = vector_force_field(azim , dist,cmd_steer)
        if mission_flag == YEAH_TUNNEL:
        
            # 정적 장애물 추가 (9.16)
            # cmd_steer, cmd_velocity = static.static_obs(azim, dis, yaw, [cmd_steer, cmd_velocity])
            static.static_obs(azim, dis, yaw, [cmd_steer, cmd_velocity])

            cmd_steer, cmd_velocity = cam.recv_lane_cmd()
            
            # 동적 장애물 추가 (9.16)
            # cmd_brake = dynamic.dynamic_obs()

        elif mission_flag ==YEAH_UTURN:
            cmd_gear, cmd_velocity, cmd_steer, cmd_brake = uturn.uturn(azim,dis,x,y,Wa,yaw,[cmd_gear, cmd_velocity, cmd_steer, cmd_brake])
        
        # -----------------------------------------------------------#
        #                      CONTROL & COMMAND                     #
        # -----------------------------------------------------------#
        if data_IO.mode == MOD_REAL:
            vel_ctrl.vel_cmd = vel_ctrl.velocity_control(cmd_velocity,meas_speed)
            # print(cmd_gear, cmd_steer, vel_ctrl.vel_cmd, cmd_brake, meas_speed)
            data_IO.command_to_vehicle(cmd_gear, cmd_steer , vel_ctrl.vel_cmd, cmd_brake)

            # data_IO.command_to_vehicle(cmd_gear, cmd_steer , cmd_velocity, cmd_brake)

        elif data_IO.mode == MOD_MORAI:
        # COMMAND CONTROL INPUT to ERP
            data_IO.command_to_vehicle(cmd_gear, cmd_steer , cmd_velocity, cmd_brake)

        
        # -----------------------------------------------------------#
        #                            PLOT                            #
        # -----------------------------------------------------------#

        # galplot(Wa)
        # time.sleep(0.5)

        # -----------------------------------------------------------#
        #                        Time Idling                         #
        # -----------------------------------------------------------#
        while (time.time() - time_pre) < 0.05:
            pass
        
        
        
        
    # # 주행 데이터 저장
    # # Export Data
    # nowdate = datetime.datetime.now()
    # exportfile = nowdate.strftime("%Y-%m-%d_%H%M") + "guidance_exp.csv" 

    # with open(exportfile , 'w', newline='') as file:
    #     # CSV writer 생성
    #     csv_writer = csv.writer(file)

    #     # 각 행을 CSV 파일에 작성
    #     for row in gps_buf:
    #         csv_writer.writerow(row)

    # # 할거 : 현재 미션 인지하고 , 수행
