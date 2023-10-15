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

# def plot(waymaker: WayMaker):
#     plt.cla()
#     # plt.plot(park_x,park_y , "bo")
#     # plt.plot(park_enter_x,park_enter_y,"ko")
#     # plt.plot(pcx,pcy,'yo')
#     plt.plot(waymaker.cx, waymaker.cy, "bo", markersize=1)
#     plt.plot(waymaker.cx[mission_master.Delivery_idx[0]:mission_master.Delivery_idx[1]],
#              waymaker.cy[mission_master.Delivery_idx[0]:mission_master.Delivery_idx[1]], "ro", markersize=3)  # 배달 구간
#     plt.plot(waymaker.cx[mission_master.Delivery_idx[2]:mission_master.Delivery_idx[3]],
#              waymaker.cy[mission_master.Delivery_idx[2]:mission_master.Delivery_idx[3]], "ro", markersize=3)  # 배달 구간

#     plt.plot(waymaker.cx[mission_master.TrafficLeft_idx[0]:mission_master.TrafficLeft_idx[1]],
#              waymaker.cy[mission_master.TrafficLeft_idx[0]:mission_master.TrafficLeft_idx[1]], "mo",
#              markersize=3)  # 좌회전
#     plt.plot(waymaker.cx[mission_master.TrafficLeft_idx[2]:mission_master.TrafficLeft_idx[3]],
#              waymaker.cy[mission_master.TrafficLeft_idx[2]:mission_master.TrafficLeft_idx[3]], "mo", markersize=3)
#     plt.plot(waymaker.cx[mission_master.TrafficLeft_idx[4]:mission_master.TrafficLeft_idx[5]],
#              waymaker.cy[mission_master.TrafficLeft_idx[4]:mission_master.TrafficLeft_idx[5]], "mo", markersize=3)

#     plt.plot(waymaker.cx[mission_master.TrafficStraight_idx[0]:mission_master.TrafficStraight_idx[1]],
#              waymaker.cy[mission_master.TrafficStraight_idx[0]:mission_master.TrafficStraight_idx[1]], "yo",
#              markersize=3)  # 직진
#     plt.plot(waymaker.cx[mission_master.TrafficStraight_idx[2]:mission_master.TrafficStraight_idx[3]],
#              waymaker.cy[mission_master.TrafficStraight_idx[2]:mission_master.TrafficStraight_idx[3]], "yo",
#              markersize=3)
#     plt.plot(waymaker.cx[mission_master.TrafficStraight_idx[4]:mission_master.TrafficStraight_idx[5]],
#              waymaker.cy[mission_master.TrafficStraight_idx[4]:mission_master.TrafficStraight_idx[5]], "yo",
#              markersize=3)
#     plt.plot(waymaker.cx[mission_master.TrafficStraight_idx[6]:mission_master.TrafficStraight_idx[7]],
#              waymaker.cy[mission_master.TrafficStraight_idx[6]:mission_master.TrafficStraight_idx[7]], "yo",
#              markersize=3)

#     plt.plot(waymaker.cx[mission_master.StaticObs_idx[0]:mission_master.StaticObs_idx[1]],
#              waymaker.cy[mission_master.StaticObs_idx[0]:mission_master.StaticObs_idx[1]], "go", markersize=3)  # 좌회전
#     plt.plot(waymaker.cx[mission_master.StaticObs_idx[2]:mission_master.StaticObs_idx[3]],
#              waymaker.cy[mission_master.StaticObs_idx[2]:mission_master.StaticObs_idx[3]], "go", markersize=3)

#     plt.plot(x, y, "ko")
#     plt.plot(tx, ty, "yo")
#     # plt.xlim((x-10 ,x+10))
#     # plt.ylim((y-10 ,y+10))
#     plt.grid()
#     plt.pause(0.0001)
#     # ----------------------------------PLOT------------------------------------#   
#     # plt.cla()    
#     # plt.grid()
#     # plt.plot(park_enter_body_x, park_enter_body_y , "bo")
#     # plt.plot(pc_x , pc_y , "yo")
#     # plt.xlim((-10 ,10))
#     # plt.ylim((-10 ,10))
#     # plt.pause(0.0001)
#     # time.sleep(0.05)

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

    Wa = WayMaker("waypoint/예선2222.csv")
    data_IO = DataCommunication(MOD_MORAI)
    cmd_gear = 0
    cmd_steer = 0
    cmd_velocity = 20
    cmd_brake = 0

    lat, lon, x, y, yaw = data_IO.get_att_data()
    target_ind, _ = Wa.Trajectory.search_target_index(x, y)
    
    # mission_master = Mission_GalSang(Wa.cx, Wa.cy) # for galsang
    # mission_master = Mission(Wa.cx, Wa.cy) # for bonsun
    # mission_master = Mission_YeahSon(Wa.cx, Wa.cy) #for yeason
    
    para = PARALLEL()
    # cam = Camera_IO()

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
        # mission_flag = mission_master.current_mission(Wa.Trajectory.my_curr_index)

        # plt.cla()
        # plt.plot(pcx,pcy, "bo")
        # plt.xlim(-1, 10)
        # plt.ylim(-10,10)
        # plt.pause(0.0001)
        # cmd_steer = vector_force_field(azim , dist,cmd_steer)

        
        # -----------------------------------------------------------#
        #                      CONTROL & COMMAND                     #
        # -----------------------------------------------------------#
        if data_IO.mode == MOD_REAL:
            vel_ctrl.vel_cmd = vel_ctrl.velocity_control(cmd_velocity,meas_speed)
            print(cmd_gear, cmd_steer, vel_ctrl.vel_cmd, cmd_brake, meas_speed)
            data_IO.command_to_vehicle(cmd_gear, cmd_steer , vel_ctrl.vel_cmd, cmd_brake)
        elif data_IO.mode == MOD_MORAI:
        # COMMAND CONTROL INPUT to ERP
            data_IO.command_to_vehicle(cmd_gear, cmd_steer , cmd_velocity, cmd_brake)

        
        # -----------------------------------------------------------#
        #                            PLOT                            #
        # -----------------------------------------------------------#

        galplot(Wa)
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
