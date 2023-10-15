####################################
# Main Code For Mission Driving    #
# 2023-09-09                       #
# Author : NKH                     #
####################################
import subprocess
import atexit
from scripts.core import *
import matplotlib.pyplot as plt
import datetime
import time
import matplotlib as mpl
import matplotlib.style as mplstyle
mpl.use('TkAgg')
mpl.rcParams['path.simplify'] = True
mpl.rcParams['path.simplify_threshold'] = 0.0
mplstyle.use('fast')

# batch_file_path = "C:/Users/HADA/Desktop/all_sensor.bat"  # 배치 파일 경로를 지정하세요
# process = subprocess.Popen(batch_file_path, shell=True)
# process.wait()

# def cleanup():
#     if process.poll() is None:  # 프로세스가 아직 실행 중인 경우
#         process.terminate()     # 프로세스 종료
   
def bonplot(waymaker: WayMaker):
    plt.cla()
    # plt.plot(park_x,park_y , "bo")
    # plt.plot(park_enter_x,park_enter_y,"ko")
    # plt.plot(pcx,pcy,'yo')
    plt.plot(waymaker.cx, waymaker.cy, "bo", markersize=1)
    plt.plot(waymaker.cx[mission_master.Delivery_idx[0]:mission_master.Delivery_idx[1]],
             waymaker.cy[mission_master.Delivery_idx[0]:mission_master.Delivery_idx[1]], "ro", markersize=3)  # 배달 구간
    plt.plot(waymaker.cx[mission_master.Delivery_idx[2]:mission_master.Delivery_idx[3]],
             waymaker.cy[mission_master.Delivery_idx[2]:mission_master.Delivery_idx[3]], "ro", markersize=3)  # 배달 구간

    # plt.plot(waymaker.cx[mission_master.TrafficLeft_idx[0]:mission_master.TrafficLeft_idx[1]],
    #          waymaker.cy[mission_master.TrafficLeft_idx[0]:mission_master.TrafficLeft_idx[1]], "mo",
    #          markersize=3)  # 좌회전
    # plt.plot(waymaker.cx[mission_master.TrafficLeft_idx[2]:mission_master.TrafficLeft_idx[3]],
    #          waymaker.cy[mission_master.TrafficLeft_idx[2]:mission_master.TrafficLeft_idx[3]], "mo", markersize=3)
    # plt.plot(waymaker.cx[mission_master.TrafficLeft_idx[4]:mission_master.TrafficLeft_idx[5]],
    #          waymaker.cy[mission_master.TrafficLeft_idx[4]:mission_master.TrafficLeft_idx[5]], "mo", markersize=3)

    # plt.plot(waymaker.cx[mission_master.TrafficStraight_idx[0]:mission_master.TrafficStraight_idx[1]],
    #          waymaker.cy[mission_master.TrafficStraight_idx[0]:mission_master.TrafficStraight_idx[1]], "yo",
    #          markersize=3)  # 직진
    # plt.plot(waymaker.cx[mission_master.TrafficStraight_idx[2]:mission_master.TrafficStraight_idx[3]],
    #          waymaker.cy[mission_master.TrafficStraight_idx[2]:mission_master.TrafficStraight_idx[3]], "yo",
    #          markersize=3)
    # plt.plot(waymaker.cx[mission_master.TrafficStraight_idx[4]:mission_master.TrafficStraight_idx[5]],
    #          waymaker.cy[mission_master.TrafficStraight_idx[4]:mission_master.TrafficStraight_idx[5]], "yo",
    #          markersize=3)
    # plt.plot(waymaker.cx[mission_master.TrafficStraight_idx[6]:mission_master.TrafficStraight_idx[7]],
    #          waymaker.cy[mission_master.TrafficStraight_idx[6]:mission_master.TrafficStraight_idx[7]], "yo",
    #          markersize=3)
    plt.plot(waymaker.cx[mission_master.Park_idx[0]:mission_master.Park_idx[1]],
             waymaker.cy[mission_master.Park_idx[0]:mission_master.Park_idx[1]], "ko",
             markersize=3)

    plt.plot(waymaker.cx[mission_master.StaticObs_idx[0]:mission_master.StaticObs_idx[1]],
             waymaker.cy[mission_master.StaticObs_idx[0]:mission_master.StaticObs_idx[1]], "go", markersize=3)  # 좌회전
    # plt.plot(waymaker.cx[mission_master.StaticObs_idx[2]:mission_master.StaticObs_idx[3]],
    #          waymaker.cy[mission_master.StaticObs_idx[2]:mission_master.StaticObs_idx[3]], "go", markersize=3)

    plt.plot(x, y, "ko")
    plt.plot(tx, ty, "yo")
    plt.xlim((x-10 ,x+10))
    plt.ylim((y-10 ,y+10))
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


if __name__ == "__main__":

    Wa = WayMaker("waypoint/1002_HGU본선시험.csv")
    data_IO = DataCommunication(MOD_REAL)
    ref_cmd_gear = 0
    ref_cmd_steer = 0
    ref_cmd_velocity = 6
    ref_cmd_brake = 0
    cmd_gear = 0
    cmd_steer = 0
    cmd_velocity = 8
    cmd_brake = 0
    
    lat, lon, x, y, yaw = data_IO.get_att_data()
    target_ind, _ = Wa.Trajectory.search_target_index(x, y)
    
    mission_master = Mission_Gal_Bon(Wa.cx, Wa.cy) # for bonsun
    para = PARALLEL()
    # cam = Camera_IO()

    vel_ctrl = VelCon()
    mission_flag = 0  #
    cam_brake_flag = 0
    
    deliv = DELIVERY()
    rturn = JungZee_Gal()
    
    yu = YawUpdate(x,y)
    velfil = VelFilter(lat,lon)
    yaw_bias = 0


    
    gps_buf =[]
    while not (isEnd()):
        time_pre = time.time()
        # -----------------------------------------------------------#
        #                         IMPORT DATA                        #
        # -----------------------------------------------------------#
        lat, lon, x, y, yaw = data_IO.get_att_data()
        if data_IO.mode == MOD_REAL: meas_speed = velfil.main(lat,lon)
        # print(f"Lat: {lat:.6f} Lon: {lon:.6f} Yaw: {yaw:.2f}")
        azim, dist = data_IO.get_lidar_data()

        gps_buf.append([lat , lon])
        # YAW UPDATE(1002 추가)    
        velfil.yoyo(yaw)
        sibal =  normalize_angle(velfil.bias)
        if math.fabs(velfil.residual) < 0.005 and sibal < 80:
            yaw_bias = velfil.bias
            print(sibal)

        yaw = yaw + yaw_bias
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
        # cam.send_flag(mission_flag)
        # cam_brake_flag = cam.recv_flag()
        # mission_flag = MISSION_NONE
        if mission_flag == MISSION_DELIVERY_A:
            cmd_gear, cmd_velocity, cmd_steer, cmd_brake = deliv.delivery_mission(mission_flag, 0.5, [ref_cmd_gear, ref_cmd_velocity, cmd_steer, ref_cmd_brake])
            cmd_velocity = 6
            Wa.Trajectory.set_Lf(2.4)
        elif mission_flag == MISSION_DELIVERY_B:
            cmd_gear, cmd_velocity, cmd_steer, cmd_brake = deliv.delivery_mission(mission_flag, 0.5,[ref_cmd_gear, ref_cmd_velocity, cmd_steer, ref_cmd_brake])
            cmd_velocity = 6
            Wa.Trajectory.set_Lf(2.4)
        # elif mission_flag == MISSION_STATIC:
        #     cmd_velocity = 6
        #     Wa.Trajectory.set_Lf(2.4)
        #     cmd_steer = vector_force_field(azim , dist, cmd_steer)
        elif mission_flag == MISSION_PARKING:
            Wa.Trajectory.set_Lf(2.4)
            cmd_gear, cmd_velocity, cmd_steer, cmd_brake = para.parallel_parking(x, y, lat, lon, yaw, azim, dist ,[cmd_gear, cmd_velocity, cmd_steer, cmd_brake])
       
        # elif mission_flag == MISSION_TURNRIGHT:
        #     cmd_velocity , cmd_brake = rturn.right_stop(x,y,ref_cmd_velocity,ref_cmd_brake)
        elif mission_flag == MISSION_NONE :
            cmd_gear = 0
            cmd_velocity = 7
            cmd_brake = 0
            Wa.Trajectory.set_Lf(3)

            # cmd_velocity , cmd_brake = rturn.right_stop(x,y,cmd_velocity,cmd_brake)

        else:
            cmd_gear = 0
            cmd_velocity = 6
            cmd_brake = 0
            # cmd_velocity , cmd_brake = rturn.right_stop(x,y,cmd_velocity,cmd_brake)

        # parallel_parking
        



        # -----------------------------------------------------------#
        #                      CONTROL & COMMAND                     #
        # -----------------------------------------------------------#
        if data_IO.mode == MOD_REAL:
            vel_ctrl.vel_cmd = vel_ctrl.velocity_control(cmd_velocity,meas_speed)
            # print(cmd_gear, cmd_steer, vel_ctrl.vel_cmd, cmd_brake, meas_speed)
            
            data_IO.command_to_vehicle(cmd_gear, cmd_steer , vel_ctrl.vel_cmd, cmd_brake)
        elif data_IO.mode == MOD_MORAI:
        # COMMAND CONTROL INPUT to ERP
            data_IO.command_to_vehicle(cmd_gear, cmd_steer , cmd_velocity, cmd_brake)



        # -----------------------------------------------------------#
        #                            PLOT                            #
        # -----------------------------------------------------------#
        # bonplot(Wa)

        # -----------------------------------------------------------#
        #                        Time Idling                         #
        # -----------------------------------------------------------#
        while (time.time() - time_pre) < 0.05:
            pass
        
        # deltime = time.time() - time_pre
        # print(1/deltime)
        
        
    # 주행 데이터 저장
    # Export Data
    nowdate = datetime.datetime.now()
    exportfile = nowdate.strftime("%Y-%m-%d_%H%M") + "guidance_exp.csv" 

    # with open(exportfile , 'w', newline='') as file:
    #     # CSV writer 생성
    #     csv_writer = csv.writer(file)

    #     # 각 행을 CSV 파일에 작성
    #     for row in gps_buf:
    #         csv_writer.writerow(row)

    # atexit.register(cleanup)