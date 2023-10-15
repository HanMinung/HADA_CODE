####################################
# Choi Zhong Yeah Son              #
# 2023-09-22                       #
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
import subprocess
import atexit
from scripts.core import *
import matplotlib.pyplot as plt
import datetime
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
    

def yeahplot(waymaker: WayMaker):
    plt.cla()
    plt.grid()
    plt.plot(waymaker.cx, waymaker.cy, "ko")
    plt.plot(waymaker.cx[mission_master.Uturn_idx[0]:mission_master.Uturn_idx[1]],
             waymaker.cy[mission_master.Uturn_idx[0]:mission_master.Uturn_idx[1]], "yo", markersize=3)  # 유턴

    # plt.plot(x, y, "go", markersize=3)
    # plt.plot([x,x+3*math.sin(yaw*np.pi/180)] ,[ y,y+3*math.cos(yaw*np.pi/180) ] ,"g-" , linewidth = 5)
    # # plt.plot(x+3*math.sin(yaw*np.pi/180), y+3*math.cos(yaw*np.pi/180), "ko")
    # plt.plot(tx, ty, "yo")

    plt.pause(0.0001)


if __name__ == "__main__":

    Wa = WayMaker("최종예선.csv")
    data_IO = DataCommunication(MOD_MORAI)
    cmd_gear = 0
    cmd_steer = 0
    cmd_velocity = 12
    cmd_brake = 0

    lat, lon, x, y, yaw = data_IO.get_att_data()
    target_ind, _ = Wa.Trajectory.search_target_index(x, y)

    mission_master = Mission_YeahSon(Wa.cx, Wa.cy)  # for yeason

    #
    uturn = UTURN_2()
    static = STATIC()
    # dynamic = DYNAMIC()

    cam = Camera_IO()
    vel_ctrl = VelCon()
    velfil = VelFilter(lat, lon)
    yaw_bias = 0

    while not (isEnd()):
        time_pre = time.time()
        # -----------------------------------------------------------#
        #                         IMPORT DATA                        #
        # -----------------------------------------------------------#
        lat, lon, x, y, yaw = data_IO.get_att_data()
        # print(f"Lat: {lat:.6f} Lon: {lon:.6f} Yaw: {yaw:.2f}")
        azim, dis = data_IO.get_lidar_data()

        # 요 추정 필터 (1008)
        velfil.yoyo(yaw)
        meas_speed = velfil.main(lat, lon)
        sibal =  normalize_angle(velfil.bias)
        if math.fabs(velfil.residual) < 0.005 and sibal < 50 and meas_speed > 4:
            yaw_bias = velfil.bias
            print(sibal)
            
        yaw = yaw + yaw_bias

        print(f"속도추정:{meas_speed :.2f}")

        # -----------------------------------------------------------#
        #                           GUIDANCE                         #
        # -----------------------------------------------------------#
        target_ind, _ = Wa.Trajectory.search_target_index(x, y)
        tx = Wa.cx[target_ind]
        ty = Wa.cy[target_ind]
        cmd_steer = impact_Angle_steer_control(x, y, yaw, tx, ty)
        # print(cmd_steer)

        # -----------------------------------------------------------#
        #                           MISSION                          #
        # -----------------------------------------------------------#
        mission_flag = mission_master.current_mission(Wa.Trajectory.my_curr_index)
        # mission_flag = YEAH_TUNNEL

        # if mission_flag == YEAH_TUNNEL :#or int(data_IO.gpsqual) < 4:

        #     static_end, static_flag = static.static_obs(dis)

        #     # print(static_end)

        #     cmd_steer, cmd_velocity = cam.recv_lane_cmd()

        #     # cmd_velocity, cmd_brake = dynamic.dynamic_obs(static_flag, cmd_velocity)

        if mission_flag == YEAH_UTURN:

            cmd_gear, cmd_velocity, cmd_steer, cmd_brake = uturn.uturn(azim, dis, x, y, Wa, yaw,
                                                                       [cmd_gear, cmd_velocity, cmd_steer, cmd_brake])
            cmd_velocity = 7
        else:
            cmd_velocity = 12
            
        Lf = cmd_velocity / 3
        Wa.Trajectory.set_Lf(Lf)
        # -----------------------------------------------------------#
        #                      CONTROL & COMMAND                     #
        # -----------------------------------------------------------#
        if data_IO.mode == MOD_REAL:

            vel_ctrl.vel_cmd = vel_ctrl.velocity_control(cmd_velocity, meas_speed)

            if mission_flag ==YEAH_UTURN or mission_flag == MISSION_NONE:
            # print(cmd_gear, cmd_steer, vel_ctrl.vel_cmd, cmd_brake, meas_speed)
                data_IO.command_to_vehicle(cmd_gear, cmd_steer , vel_ctrl.vel_cmd, cmd_brake)
            else:
                data_IO.command_to_vehicle(cmd_gear, cmd_steer , cmd_velocity, cmd_brake)

        elif data_IO.mode == MOD_MORAI:
            # COMMAND CONTROL INPUT to ERP
            data_IO.command_to_vehicle(cmd_gear, cmd_steer, cmd_velocity, cmd_brake)

        # -----------------------------------------------------------#
        #                            PLOT                            #
        # -----------------------------------------------------------#

        # yeahplot(Wa)
        # time.sleep(0.5)

        # -----------------------------------------------------------#
        #                        Time Idling                         #
        # -----------------------------------------------------------#
        # deltime = time.time() - time_pre
        # print(1/deltime)

        while (time.time() - time_pre) < 0.05:
            pass

