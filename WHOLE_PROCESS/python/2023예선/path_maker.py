####################################
# 웨이포인트 저장 코드              #
# 2023-09-07                    #
# Author : NKH                     #
####################################

from scripts.core import *
import matplotlib.pyplot as plt
import datetime



if __name__ == "__main__":

 
    data_IO = DataCommunication(MOD_REAL)

    lat, lon, x, y, yaw = data_IO.get_att_data()
    
    prev_x,prev_y = x,y

    gps_buf =[]
    cn=0
    while not (isEnd()):

        # import data
        lat, lon, x, y, yaw = data_IO.get_att_data()
        gps_prev = [lat,lon]
        if np.hypot(x-prev_x , y-prev_y) > 1:
            gps_buf.append([ lat,lon])
            # estim_yaw = 90 - np.arctan2(y - prev_y , x - prev_x) * 180 /np.pi
            prev_x,prev_y = x,y
            cn+=1
            print("path making",cn)
            # print(f"estim: {estim_yaw} mea_yaw: {yaw}")
        time.sleep(0.05)


    # 주행 데이터 저장
    # Export Data
    nowdate = datetime.datetime.now()
    exportfile = nowdate.strftime("%Y-%m-%d_%H%M") + "guidance_exp.csv" 

    with open(exportfile , 'w', newline='') as file:
        # CSV writer 생성
        csv_writer = csv.writer(file)

        # 각 행을 CSV 파일에 작성
        for row in gps_buf:
            csv_writer.writerow(row)

    # 할거 : 현재 미션 인지하고 , 수행
