####################################
# 웨이포인트 저장 코드              #
# 2023-09-07                    #
# Author : NKH                     #
####################################

from scripts.core import *
import matplotlib.pyplot as plt
import datetime



if __name__ == "__main__":

 
    data_IO = DataCommunication(MOD_MORAI)

    lat, lon, x, y, yaw = data_IO.get_att_data()


    gps_buf =[]
    while not (isEnd()):

        # import data
        lat, lon, x, y, yaw = data_IO.get_att_data()

        gps_buf.append([ lat,lon])
        print("path making")
        
        time.sleep(0.5)


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
