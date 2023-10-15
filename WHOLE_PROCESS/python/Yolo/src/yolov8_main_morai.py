from threading import Thread

# from serial_node import *
from yolov8_engine import *

# from module_woong import *

from morai.lib.cam_util import UDP_CAM_Parser
import os, json

# deliv = DELIVERY()

if __name__ == '__main__':

    '''
    camera_init(원하는카메라=True, 나머지 선언안하면 default = False): 원하는 카메라의 정보를 가지고 온다. bottom, top, right, left = True or False

    인스턴스 선언 YOLOV8(camera_capture : 원하는 위에서 열었던 카메라 정보 딕셔너리 형태 ==> 만약 카메라 말고 영상을 열고 싶으면 camera_capture에 영상 경로
                        cam_mode       : delivery, line, traffic, origin 존재
                        save_output    : True or False
                        window_name    : 저장 이름에 영향을 줌) 

    thread 선언 : 인스턴스.run을 타겟으로 설정한다

    thread 시작 ==> while문 각각 계속 돌아감

    while
        main문 안에 있는 while을 통해서 imshow 및 종료 진행

        q를 입력시 인스턴스.__del__() : 인스턴스 종료 및 각 thread while 종료

        thread.join() : 진짜 스레드 종료
    '''

    # morai
    path = os.path.dirname(os.path.abspath(__file__))
    # JSON 파일이 있는 디렉토리를 작성
    params_path = os.path.join(path, 'morai', 'params.json')
    with open(params_path, 'r') as fp:
        # JSON 파일을 파이썬 객체로 로드
        params = json.load(fp)

    params = params["params"]
    user_ip = params["user_ip"]
    host_ip = params["host_ip"]
    cam_port = params["cam_dst_port"]
    cam_port2 = params["cam2_dst_port"]

    params_cam = {
        "localIP": user_ip,
        "localPort": cam_port,
        "localPort2": cam_port2,
        "hostIP": host_ip,
        "Block_SIZE": int(65000)
    }

    udp_cam_line = UDP_CAM_Parser(ip=params_cam["hostIP"], port=params_cam["localPort"], params_cam=params_cam)
    time.sleep(2)
    my_cap = morai_VideoCapture(udp_cam_line)
    # cap_morai = cv.VideoCapture(my_cap)
    yolo_line = YOLOV8(my_cap,
                          cam_mode='line', save_output=False,
                          window_name='line', morai=True)

    thread0 = Thread(target=yolo_line.run)
    # thread1 = Thread(target=yolo_rubber.run)

    thread0.start()
    # thread1.start()

    print('opencv version: ' + cv.__version__)  # cv version check

    print_cnt: int = 0

    while True:

        frame = yolo_line.get_frame()
        if frame.size != 0:
            cv.imshow("line", frame)

        print_cnt += 1
        if print_cnt % 100 == 0:
            yolo_line.print_lane_result()
            # yolo_rubber.print_rubber_result()

        key = cv.waitKey(1)
        if key == 27:
            yolo_line.__del__()
            # yolo_rubber.__del__()

            cv.destroyAllWindows()  # 모든 창 닫기
            break
        elif key == ord('s'):
            cv.waitKey()

    print('camera finsh ...')
    thread0.join()
    # thread1.join()
