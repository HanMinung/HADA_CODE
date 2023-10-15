from threading import Thread
from yolov8_engine import *
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

    # 카메라
    cap = camera_init(bottom=True, top=True, upper= True)

    yolo_line = YOLOV8(camera_capture=cap['bottom'],
                       cam_mode='line', save_output=True,
                       window_name='bottom')

    yolo_delivery = YOLOV8(camera_capture=cap['top'],
                           cam_mode='delivery', save_output=False,
                           window_name='top_deliv')

    yolo_traffic = YOLOV8(camera_capture=cap['upper'],
                           cam_mode='traffic', save_output=True,
                           window_name='upper_traff')
    
    # yolo_traffic_origin = YOLOV8(camera_capture=cap['upper'],
    #                        cam_mode='origin', save_output=True,
    #                        window_name='upper_traff')

    thread0 = Thread(target=yolo_line.run)
    thread1 = Thread(target=yolo_delivery.run)
    thread2 = Thread(target=yolo_traffic.run)
    # thread3 = Thread(target=yolo_traffic_origin.run)

    thread0.start()
    thread1.start()
    thread2.start()
    # thread3.start()

    print('opencv version: ' + cv.__version__)  # cv version check

    print_cnt: int = 0

    while True:

        frame = yolo_line.get_frame()
        if frame.size != 0:
            cv.imshow("line", frame)

        frame = yolo_delivery.get_frame()
        if frame.size != 0:
            cv.imshow("delivery", frame)

        frame = yolo_traffic.get_frame()
        if frame.size != 0:
            cv.imshow("traffic", frame)

        print_cnt += 1
        if print_cnt % 100 == 0:
            yolo_line.print_lane_result()
            yolo_delivery.print_delivery_result()
            yolo_traffic.print_traffic_result()

        key = cv.waitKey(1)
        if key == 27:
            yolo_line.__del__()
            yolo_delivery.__del__()
            yolo_traffic.__del__()
            # yolo_traffic_origin

            cv.destroyAllWindows()  # 모든 창 닫기
            break
        elif key == ord('s'):
            cv.waitKey()

    print('camera finsh ...')
    thread0.join()
    thread1.join()
    thread2.join()
    # thread3.join()
