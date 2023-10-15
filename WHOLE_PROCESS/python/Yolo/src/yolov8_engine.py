import time
from datetime import datetime
# ---------------------------------------------------------------------------------------------
import sys

# project val 경로
absolute_path_to_folder = 'C:\\Users\\HADA\Desktop\\0922_파이널'
sys.path.append(absolute_path_to_folder)
# ---------------------------------------------------------------------------------------------

from project_val import *
from ultralytics import YOLO

from yolov8_global import *


class YOLOV8:
    # =============== 클래스 맴버 선언 =============== #

    __weight_list: dict[str, str] = {'rubber': 'runs\\segment\\rubber\\weights\\best.engine',
                                     'line': 'runs\\segment\\bev_line\\weights\\best.engine',
                                     'delivery': 'runs\\segment\\delivery\\weights\\best.engine',
                                     'traffic': 'runs\\segment\\traffic_light\\weights\\best.engine',
                                     }

    __user_font = cv.FONT_HERSHEY_COMPLEX

    # __window_names: dict[str, str] = {'left': '8&2c2f8ec&0',
    #                                   'right': '9&93b8a67&0',
    #                                   'top': '9&3872e35&0',
    #                                   'bottom': '9&1ea96576&0'}

    # cv imshow 변수
    __line_width = 2
    __font_thick = 1

    undist = UNDISTORT()
    wide = CALIBRATION_WIDE_LENS()
    ipm = IPM()
    ipm_point = IPM_POINT()

    y_sections: list[int, int, int] = [37, 143, 252]  # 5m 4m 3m 에 해당하는 y 픽셀 좌표
    y_stop: int = 361  # 2m 구분선이고 이 선에 정지선이 오면 멈춘다 --> 신호등이랑 같이

    # 각 구간에 대한 +- 100 픽셀 범위를 생성합니다.
    y_interval: int = 50
    y_ranges: list[(int, int), (int, int), (int, int), (int, int)] = [(y - 50, y + 50) for y in
                                                                      y_sections]
    origin_point: tuple[int, int] = (320, 480)

    stop_line_flag: bool = False
    traffic_stop: bool = False  # 신호등에 의해 멈추는 flag 인스턴스끼리 공용으로 존재해야한다.

    rubber_in_tunnel: int = 0

    rub_bev_l_point: list = []
    rub_bev_r_point: list = []
    rub_bev_flag: str = ''

    mission_state_flag: int = MISSION_NONE

    def __init__(self, camera_capture: cv.VideoCapture, cam_mode: str, window_name: str, save_output: bool = False,
                 morai: bool = False):
        self.__cap: cv.VideoCapture = camera_capture
        self.__cam_mode: str = cam_mode
        self.__window_name: str = window_name
        self.__save_output: bool = save_output
        self.__morai: bool = morai

        self.traffic_give_sign: int = MISSION_NONE

        self.__frame_out: np.ndarray = np.array([])

        self.deliv_flag: int  # 자동차가 실제로 멈췄을 때 1
        self.flag: bool = False
        self.threadStop: bool = False

        if cam_mode != 'origin':
            self.__weight_path: str = YOLOV8.__weight_list[cam_mode]
            # class 이름을 yaml파일에서 불러온다
            label_path: str = label_list[cam_mode]
            self.__label_cls_name: list[str] = get_clsname(label_path)

        # TODO 배달 관련 flag 일단 클래스 변수로 할당함
        # 나중에 메인 코드에서 전체 flag 관련 변수가 존재하면 대체해야한다

        self.__deliv_a = True
        self.__deliv_b = False

        # self.width = int(self.__cap.get(cv.CAP_PROP_FRAME_WIDTH))
        # self.height = int(self.__cap.get(cv.CAP_PROP_FRAME_HEIGHT))
        # self.fps = int(self.__cap.get(cv.CAP_PROP_FPS))
        self.width = 640
        self.height = 480
        self.fps = 60
        self.target: str = ' '

        '''
        lane detection용 변수
        '''
        self.total_gamma: float = 0.0
        self.out_gamma: int = 0

        self.vel_before_static: int = 9
        self.vel_static_ing: int = 6
        self.vel_after_static: int = 13

        self.vel_cmd: int = 0
        self.brake_cmd: int = 0

        self.shm_stop_flag: int = CAM2MAIN_NONE
        self.current_sign_info: int = 0
        self.cls_name: str = ''

        self.rubber_cnt: int = 0
        self.rubber_center: int = 0
        self.max_area: int = 0
        self.max_ratio: float = 0.0

        self.lidar_steer: float = 0.0
        self.total_gamma: int = 0
        self.static_done_cnt: int = 0

    def run(self):

        '''
        카메라 및 YOLO 모델 초기화 후 YOLO 객체 인식 모델 시작
        '''

        # print("cex", MISSION_PARKING)

        cap: cv.VideoCapture = self.__cap

        if self.__cam_mode != 'origin':
            model: YOLO = YOLO(self.__weight_path)

        frame: np.ndarray

        if self.__save_output:
            fourcc = cv.VideoWriter_fourcc(*'mp4v')
            out_filename: str = datetime.today().strftime(
                "output/output_%Y_%m%d_%H%M_" + self.__cam_mode + '_' + self.__window_name + ".mp4")  # save results to project/name
            frame_size = (self.width, self.height)
            out = cv.VideoWriter(out_filename, fourcc, int(self.fps), frame_size)

            # camera mode가 배달일 때
            match self.__cam_mode:
                case 'delivery':
                    self._deliv_mission(cap, model, out)
                # 카메라 모드가 rubber일 때
                case 'rubber':
                    self._rubber_mission(cap, model, out)

                case 'traffic':
                    self._traffic_mission(cap, model, out)

                case 'line':
                    self._lane_mission(cap, model, out)

                case _:
                    while cap.isOpened() and not self.threadStop:  # 종료 조건 체크
                        ret, frame = cap.read()
                        # frame = YOLOV8.ipm.ipm_transform(frame) # bev로 변환이 필요한 bottom 카메라 사용
                        # 밖에 나가야할 것 저장
                        self.__frame_out = frame
                        out.write(self.__frame_out)

                        if not ret:
                            raise Exception("Cannot read frame")

        else:
            # camera mode가 배달일 때
            match self.__cam_mode:
                case 'delivery':
                    self._deliv_mission(cap, model)
                # 카메라 모드가 rubber일 때
                case 'rubber':
                    self._rubber_mission(cap, model)

                case 'traffic':
                    self._traffic_mission(cap, model)

                case 'line':
                    self._lane_mission(cap, model)

                case _:
                    while cap.isOpened() and not self.threadStop:  # 종료 조건 체크
                        ret, frame = cap.read()

                        # 밖에 나가야할 것 저장
                        self.__frame_out = frame

                        if not ret:
                            raise Exception("Cannot read frame")

    def get_frame(self):
        return self.__frame_out

    def get_flag(self):
        return self.flag

    def get_gamma(self):
        return self.total_gamma

    def get_delivery_center(self):
        return self.center_delivery

    def __del__(self):
        self.threadStop = True

    def print_delivery_result(self):
        print("-------------------DELIVERY-------------------")
        # print(f"DELIVERY TARGET         : {self.target}")
        print(f"DELIVERY FLAG           : {self.deliv_flag}")
        print(f"CENTER POINT            : {self.center_delivery}")

    def _deliv_mission(self, cap: cv.VideoCapture, model: YOLO, out: cv.VideoWriter = None):

        sm_name_send: str = CAM_TO_DELIVERY
        sm_name_recv: str = DELIVERY_TO_CAM

        shm_send_size: int = 4 * 2  # 4 * 3 * 3   (int  int) * 1
        shm_send = open_send_shm(sm_name_send, shm_send_size)
        # print("sm data type: ", type(shm_send))

        frame: np.ndarray
        undistorted_frame: np.ndarray
        ret: bool
        fps: float = 0

        # 배달 변수
        a_frame: int = 0
        b_frame: int = 0  # B frame count
        sm_cls = 0  # shared memory cls
        sm_cls_destin = None

        # yolo 변수
        label_cls_idx: int = 0
        xywh: tuple[int, int, int, int]
        xyxy: tuple[int, int, int, int]

        sm_data = []

        blue_squares: list[tuple[int, int, int, int]] = []
        a_signs: list[tuple[int, int, int, int]] = []
        b_signs: list[tuple[int, int, int, int]] = []
        destinations: list[tuple[int, int, int, int]] = []
        find = ''

        check_type: str = 'A'
        check_object: list[str, 2]
        check_sign: bool

        frame_count: list[int] = [0]

        destination_idx: int = 0
        self.center_delivery: list[int, int] = []

        cnt_dlivery_num: list[int, int, int] = [0, 0, 0]

        while cap.isOpened() and not self.threadStop:
            start_time = time.time()
            prev_time = start_time

            # Read a frame from the video
            ret, frame = cap.read()

            if not ret:
                print("Camera is Disconnected ...!")
                break

            # 공유 메모리에서 정지 flag 받아오기:
            self.deliv_flag = recv_data(sm_name_recv, 1)

            if self.deliv_flag == 0:  # sm data에서 아무 정보가 넘어오지 않는 것

                check_type = 'None'
                self.center_delivery = []
                send_data(shm_send, (2, 1), self.center_delivery)
                # continue

            elif self.deliv_flag == DELIV_A_START:
                check_type = 'A'

            elif self.deliv_flag == DELIV_A_FINISH:
                check_type = 'None'
                self.center_delivery = []
                send_data(shm_send, (2, 1), self.center_delivery)
                # continue

            elif self.deliv_flag == DELIV_B_START:
                check_type = 'B'

            elif self.deliv_flag == DELIV_B_FINISH:
                check_type = 'None'
                self.center_delivery = []
                send_data(shm_send, (2, 1), self.center_delivery)
                # continue

            self.target = check_type  # 메인문 출력을 위한 저장

            undistorted_frame = YOLOV8.undist.undistort(frame)

            results = model.predict(undistorted_frame, imgsz=640, half=True, device=0, conf=0.5)
            result = results[0]

            for detection in result:
                box = detection.boxes.cpu().numpy()[0]

                # ===========================
                label_cls_idx = int(box.cls[0])

                xywh = box.xywh[0].astype(int)
                # area        = self.xywh[2] * self.xywh[3]
                xyxy = box.xyxy[0].astype(int)
                # x1      = self.xyxy[0]
                # y1      = self.xyxy[1]

                # 배달 표지판 인식
                detect_object: str = self.__label_cls_name[label_cls_idx]

                match check_type, detect_object:
                    case (_, 'Blue-square-sign'):
                        blue_squares.append(xyxy)

                    case ('A', 'A'):
                        a_signs.append(xywh)
                        destination_idx = label_cls_idx

                    case ('A', '1') | ('A', '2') | ('A', '3'):
                        destinations.append(xywh)

                        if detect_object == '1':
                            cnt_dlivery_num[0] += 1
                        elif detect_object == '2':
                            cnt_dlivery_num[1] += 1
                        elif detect_object == '3':
                            cnt_dlivery_num[2] += 1

                    case ('B', 'B'):
                        b_signs.append(xywh)
                        destination_idx = label_cls_idx

                    case ('B', _):
                        if detect_object == find:
                            destinations.append(xywh)

                # 가장 많이 검출된 배달 번호를 찾는다
                find = str(cnt_dlivery_num.index(max(cnt_dlivery_num)) + 1)

            self._deliv_active(check_type, undistorted_frame, frame_count, blue_squares,
                               a_signs, b_signs, destinations, destination_idx)

            send_data(shm_send, (2, 1), self.center_delivery)

            diff_time = time.time() - prev_time

            if diff_time > 0:
                fps = 1 / diff_time

            cv.putText(undistorted_frame, f'FPS    : {fps:.2f}', (20, 40), YOLOV8.__user_font, 1, (0, 255, 0), 2)
            cv.putText(undistorted_frame, f'Target : {check_type}', (20, 70), YOLOV8.__user_font, 1, (0, 255, 0), 2)
            cv.putText(undistorted_frame, f'Number : {find}', (20, 100), YOLOV8.__user_font, 1, (0, 255, 0), 2)

            self.__frame_out = undistorted_frame
            # cv.imshow(self.__window_name, self.__frame_out)

            if self.__save_output:
                out.write(undistorted_frame)

            sm_data = []  # sm data list 초기화
            blue_squares.clear()  # 파란 표지판 좌표 (xyxy)
            a_signs.clear()
            b_signs.clear()  # B 좌표 (xywh)
            destinations.clear()  # 도착지 숫자 좌표 (xywh)

        # 반복문 종료시 memory 해제
        close_shm(shm_send)

    def _deliv_active(self, check_type: str, frame: np.ndarray,
                      frame_count: list[int],
                      blue_squares: list[tuple[int, int, int, int]],
                      a_signs: list[tuple[int, int, int, int]],
                      b_signs: list[tuple[int, int, int, int]],
                      destination: list[tuple[int, int, int, int]],
                      color_idx: int
                      ):

        signs: list[tuple[int, int, int, int]]
        bs_idx: int = -1

        match check_type:
            case 'A':
                signs = a_signs
            case 'B':
                signs = b_signs
            case _:
                return

        if blue_squares and destination and signs:
            for index, blue_square in enumerate(blue_squares):
                if blue_square[0] < destination[0][0] < blue_square[2] and blue_square[1] < destination[0][1] < \
                        blue_square[3]:
                    bs_idx = index
                    break

            if bs_idx == -1:
                return

            for index, sign in enumerate(signs):
                if blue_squares[bs_idx][0] < sign[0] < blue_squares[bs_idx][2] and \
                        blue_squares[bs_idx][1] < sign[1] < blue_squares[bs_idx][3]:
                    frame_count[0] += 1  # 5 frame 연속으로 count가 되면 표지판으로 인식한다

        else:
            frame_count[0] = 0

        if frame_count[0] > 5:
            center_x = int((blue_squares[bs_idx][0] + blue_squares[bs_idx][2]) / 2)
            center_y = int((blue_squares[bs_idx][1] + blue_squares[bs_idx][3]) / 2)

            self.center_delivery = [center_x, center_y]

            cv.rectangle(frame, blue_squares[bs_idx][:2], blue_squares[bs_idx][2:],
                         colors[color_idx], thickness=YOLOV8.__line_width, lineType=cv.LINE_AA)
        else:
            self.center_delivery = []

    def print_traffic_result(self):
        print("--------------------TRAFFIC-------------------")
        print(f"TRAFFIC STOP            : {YOLOV8.traffic_stop}")
        print(f"TRAFFIC AREA            : {self.current_sign_info}")

    def _traffic_mission(self, cap: cv.VideoCapture, model: YOLO, out: cv.VideoWriter = None):

        sm_name_recv: str = MAIN_TO_TRAFFIC

        frame: np.ndarray
        undistorted_frame: np.ndarray
        ret: bool
        fps: float = 0

        # yolo 변수

        xywh: tuple[int, int, int, int]
        xyxy: tuple[int, int, int, int]
        area: int
        max_area: int = 0
        max_idx: int = 0

        check_object: list[str, 2]

        frame_count: list[int] = [0]

        traffic_detect_cls: str = ' '
        traffic_detect_cnt: int = 0
        label_cls_idx: int = 0
        zoom_scale: float = 1
        current_sign_info: int

        while cap.isOpened() and not self.threadStop:
            start_time = time.time()
            prev_time = start_time

            # 변수 초기화
            max_area = 0

            # Read a frame from the video
            ret, frame = cap.read()

            if not ret:
                print("Camera is Disconnected ...!")
                break

            if self.__morai:
                zoom_frame = frame

            # TODO 차에서 내가 숫자 누르면 직진, 4구, 3구 할 수 있도록 변형 해보자
            # self.current_sign_info = self.traffic_give_sign
            self.current_sign_info = recv_data(sm_name_recv, 1)
            YOLOV8.mission_state_flag = self.current_sign_info

            if self.current_sign_info == MISSION_TURNLEFT4:  # 좌회전 4구
                max_det_num = 3
                zoom_scale = 1.6
            if self.current_sign_info == MISSION_TURNLEFT3:  # 좌회전 3구
                max_det_num = 3
                zoom_scale = 2
            else:
                max_det_num = 1
                zoom_scale = 1.6

            zoom_frame = digital_zoom(frame, zoom_scale, self.width, self.height)

            zoom_frame_roi = extract_roi(zoom_frame)
            # 한 프레임에서 신호등은 하나만 인식하면 되기 때문에 max_det = 1로 설정
            results = model.predict(zoom_frame_roi, imgsz=640, half=True, device=0, conf=0.7, max_det=max_det_num)
            result = results[0]

            for detection in result:

                box = detection.boxes.cpu().numpy()[0]

                # ===========================
                label_cls_idx = int(box.cls[0])
                xywh = box.xywh[0].astype(int)
                area = xywh[2] * xywh[3]
                xyxy = box.xyxy[0].astype(int)
                conf = box.conf[0]

                if area > max_area:
                    max_area = area
                    max_idx = label_cls_idx
                    max_box = box

            if not result:  # 객체가 인식이 된다면
                traffic_detect_cls = ' '
                traffic_detect_cnt = 0

            else:
                draw_box(zoom_frame, max_box, self.__label_cls_name)

            # TODO 현재 GPS 위치에 따라서 카메라 인식과 카메라 정지를 결정한다
            if YOLOV8.mission_state_flag == MISSION_STRAIGHT or YOLOV8.mission_state_flag == MISSION_TURNLEFT4 or YOLOV8.mission_state_flag == MISSION_TURNLEFT3:

                # 만약 이전 프레임과 다른 객체가 인식 되면
                if traffic_detect_cls != self.__label_cls_name[max_idx]:
                    traffic_detect_cls = self.__label_cls_name[max_idx]
                    traffic_detect_cnt = 0

                # 이전 프레임과 같은 객체가 인식되면 카운드 증가
                else:
                    traffic_detect_cnt += 1

                if traffic_detect_cnt == 5:

                    if self.current_sign_info == MISSION_STRAIGHT:  # 직진

                        match traffic_detect_cls:

                            case 'G' | 'G-L' if YOLOV8.traffic_stop:
                                YOLOV8.traffic_stop = False

                            case 'R-L' | 'R' | 'Y' if not YOLOV8.traffic_stop:
                                YOLOV8.traffic_stop = True

                    elif self.current_sign_info == MISSION_TURNLEFT4:  # 4구 좌회전

                        match traffic_detect_cls:

                            case 'R-L' | 'G-L' if YOLOV8.traffic_stop:
                                YOLOV8.traffic_stop = False

                            case 'G' | 'R' | 'Y' if not YOLOV8.traffic_stop:
                                YOLOV8.traffic_stop = True

                    elif self.current_sign_info == MISSION_TURNLEFT3:  # 3구 좌회전

                        match traffic_detect_cls:

                            case 'R-L-3' if YOLOV8.traffic_stop:
                                YOLOV8.traffic_stop = False

                            case 'G' | 'R' | 'Y' | 'R-L' | 'G-L' if not YOLOV8.traffic_stop:
                                YOLOV8.traffic_stop = True

            else:
                YOLOV8.traffic_stop = False

            if YOLOV8.traffic_stop:
                cv.putText(zoom_frame, f'Brake : {YOLOV8.traffic_stop}', (20, 80), YOLOV8.__user_font, 1,
                           (0, 0, 255), 2)
            else:
                cv.putText(zoom_frame, f'Brake : {YOLOV8.traffic_stop}', (20, 80), YOLOV8.__user_font, 1,
                           (0, 255, 0), 2)

            # detect_object: str = self.__label_cls_name[label_cls_idx]
            diff_time = time.time() - prev_time

            if diff_time > 0:
                fps = 1 / diff_time

            cv.putText(zoom_frame, f'FPS : {fps:.2f}', (20, 40), YOLOV8.__user_font, 1, (0, 255, 0), 2)
            self.__frame_out = zoom_frame

            if self.__save_output:
                out.write(self.__frame_out)

    def print_lane_result(self):
        print("---------------------LANE---------------------")

        # print(f"GAMMA                   : {self.total_gamma}")
        print(f"OUTPUT GAMMA            : {self.out_gamma}")
        # print(f"VELOCITY CMD            : {self.vel_cmd}")

        print(f"LIDAR STEER             : {self.lidar_steer}")
        # print(f"STOP FLAG               : {self.shm_stop_flag}")
        print(f"BEV RUBBER FLAG         : {YOLOV8.rub_bev_flag}")
        print(f"TUNNLE FLAG             : {YOLOV8.rubber_in_tunnel}")

    def _lane_mission(self, cap: cv.VideoCapture, model: YOLO, out: cv.VideoWriter = None):

        sm_name_gamma_send: str = "LANE_GAMMA_TO_MAIN"
        sm_name_velocity_send: str = "LANE_VELOCITY_TO_MAIN"
        sm_name_stop_send: str = "LANE_STOP_TO_MAIN"

        sm_name_traffic_stop_send: str = "TRAFFIC_TO_MAIN"

        shm_send_size: int = 4 * 1
        shm_gamma_send = open_send_shm(sm_name_gamma_send, shm_send_size)
        shm_velocity_send = open_send_shm(sm_name_velocity_send, shm_send_size)
        shm_stop_send = open_send_shm(sm_name_stop_send, shm_send_size)

        shm_traffic_stop_send = open_send_shm(sm_name_traffic_stop_send, shm_send_size)

        ipm_frame: np.ndarray
        frame: np.ndarray
        undistorted_frame: np.ndarray
        ret: bool
        fps: float = 0

        # yolo 변수
        label_cls_idx: int = 0
        xywh: tuple[int, int, int, int]
        xyxy: tuple[int, int, int, int]

        x_center: int
        self.y_line_center: int
        right_avg_x: list[int, int, int]
        left_avg_x: list[int, int, int]

        # 좌표 이전값
        # 처음에는 None으로 초기화 해야함

        prev_right_avg_x: list[int, int, int] = None
        prev_left_avg_x: list[int, int, int] = None

        gamma0: float = 0.0
        gamma1: float = 0.0
        gamma2: float = 0.0

        self.left_lane_center: int = 0
        self.right_lane_center: int = 0
        self.stop_line_y_center: int = 0

        stop_lines: list
        stop_lines_idx: int
        stop_line_flag: bool = False
        stop_line_cnt: int = 0
        line_idx: int = 0
        lines_data: dict
        self.y_line_flag: bool = False
        self.w_line_flag: bool = False

        self.single_lane_cnt: int = 0

        self.right_lane: bool = False
        self.left_lane: bool = False

        self.static_executed: bool = False  # 정적이 실행된적이 있는가
        self.pre_rubber_state: str = ''

        rubber_l_line: list[int, int, int]
        rubber_r_line: list[int, int, int]

        non_rubber_line: list[int, int, int]

        self.rubber_left_flag: bool = False
        self.rubber_right_flag: bool = False

        while cap.isOpened() and not self.threadStop:

            start_time = time.time()
            prev_time = start_time

            # Read a frame from the video
            ret, frame = cap.read()

            if not ret:
                print("Camera is Disconnected ...!")
                break

            # bottom 카메라 bev로 변환
            if self.__morai:
                undistorted_frame = frame
                ipm_frame = YOLOV8.ipm.ipm_transform(undistorted_frame)
            else:
                undistorted_frame = YOLOV8.undist.undistort(frame)
                ipm_frame = YOLOV8.ipm.ipm_transform(undistorted_frame)

            # yolo 모델에서 결과 받아오기
            results = model.predict(ipm_frame, imgsz=640, half=True, device=0, conf=0.5)
            result = results[0]

            # 각 변수 초기화
            left_segment = []
            right_segment = []
            self.y_line_center = None
            self.reverse_drive: int = 0

            line_idx = 0
            lines_data = {}

            stop_line_flag = False
            self.y_line_flag = False
            self.w_line_flag = False
            self.right_lane = False
            self.left_lane = False

            stop_lines_idx = 0
            stop_lines = []

            rubber_l_line = [0] * 3
            rubber_r_line = [0] * 3
            non_rubber_line = [0] * 3

            for detection in result:
                box = detection.boxes.cpu().numpy()[0]

                label_cls_idx = int(box.cls[0])
                xywh = box.xywh[0].astype(int)
                segment = detection.masks.xy[0].astype(int)

                if self.__label_cls_name[label_cls_idx] != 'stop_line':

                    lines_data[line_idx] = [label_cls_idx, xywh, segment]

                    if self.__label_cls_name[label_cls_idx] == 'y_line':

                        if xywh[0] < 450:  # xywh[0] = x_cneter
                            # 노란선이 420 픽셀보다 오른쪽에 있으면 노란색 차선이 아니라고 판단
                            self.y_line_flag = True
                            self.y_line_center = xywh[0]

                            self.reverse_drive = 0

                        else:
                            self.y_line_flag = False
                            self.y_line_center = None

                            self.reverse_drive += 1

                    line_idx += 1
                else:
                    stop_lines.append({'index': stop_lines_idx,
                                       'x_center': xywh[0],  # x_center == xywh[0]
                                       'y_center': xywh[1],
                                       'segment': segment})

                    stop_lines_idx += 1

            if lines_data:

                for key in lines_data:  # [label_cls_idx, xywh, segment]

                    line = lines_data[key]
                    line_cls_idx = line[0]
                    line_xywh = line[1]
                    line_x_center = line_xywh[0]
                    line_segment = line[2]

                    if self.y_line_flag:  # 중앙선이 있을 때

                        if self.__label_cls_name[line_cls_idx] == 'w_line':

                            if self.y_line_center < line_x_center:  # 노란 차선보다 오른쪽에 있는 흰 차선만 주행 차선으로 인정
                                # self.right_lane = True
                                # right_segment.extend(line_segment)
                                # self.right_lane_center = line_x_center

                                if line_x_center > 320:
                                    self.right_lane = True
                                    right_segment.extend(line_segment)
                                    self.right_lane_center = line_x_center

                                elif line_x_center < 320:
                                    self.left_lane = True
                                    left_segment.extend(line_segment)
                                    self.left_lane_center = line_x_center

                            else:  # 노란 차선보다 왼쪽에 있는 흰 차선은 넘어감
                                pass

                        else:  # 노란 차선일 때

                            if line_x_center < 320:
                                self.left_lane = True
                                left_segment.extend(line_segment)
                                self.left_lane_center = line_x_center

                            elif line_x_center > 320:
                                self.right_lane = True
                                right_segment.extend(line_segment)
                                self.right_lane_center = line_x_center

                        fill_poly(line_cls_idx, ipm_frame, line_segment)

                    # 중앙선이 없을 때
                    else:
                        if line_x_center > 320:
                            self.right_lane = True
                            right_segment.extend(line_segment)
                            self.right_lane_center = line_x_center
                            self.w_line_flag = True

                        elif line_x_center < 320:
                            self.left_lane = True
                            left_segment.extend(line_segment)
                            self.left_lane_center = line_x_center

                        fill_poly(label_cls_idx, ipm_frame, line_segment)

            # stop_line_flag 선택
            # --------------------------------------------------------------------------------------------

            # TODO 신호등 작동하는지 확인하기 in morai
            if YOLOV8.mission_state_flag == MISSION_STRAIGHT or YOLOV8.mission_state_flag == MISSION_TURNLEFT4 or YOLOV8.mission_state_flag == MISSION_TURNLEFT3:

                for stop_line in stop_lines:

                    if self.left_lane_center < stop_line['x_center'] < self.right_lane_center:

                        stop_line_flag = True

                        # 정지선을 그리는 부분
                        if stop_line['segment'].shape[0] > 0:
                            color = colors[0]
                            cv.fillPoly(ipm_frame, [stop_line['segment']], color)

                            self.stop_line_y_center = stop_line['y_center']
                        break

                if YOLOV8.traffic_stop:  # traffic_stop이 true면 각 위치에 맞는 정지 flag # 정지선이 인식이 안될 수도 있어서 신호등만 했다

                    stop_line_cnt += 1

                    if stop_line_cnt > 5:

                        if self.stop_line_y_center > 200:
                            cv.putText(ipm_frame, f'Brake', (20, 80), YOLOV8.__user_font, 1, (0, 0, 255), 2)
                            self.shm_stop_flag = CAM2MAIN_BRAKE

                else:
                    stop_line_cnt = 0
                    self.shm_stop_flag = CAM2MAIN_NONE

            else:
                stop_line_cnt = 0
                self.shm_stop_flag = CAM2MAIN_NONE

            send_data(shm_traffic_stop_send, (1, 1), [self.shm_stop_flag])
            # --------------------------------------------------------------------------------------------

            # 우측 좌측 차선 각각의 중앙점 추출 (섹션 별로)
            right_avg_x = mean_x_list(right_segment, YOLOV8.y_ranges)
            left_avg_x = mean_x_list(left_segment, YOLOV8.y_ranges)

            # None 값 제거
            right_avg_x = replace_none(right_avg_x, prev_right_avg_x)
            left_avg_x = replace_none(left_avg_x, prev_left_avg_x)

            # 이전 값에 현재 값을  저장
            prev_right_avg_x = right_avg_x if right_avg_x is not None else prev_right_avg_x
            prev_left_avg_x = left_avg_x if left_avg_x is not None else prev_left_avg_x

            # --------------- 라인 기반 기준 각도 산출 ---------------
            middle_points = get_middle_point(prev_right_avg_x, prev_left_avg_x, YOLOV8.y_sections)

            if len(middle_points) == 3:
                gamma0 = calculate_gamma(YOLOV8.origin_point, middle_points[2])
                gamma1 = calculate_gamma(middle_points[2], middle_points[1])
                gamma2 = calculate_gamma(middle_points[1], middle_points[0])

                self.total_gamma = (0.7 * gamma0 + 0.2 * gamma1 + 0.1 * gamma2)

            # 라이다로부터 line flag 시작
            flag = recv_data(STATIC_FALG_TO_LANE, 1)

            YOLOV8.rubber_in_tunnel = flag
            # 음영지역 lane case
            # 터널 & 정적 켜진적 X
            # 속도 8 주행 & 일반 gamma
            if flag == YEAH_TUNNEL and not self.static_executed:

                if len(middle_points) == 3:
                    draw_points(ipm_frame, middle_points, YOLOV8.origin_point)

                # --------------- 최종 각도 및 속도 ---------------
                if abs(self.total_gamma) < 3:
                    self.vel_cmd = int(self.vel_before_static)
                    self.out_gamma = int(0.8 * self.total_gamma)

                elif abs(self.total_gamma) > 7:
                    self.vel_cmd = int(self.vel_before_static - 0.5 * abs(self.total_gamma))
                    self.out_gamma = int(0.4 * self.total_gamma)

                else:
                    self.vel_cmd = int(self.vel_before_static - 0.3 * abs(self.total_gamma))
                    self.out_gamma = int(0.6 * self.total_gamma)

                cv.putText(ipm_frame, f'normal gamma : {self.out_gamma:.2f}', (20, 80), YOLOV8.__user_font, 1,
                           (0, 255, 0), 2)

            # # 터널 내 정적 중
            # # # 속도 5 주행 & 혼합 gamma
            elif flag == YEAH_TUNNEL_STATIC:

                if YOLOV8.rub_bev_flag == RUBBER_NONE and not self.static_executed:

                    # 러버콘이 인식 되기 전은 그냥 위에서 찾은 라인으로 가면 됨
                    pass

                elif YOLOV8.rub_bev_flag == RUBBER_LEFT:
                    # 오른쪽 차선 점 세개의 x값 차이를 구해서 똑같이 더한다

                    self.static_executed = True

                    YOLOV8.rub_bev_l_point[0] = YOLOV8.rub_bev_l_point[0] * 1.1

                    # 영향력이 가장 적은 점
                    rubber_l_line[0] = YOLOV8.rub_bev_l_point[0] + (prev_right_avg_x[0] - prev_right_avg_x[1])

                    # 두번째 아래점
                    rubber_l_line[1] = YOLOV8.rub_bev_l_point[0] + (prev_right_avg_x[1] - prev_right_avg_x[2])

                    # 제일 아래 점
                    rubber_l_line[2] = YOLOV8.rub_bev_l_point[0]

                    middle_points = get_middle_point(prev_right_avg_x, rubber_l_line, YOLOV8.y_sections)

                    self.pre_rubber_state = RUBBER_LEFT

                    self.rubber_left_flag = True

                elif YOLOV8.rub_bev_flag == RUBBER_RIGHT:

                    self.static_executed = True

                    YOLOV8.rub_bev_r_point[0] = YOLOV8.rub_bev_r_point[0] * 0.9

                    # 영향력이 가장 적은 점
                    rubber_r_line[0] = YOLOV8.rub_bev_r_point[0] + (prev_left_avg_x[0] - prev_left_avg_x[1])

                    # 두번째 아래점
                    rubber_r_line[1] = YOLOV8.rub_bev_r_point[0] + (prev_left_avg_x[1] - prev_left_avg_x[2])

                    # 제일 아래 점
                    rubber_r_line[2] = YOLOV8.rub_bev_r_point[0]

                    middle_points = get_middle_point(rubber_r_line, prev_left_avg_x, YOLOV8.y_sections)

                    self.pre_rubber_state = RUBBER_RIGHT

                    self.rubber_right_flag = True

                elif YOLOV8.rub_bev_flag == RUBBER_BOTH:

                    self.static_executed = True

                    # 왼쪽 클러스터가 먼저 나온 케이스
                    if YOLOV8.rub_bev_l_point[1] > YOLOV8.rub_bev_r_point[1]:

                        YOLOV8.rub_bev_l_point[0] = YOLOV8.rub_bev_l_point[0] * 1.1
                        # 영향력이 가장 적은 점
                        rubber_l_line[0] = YOLOV8.rub_bev_l_point[0] + (prev_right_avg_x[0] - prev_right_avg_x[1])

                        # 두번째 아래점
                        rubber_l_line[1] = YOLOV8.rub_bev_l_point[0] + (prev_right_avg_x[1] - prev_right_avg_x[2])

                        # 제일 아래 점
                        rubber_l_line[2] = YOLOV8.rub_bev_l_point[0]

                        middle_points = get_middle_point(prev_right_avg_x, rubber_l_line, YOLOV8.y_sections)

                    else:  # 오른쪽 클러스터가 먼저 나오는 경우

                        YOLOV8.rub_bev_r_point[0] = YOLOV8.rub_bev_r_point[0] * 0.9

                        rubber_r_line[0] = YOLOV8.rub_bev_r_point[0] + (prev_left_avg_x[0] - prev_left_avg_x[1])

                        # 두번째 아래점
                        rubber_r_line[1] = YOLOV8.rub_bev_r_point[0] + (prev_left_avg_x[1] - prev_left_avg_x[2])

                        # 제일 아래 점
                        rubber_r_line[2] = YOLOV8.rub_bev_r_point[0]

                        middle_points = get_middle_point(rubber_r_line, prev_left_avg_x, YOLOV8.y_sections)


                elif YOLOV8.rub_bev_flag == RUBBER_NONE and self.static_executed and self.rubber_left_flag and self.rubber_right_flag:
                    # 러버콘의 인식이 끊기기 전의 state를 알아야 한다.
                    # pre state를 담는 변수 필요
                    # 그러면 전에 인식 되었던 반대 라인과 중앙라인 얼추 잡아서

                    # (139, 255),
                    # (455, 253),
                    # (149, 143),
                    # (454, 142),
                    # (304, 144), 차선 중앙

                    # print("끝났다고오오오오")
                    if self.pre_rubber_state == RUBBER_RIGHT:

                        # gamma1은 degree 단위다
                        # gamma1을 이용해서 153/cos(gamma1)을 구한다

                        non_rubber_line[2] = prev_left_avg_x[2] + 153 / math.cos(math.radians(gamma1))

                        non_rubber_line[1] = non_rubber_line[2] + (prev_left_avg_x[1] - prev_left_avg_x[2])

                        non_rubber_line[0] = non_rubber_line[2] + (prev_left_avg_x[0] - prev_left_avg_x[1])

                        middle_points = get_middle_point(non_rubber_line, prev_left_avg_x, YOLOV8.y_sections)


                    elif self.pre_rubber_state == RUBBER_LEFT:

                        non_rubber_line[2] = prev_right_avg_x[2] - 153 / math.cos(math.radians(gamma1))

                        non_rubber_line[1] = non_rubber_line[2] + (prev_right_avg_x[1] - prev_right_avg_x[2])

                        non_rubber_line[0] = non_rubber_line[2] + (prev_right_avg_x[0] - prev_right_avg_x[1])

                        middle_points = get_middle_point(prev_right_avg_x, non_rubber_line, YOLOV8.y_sections)

                if len(middle_points) == 3:
                    draw_points(ipm_frame, middle_points, YOLOV8.origin_point)

                    gamma0 = calculate_gamma(YOLOV8.origin_point, middle_points[2])
                    gamma1 = calculate_gamma(middle_points[2], middle_points[1])
                    gamma2 = calculate_gamma(middle_points[1], middle_points[0])

                    self.total_gamma = (0.7 * gamma0 + 0.2 * gamma1 + 0.1 * gamma2)

                self.vel_cmd = self.vel_static_ing
                self.out_gamma = int(self.total_gamma * 0.6)
                cv.putText(ipm_frame, f'static gamma : {self.out_gamma:.2f}', (20, 80), YOLOV8.__user_font, 1,
                           (0, 255, 0), 2)

            elif flag == YEAH_TUNNEL_STATIC_DONE:

                draw_points(ipm_frame, middle_points, YOLOV8.origin_point)

                if self.right_lane and self.left_lane:
                    self.vel_cmd = self.vel_static_ing
                    self.out_gamma = int(self.total_gamma * 0.5)
                    cv.putText(ipm_frame, f'comeback gamma : {self.out_gamma:.2f}', (20, 80), YOLOV8.__user_font, 1,
                               (0, 255, 0), 2)

                else:  # 차선 하나만 인식하는 경우
                    self.single_lane_cnt += 1
                    self.vel_cmd = self.vel_static_ing
                    self.out_gamma = int(self.total_gamma * 0.6)

                    if self.single_lane_cnt > 10:  # 하나의 차선이 인식될 때

                        if self.y_line_flag and not self.w_line_flag:  # 노란 차선만 인식 될 때 우회전
                            self.vel_cmd = self.vel_static_ing
                            self.out_gamma = 7

                        elif not self.y_line_flag and self.w_line_flag:  # 흰색 차선만 인식 될 때 좌회전
                            self.vel_cmd = self.vel_static_ing
                            self.out_gamma = -7

                    cv.putText(ipm_frame, f'single lane gamma : {self.out_gamma:.2f}', (20, 80), YOLOV8.__user_font,
                                1,
                                (0, 255, 0), 2)

            # # 터널 & 정적 켜진적 O
            # # 속도 13 주행 & 일반 gamma
            elif flag == YEAH_TUNNEL and self.static_executed:

                draw_points(ipm_frame, middle_points, YOLOV8.origin_point)

                if abs(self.total_gamma) < 3:
                    self.vel_cmd = int(self.vel_after_static)
                    self.out_gamma = int(0.8 * self.total_gamma)

                elif abs(self.total_gamma) > 7:
                    self.vel_cmd = int(self.vel_after_static - 0.5 * abs(self.total_gamma))
                    self.out_gamma = int(0.4 * self.total_gamma)

                else:
                    self.vel_cmd = int(self.vel_after_static - 0.3 * abs(self.total_gamma))
                    self.out_gamma = int(0.6 * self.total_gamma)

                cv.putText(ipm_frame, f'after static gamma : {self.out_gamma:.2f}', (20, 80), YOLOV8.__user_font, 1,
                           (0, 255, 0), 2)

            send_data(shm_velocity_send, (1, 1), [self.vel_cmd])
            send_data(shm_gamma_send, (1, 1), [self.out_gamma])

            diff_time = time.time() - prev_time
            if diff_time > 0:
                fps = 1 / diff_time

            cv.putText(ipm_frame, f'FPS : {fps:.2f}', (20, 40), YOLOV8.__user_font, 1, (0, 255, 0), 2)
            self.__frame_out = ipm_frame

            if self.__save_output:
                out.write(self.__frame_out)

    def print_rubber_result(self):
        print("--------------------RUBBER--------------------")
        print(f"RUBBER FLAG             : {YOLOV8.rub_bev_flag}")
        print(f"RUBBER LEFT POINT       : {YOLOV8.rub_bev_l_point}")
        print(f"RUBBER RIGHT POINT      : {YOLOV8.rub_bev_r_point}")

    def _rubber_mission(self, cap: cv.VideoCapture, model: YOLO, out: cv.VideoWriter = None):

        sm_name_send: str = RUBBER_TO_STATIC

        shm_send_size: int = 4 * 2  # 4 * 3 * 3   (int  int) * 1
        shm_send = open_send_shm(sm_name_send, shm_send_size)

        frame: np.ndarray
        undistorted_frame: np.ndarray
        ret: bool
        fps: float = 0

        # yolo 변수
        xyxy: tuple[int, int, int, int]

        self.max_center_y: int

        self.rubber_center: list[tuple[int, int]]

        x_center: int
        y_center: int
        self.ipm_point_data: list
        self.max_area: int = 0

        cluster_1 = np.array([])
        cluster_2 = np.array([])
        coordinates = np.array([])
        left_rub = np.array([])
        right_rub = np.array([])

        rub_x_criteria: int = 320

        while cap.isOpened() and not self.threadStop:

            start_time = time.time()
            prev_time = start_time

            # Read a frame from the video
            ret, frame = cap.read()

            # roi를 위해 frame 복사
            if not ret:
                print("Camera is Disconnected ...!")
                break

            if self.__morai:
                undistorted_frame = frame.copy()
                # frame_roi = np.copy(undistorted_frame)
                # frame_roi[0: self.height // 2, 0: self.width] = 0
            else:
                undistorted_frame = YOLOV8.undist.undistort(frame.copy())
                # undistorted_frame = frame.copy()  # 지금 임시로 하는거라

            results = model.predict(undistorted_frame, imgsz=640, half=True, device=0, conf=0.4, max_det=4)
            result = results[0]

            undistorted_frame = YOLOV8.ipm.ipm_transform(undistorted_frame)

            # rubber cone 개수 초기화
            self.rubber_cnt = 0
            self.max_center_y = 0
            self.max_area = 0
            self.rubber_center = []

            # YOLOV8.rub_bev_l_point = []
            # YOLOV8.rub_bev_r_point = []

            self.ipm_point_data = []


            cluster_1 = np.array([])
            cluster_2 = np.array([])
            coordinates = np.array([])
            left_rub = np.array([])
            right_rub = np.array([])

            if YOLOV8.rubber_in_tunnel == YEAH_TUNNEL or YOLOV8.rubber_in_tunnel == YEAH_TUNNEL_STATIC:

                for detection in result:
                    box = detection.boxes.cpu().numpy()[0]
                    label_cls_idx = int(box.cls[0])
                    xywh = box.xywh[0].astype(int)
                    xyxy = box.xyxy[0].astype(int)

                    x_l, y_l = xyxy[0], xyxy[3]
                    x_c, y_c = (xyxy[0] + xyxy[2]) // 2, xyxy[3]
                    x_r, y_r = xyxy[2], xyxy[3]

                    self.ipm_point_data = YOLOV8.ipm_point.transform_points(
                        np.float32([(x_l, y_l), (x_c, y_c), (x_r, y_r)]))
                    coordinates = np.squeeze(self.ipm_point_data, axis=1)  # 두번째 차원을 제거 하는 코드

                    # ipm 변환 중앙점 y 좌표가 0보다 클 때만 인식 시작
                    if 450 > coordinates[1, 1] > 10:

                        if cluster_1.size == 0 and cluster_2.size == 0:  # 제일 처음 detection 된 obj
                            cluster_1 = np.array(coordinates)

                        elif distance(cluster_1[1], (x_c, y_c)) < 220:  # 튜닝의 영역

                            if cluster_1[1, 0] > coordinates[1, 0]:  # 먼저 저장된 중앙점이 더 우측에 있을 때
                                # 가장 왼쪽에 있는 점을 업데이트
                                cluster_1[0, 0] = coordinates[0, 0]

                            else:  # 먼저 저장된 중앙점이 왼쪽에 있을 때
                                # 가장 우측에 있는 점을 업데이트
                                cluster_1[2, 0] = coordinates[2, 0]

                            # 중앙점을 새롭게 저장된 끝점을 기준으로 하나의 점을 만든다
                            cluster_1[1, 0] = int((cluster_1[0, 0] + cluster_1[2, 0]) / 2)

                            # cluster_1 = np.append(cluster_1, [coordinates], axis=0)
                        else:

                            if cluster_2.size != 0:  # 이미 클러스터 2가 채워져 있을 때
                                if cluster_2[1, 0] > coordinates[1, 0]:  # 먼저 저장된 중앙점이 더 우측에 있을 때
                                    # 가장 왼쪽에 있는 점을 업데이트
                                    cluster_2[0, 0] = coordinates[0, 0]

                                else:  # 먼저 저장된 중앙점이 왼쪽에 있을 때p
                                    # 가장 우측에 있는 점을 업데이트
                                    cluster_2[2, 0] = coordinates[2, 0]

                                # 중앙점을 새롭게 저장된 끝점을 기준으로 하나의 점을 만든다
                                cluster_2[1, 0] = int((cluster_2[0, 0] + cluster_2[2, 0]) / 2)
                            else:  # 클러스터 2에 처음으로 저장할 때
                                cluster_2 = coordinates

                    if self.max_center_y < xywh[1]:
                        x_center = xywh[0]
                        y_center = xywh[1]
                        self.max_center_y = y_center
                        self.rubber_center = [x_center, y_center]
                        self.max_area = xywh[2] * xywh[3]
                        self.max_ratio = xywh[3] / xywh[2]

                    draw_box(undistorted_frame, box, self.__label_cls_name)

                # YOLOV8.rubber_bev_points = self.ipm_point_data
                # ---------------------
                if self.max_area > 200 and self.max_ratio > 1.1:
                    send_data(shm_send, (2, 1), self.rubber_center)
                else:
                    self.rubber_center = [0, 0]
                    send_data(shm_send, (2, 1), self.rubber_center)
                # ---------------------

                # rubber의 좌우를 정한다

                # 하나의 클러스터만 잡힐 때
                if cluster_1.size != 0 and cluster_2.size == 0:
                    # 처음 정적 기준 320
                    # 정적의 기준을 지속적으로 업데이트
                    # print("cluster 1")
                    if cluster_1[1, 0] < rub_x_criteria:
                        left_rub = cluster_1
                        rub_x_criteria = cluster_1[1, 0] + 50

                    elif cluster_1[1, 0] > rub_x_criteria:
                        right_rub = cluster_1
                        rub_x_criteria = cluster_1[1, 0] - 50

                # 두개의 클러스터가 잡힐 때
                elif cluster_1.size != 0 and cluster_2.size != 0:
                    # print("cluster 2")
                    if rub_x_criteria > cluster_1[1, 0]:
                        left_rub = cluster_1
                        right_rub = cluster_2
                    else:
                        left_rub = cluster_2
                        right_rub = cluster_1

                    rub_x_criteria = int((cluster_1[1, 0] + cluster_2[1, 0]) / 2)

                # rubber의 좌우를 구분하면 왼쪽 클러스터는 x_r을 보내고 오른쪽 클러스터는 x_l을 보낸다
                # 클래스 변수로 line_mission쪽에서 바로 참조할 수 있게 한다
                if left_rub.size != 0 and right_rub.size == 0:
                    # print("만져당")
                    YOLOV8.rub_bev_flag = RUBBER_LEFT
                    YOLOV8.rub_bev_l_point = left_rub[2]
                    YOLOV8.rub_bev_r_point = []
                    cv.line(undistorted_frame, left_rub[2], left_rub[2], colors[6], 10)

                elif left_rub.size == 0 and right_rub.size != 0:

                    # print("국짐당")
                    YOLOV8.rub_bev_flag = RUBBER_RIGHT
                    YOLOV8.rub_bev_l_point = []
                    YOLOV8.rub_bev_r_point = right_rub[0]
                    cv.line(undistorted_frame, right_rub[0], right_rub[0], colors[6], 10)

                elif left_rub.size != 0 and left_rub.size != 0:

                    # print("중도")
                    YOLOV8.rub_bev_flag = RUBBER_BOTH
                    YOLOV8.rub_bev_l_point = left_rub[2]
                    YOLOV8.rub_bev_r_point = right_rub[0]
                    cv.line(undistorted_frame, left_rub[2], left_rub[2], colors[1], 10)
                    cv.line(undistorted_frame, right_rub[0], right_rub[0], colors[1], 10)

                elif left_rub.size == 0 and right_rub.size == 0:
                    # print("뭐이씨")
                    YOLOV8.rub_bev_flag = RUBBER_NONE

                cv.line(undistorted_frame, (rub_x_criteria, 480), (rub_x_criteria, 480), colors[0], 10)

            diff_time = time.time() - prev_time

            if diff_time > 0:
                fps = 1 / diff_time

            cv.putText(undistorted_frame, f'FPS : {fps:.2f}', (20, 40), YOLOV8.__user_font, 1, (0, 255, 0), 2)
            self.__frame_out = undistorted_frame

            # 영상 저장
            if self.__save_output:
                out.write(self.__frame_out)
            # 러버콘 인식 초기화
            b_rub_detect = False
            y_rub_detect = False
            d_rub_only = False
            r_rub_only = False
