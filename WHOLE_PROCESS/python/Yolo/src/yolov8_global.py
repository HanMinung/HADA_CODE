# flag data?
import math
from math import *
from multiprocessing import shared_memory

import cv2 as cv
import numpy as np
import yaml


def distance(point1, point2):
    return np.linalg.norm(point1 - point2)

def camera_init(top: bool = False, bottom: bool = False, upper: bool = False):
    cap = {}

    if top:
        cap_top = cv.VideoCapture(cam_names['top'], cv.CAP_DSHOW)
        cap_top.set(cv.CAP_PROP_FPS, 60.0)
        cap_top.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        cap_top.set(cv.CAP_PROP_AUTO_EXPOSURE, 0.75)
        # cap_top.set(cv.CAP_PROP_EXPOSURE, -4)

        cap["top"] = cap_top

    if bottom:
        cap_bottom = cv.VideoCapture(cam_names['bottom'], cv.CAP_DSHOW)
        cap_bottom.set(cv.CAP_PROP_FPS, 60.0)
        cap_bottom.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        cap_bottom.set(cv.CAP_PROP_AUTO_EXPOSURE, 0.75)
        # cap_bottom.set(cv.CAP_PROP_EXPOSURE, -4)
        cap["bottom"] = cap_bottom

    if upper:
        cap_upper = cv.VideoCapture(cam_names['upper'], cv.CAP_DSHOW)
        cap_upper.set(cv.CAP_PROP_FPS, 60.0)
        cap_upper.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        cap_upper.set(cv.CAP_PROP_AUTO_EXPOSURE, 0.75)
        # cap_top.set(cv.CAP_PROP_EXPOSURE, -4)

        cap["upper"] = cap_upper

    return cap

RUBBER_LEFT: str = 'left'
RUBBER_RIGHT: str = 'right'
RUBBER_BOTH: str = 'both'
RUBBER_NONE: str = 'none'

colors = {0: (255, 0, 0),  # 파란색
          1: (40, 188, 249),  # 개나리색
          2: (153, 0, 255),  # 진한 분홍
          3: (0, 153, 51),  # 진한 초록
          4: (255, 51, 153),  # 보라색
          5: (000, 102, 255),  # 주황색
          6: (0, 0, 255)
          }

label_list = {'rubber': 'yolo_file/rubber-3/data.yaml',
              'line': 'yolo_file/hada_bev_line_2-2/data.yaml',
              'delivery': 'yolo_file/Delivery-test2-9/data.yaml',
              'traffic': 'yolo_file/HADA_Light-8/data.yaml',
              }

cam_names: dict[str, int] = {'top': 0,
                             'bottom': 1,
                             'upper': 2, }

'''
YOLO 기본 함수
'''
def get_clsname(label_path):
    with open(label_path) as f:

        yaml_data: str = yaml.load(f, Loader=yaml.FullLoader)

        # Access the 'names' key and loop through the items
        cls_name: list[str] = yaml_data.get('names')

        if cls_name:
            for name in cls_name:
                print(name)
        else:
            print("'names' key not found in the YAML file")

    return cls_name

def draw_box(frame, box, cls_names):

    cls_idx = int(box.cls[0])
    xyxy = box.xyxy[0].astype(int)
    conf = box.conf[0]

    cv.rectangle(frame, xyxy[:2], xyxy[2:], colors[cls_idx],
                 thickness=2,
                 lineType=cv.LINE_AA)

    label: str = cls_names[cls_idx] + ' ' + str(f'{conf:.2f}')
    text_w, text_h = cv.getTextSize(label, 0, fontScale=2 / 3,
                                    thickness=1)[0]

    outside = xyxy[:2][1] - text_h >= 3
    text_p1 = xyxy[:2]
    text_p2 = xyxy[:2][0] + text_w, xyxy[:2][1] - text_h - 3 if outside else xyxy[:2][1] + text_h + 3
    cv.rectangle(frame, text_p1, text_p2, colors[cls_idx], -1, cv.LINE_AA)  # filled

    cv.putText(frame, label,
               (text_p1[0], text_p1[1] - 2 if outside else text_p1[1] + text_h + 2),
               0,
               2/3,
               (255, 255, 255),
               thickness=1,
               lineType=cv.LINE_AA)

''' 
_traffic_mission용 함수
'''
def extract_roi(frame):
    roi = frame[70:480, :, :]

    # ROI 이외의 영역을 검은색으로 채우기
    black_image = np.zeros_like(frame)

    # 빈 이미지에 ROI를 넣기
    black_image[70:480, :, :] = roi

    return black_image


def digital_zoom(frame, scale, width, height):
    center = (width // 2, height // 2)

    window_width = 640 // scale
    window_height = 480 // scale

    # 중심을 기준으로 배율에 따른 창 만들기
    startX, startY, endX, endY = center[0] - window_width // 2, center[1] - window_height // 2, center[
        0] + window_width // 2, center[1] + window_height // 2
    roi = frame[int(startY):int(endY), int(startX):int(endX)]

    zoomed = cv.resize(roi, (width, height), interpolation=cv.INTER_LINEAR)

    return zoomed


'''
_line_mission 용 함수
'''
# lane detection 함수
def mean_x_list(segment, y_ranges):
    avg_x_values = []
    for y_range in y_ranges:
        y_min, y_max = y_range
        xs_in_range = [point[0] for point in segment if y_min <= point[1] <= y_max]
        if xs_in_range:
            avg_x = int(sum(xs_in_range) / len(xs_in_range))
        else:
            avg_x = None  # No point in this y range
        avg_x_values.append(avg_x)
    return avg_x_values


def replace_none(val, previous_val):
    if previous_val is None:
        return val
    # Check and replace None only at the corresponding index
    return [v if v is not None else previous_val[i] for i, v in enumerate(val)]


# 중앙점을 받아온다
def get_middle_point(list1, list2, list_y):
    if len(list1) != len(list2):
        raise ValueError("Both lists should be of the same length")

    list_x = []
    for num1, num2 in zip(list1, list2):
        if num1 is not None and num2 is not None:
            list_x.append(int((num1 + num2) / 2.0))

    return list(zip(list_x, list_y))


def calculate_gamma(origin_point, target_point):
    dx = target_point[0] - origin_point[0]
    dy = origin_point[1] - target_point[1]  # y축의 증가 방향이 아래인 것을 반영

    radians = math.atan2(dy, dx) - math.radians(90)
    degrees = math.degrees(radians)
    degrees = degrees if degrees >= 0 else 360 + degrees

    return -(degrees if degrees <= 180 else degrees - 360)


def draw_lines(frame):
    cv.line(frame, (0, 37), (640, 37), colors[3], 2)
    cv.line(frame, (0, 143), (640, 143), colors[3], 2)
    cv.line(frame, (0, 252), (640, 252), colors[3], 2)
    cv.line(frame, (0, 361), (640, 361), colors[3], 2)


def draw_points(frame, middle_points, origin_point):
    cv.line(frame, middle_points[0], middle_points[0], colors[6], 10)
    cv.line(frame, middle_points[1], middle_points[1], colors[6], 10)
    cv.line(frame, middle_points[2], middle_points[2], colors[6], 10)
    cv.line(frame, origin_point, origin_point, colors[6], 10)


def fill_poly(idx: int, frame, segment: list):
    color = colors[idx]
    if segment.shape[0] > 0:
        cv.fillPoly(frame, [segment], color)


'''
공유 메모리 함수
'''


def open_send_shm(_shm_name: str, shm_size: int):
    print('--------------------------------------')
    print(_shm_name, " start...")
    shm = shared_memory.SharedMemory(name=_shm_name, create=True, size=shm_size)
    print("send sm memory is open...")
    print('--------------------------------------')
    return shm


# SM memory
def send_data(shm, shape: [int, int], _data: list):
    shared_buf = np.ndarray(shape, dtype='int', buffer=shm.buf)
    # Check if the middle_points list is empty and return if it is
    if not _data:  # A, 00 : cls,  000 : centerX,  000 : centerY
        shared_buf.fill(0)
        return
    else:

        shared_buf.fill(0)
        # shared_data에 현재 측정된 box의 좌표를 넘긴다.
        for i in range(len(_data)):
            shared_buf[i] = _data[i]


def recv_data(_shm_name, shape: int):
    try:
        shm = shared_memory.SharedMemory(name=_shm_name)
        shared_data = np.ndarray((shape,), dtype='int', buffer=shm.buf)
        shm_data = shared_data[:]
        # print("Shared Data:", shm_data[0])
        return shm_data[0]
    except FileNotFoundError:
        # print("recv None...")
        return 0


def close_shm(_shm_name):
    _shm_name.close()
    _shm_name.unlink()


# Distorted image (by wide angle lens) calibration
class CALIBRATION_WIDE_LENS:

    def __init__(self):
        self.wide_focal_x = 370.54525668
        self.wide_focal_y = 367.47281991

        self.wide_center_x = 322.3588068
        self.wide_center_y = 230.43829046

        self.dist_coef_k1 = -0.16524716
        self.dist_coef_k2 = -0.0169815
        self.dist_coef_p1 = -0.00244594
        self.dist_coef_p2 = 0.00038361
        self.dist_coef_k3 = 0.01692057

        self.wide_intrinsic_mat = np.array([[280.38650513, 0, 322.35003847],
                                            [0, 269.07183838, 226.76398398],
                                            [0, 0, 1]])

        self.wide_camera_mat = np.array([[self.wide_focal_x, 0, self.wide_center_x],
                                         [0, self.wide_focal_y, self.wide_center_y],
                                         [0, 0, 1]], dtype='double')

        self.dist_coeffs = np.array(
            [self.dist_coef_k1, self.dist_coef_k2, self.dist_coef_p1, self.dist_coef_p2, self.dist_coef_k3])


class UNDISTORT:

    def __init__(self):
        self.param = CALIBRATION_WIDE_LENS()

    def undistort(self, frame):
        h, w = frame.shape[:2]
        new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(self.param.wide_camera_mat, self.param.dist_coeffs,
                                                              (w, h), 1, (w, h))

        undistorted_frame = cv.undistort(frame, self.param.wide_camera_mat, self.param.dist_coeffs, None,
                                         new_camera_matrix)

        return undistorted_frame


class IPM:

    def __init__(self):
        self.src_point = np.float32([(0, 420), (640, 420), (470, 200), (157, 200)])  # 원본 이미지에서 움직이고 싶은 픽셀
        self.dst_point = np.float32([(175, 480), (465, 480), (640, 0), (0, 0)])  # bev 이미지에서 위치하고 싶은 픽셀

    def ipm_transform(self, frame):
        height, width = frame.shape[:2]

        ipm_matrix = cv.getPerspectiveTransform(self.src_point, self.dst_point)

        return cv.warpPerspective(frame, ipm_matrix, (width, height), flags=cv.INTER_LINEAR)

class IPM_POINT:

    def __init__(self):
        self.src_point = np.float32([(0, 420), (640, 420), (470, 200), (157, 200)])  # 원본 이미지에서 움직이고 싶은 픽셀
        self.dst_point = np.float32([(175, 480), (465, 480), (640, 0), (0, 0)])  # 변환된 이미지에서 위치하고 싶은 픽셀
        self.ipm_matrix = cv.getPerspectiveTransform(self.src_point, self.dst_point)

    def transform_points(self, points):
        transformed_points = cv.perspectiveTransform(points.reshape(-1,1,2), self.ipm_matrix)
        return np.round(transformed_points).astype(int)


class UNIT:

    def __init__(self):
        self.D2R = pi / 180
        self.R2D = 180 / pi
        self.KM2MS = 0.27777  # km/h to m/s

    def RAD2DEG(self, val):
        return val * self.R2D

    def DEG2RAD(self, val):
        return val * self.D2R


class morai_VideoCapture:
    def __init__(self, udp_cam):
        self.udp_cam = udp_cam

    def read(self):
        return self.udp_cam.is_img, self.udp_cam.raw_img

    def isOpened(self):
        return self.udp_cam.is_img
