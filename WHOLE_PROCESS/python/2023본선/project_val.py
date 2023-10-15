# 주행 모드
MOD_MORAI: int = 0x00
MOD_REAL: int = 0x01

# 미션 종류
MISSION_DELIVERY_A: int = 0x10  # 배달
MISSION_DELIVERY_B: int = 0x11  # 배달
MISSION_STATIC: int = 0x12  # 정적 장애물
MISSION_STRAIGHT: int = 0x13  # 직진
MISSION_TURNLEFT4: int = 0x14 # 4구 좌회전
MISSION_PARKING: int = 0x15  # 평행 주차
MISSION_NONE: int = 0x16
MISSION_TURNLEFT3: int = 0x17 # 3구 좌회전
MISSION_TURNRIGHT:int =0x18

# 카메라 -> main
CAM2MAIN_NONE: int = 0x20
CAM2MAIN_BRAKE: int = 0x21

# 라이다 -> main
LIDAR2MAIN_NONE: int = 0x30
LIDAR2MAIN_BRAKE: int = 0x31

# 배달 -> 카메라
DELIV_A_START: int = 0x40
DELIV_A_FINISH: int = 0x41
DELIV_B_START: int = 0x42
DELIV_B_FINISH: int = 0x43

# 좌표계 원점
INIT_LAT: float = 37.2
INIT_LON: float = 126.7

INIT_HEADING_AT_KCITY = 26.5 # 대회장 초기헤딩

# sm memory 이름
CAM_TO_DELIVERY:str = 'CAM_TO_DELIVERY'
DELIVERY_TO_CAM:str = 'DELIVERY_TO_CAM'
MAIN_TO_TRAFFIC:str = 'MAIN_TO_TRAFFIC'


LANE_GAMMA_TO_MAIN:str = 'LANE_GAMMA_TO_MAIN'
LANE_VELOCITY_TO_MAIN:str = 'LANE_VELOCITY_TO_MAIN'

RUBBER_TO_STATIC:str = 'RUBBER_TO_STATIC'
STATIC_FALG_TO_LANE: str = 'STATIC_FALG_TO_LANE'
STATIC_STEER_TO_LANE: str = 'STATIC_STEER_TO_LANE'


def print_flag(flag: int):
    match flag:
        case 0x00:
            print("Mod MORAI")
        case 0x01:
            print("Mod Real")
        case 0x10:
            print("Mission Delibery A")
        case 0x11:
            print("Mission Delibery B")

