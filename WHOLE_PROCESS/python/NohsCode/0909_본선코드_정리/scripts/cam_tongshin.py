'''
공유 메모리 함수
'''
from multiprocessing import shared_memory
import numpy as np
from project_val import *

def open_send_shm(_shm_name: str, shm_size: int):
    print('--------------------------------------')
    print(_shm_name, " start...")
    shm = shared_memory.SharedMemory(name=_shm_name, create=True, size=shm_size)
    print("send sm memory is open...")
    print('--------------------------------------')
    return shm


# SM memory
def send_data(shm, shape: [int, int], _data:list):
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
    
class Camera_IO:
    def __init__(self):
        self.sm_name_send: str = "MAIN_TO_TRAFFIC"
        self.sm_name_recv: str = "TRAFFIC_TO_MAIN"
        self.shm_send_size: int = 4  
        self.shm_send = open_send_shm(self.sm_name_send, self.shm_send_size)
        
    def send_flag(self, flag): # project val 참고하여 수정 필요
        send_data(self.shm_send, 1, [flag])

    def recv_flag(self):
        cam_flag = recv_data(self.sm_name_recv, 1)   
        return cam_flag
    
    def recv_lane_cmd(self): # 차선유지 조향,속도명령을 카메라에서 수신
        cam_lane_vel = recv_data( LANE_VELOCITY_TO_MAIN , 1)  
        cam_lane_gam = recv_data( LANE_GAMMA_TO_MAIN , 1)  
        return cam_lane_gam, cam_lane_vel
      
    def __del__(self):
        close_shm(self.sm_name_send)
        close_shm(self.sm_name_recv)
            
'''
sm_name_send: str = "TRAFFIC_TO_MAIN"
sm_name_recv: str = "MAIN_TO_TRAFFIC"
        
시작할때 한번만
shm_send_size: int = 4  
shm_send = open_send_shm(sm_name_send, shm_send_size)


반복문 안에서 원할떄 
self.shm_stop_flag = 1
send_data(shm_send, 1, [self.shm_stop_flag])

데이터 수신
self.current_sign_info = recv_data(sm_name_recv, 1)
'''