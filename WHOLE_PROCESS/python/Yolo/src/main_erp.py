# from serial_node import *
from yolov8_global import *

if __name__ == '__main__':

    sm_name_gamma_recv: str = "LANE_GAMMA_TO_MAIN"
    sm_name_velocity_recv: str = "LANE_VELOCITY_TO_MAIN"
    sm_name_stop_recv: str = "LANE_STOP_TO_MAIN"

    # port_num = 'COM4'
    # cmd = erpSerial(port_num)

    erp_gamma: int = 0
    erp_vel: int = 0
    erp_brake: int = 0

    print_cnt: int = 0

    while True:
        erp_gamma = recv_data(sm_name_gamma_recv, 1)
        erp_vel = recv_data(sm_name_velocity_recv, 1)
        erp_brake = recv_data(sm_name_stop_recv, 1)

        sendCommand.send_ctrl_cmd(erp_vel, erp_gamma, erp_brake)

        if print_cnt % 10 == 0:
            print(f'GAMMA       : {erp_gamma}')
            print(f'VELOCITY    : {erp_vel}')
            print(f'BRAKE       : {erp_brake}')
