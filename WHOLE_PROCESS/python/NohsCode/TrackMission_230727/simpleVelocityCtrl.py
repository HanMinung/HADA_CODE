import numpy as np
import time 
import math
import ctypes as ct
import ctypes.wintypes as wt
from drawnow import *
from math import sin, cos,asin,sqrt,atan
from sklearn.cluster import DBSCAN
from serial_node import erpSerial


if __name__ == '__main__':
    device = 'com4'
    command = erpSerial(device)

    while True:
        command.recv_serial_data()
        ref_vel = 30
        Kp = 3
        err = ref_vel - command.speed 
        input = Kp * err
        command.send_ctrl_cmd(input , 0)

