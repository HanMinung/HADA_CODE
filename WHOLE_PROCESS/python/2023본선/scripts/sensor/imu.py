
from multiprocessing import shared_memory
from winreg import KEY_ALL_ACCESS
import numpy as np
import ctypes as ct
import ctypes.wintypes as wt
import serial
import struct
import threading
from math import pi
import time
#IMU REAL TIME PLOT


# class IMU_Parser():
#     def __init__(self,port , init_heading ):
#         self.ser = serial.Serial(port, 9600)
#         self.init_heading = init_heading  # 시작 초기 절대헤딩
#         self.yaw = 0
#         self.raw_data = []
#         thread = threading.Thread(target=self.recv_imu_data)
#         thread.daemon = True 
#         thread.start() 
        
#     def recv_imu_data(self):
#         while True:
#             if self.ser.in_waiting >= 28: # Check if there's enough data available to read (assuming 7 floats)
#                 data = self.ser.read(28) # Read the data (28 bytes for 7 floats)
#                 sensorData = struct.unpack('fffffff', data) # Unpack the data as 7 floats
#                 sensorList = list(sensorData) # Convert the tuple to a list
#                 # print("Received sensor data:", sensorList) # Print the received sensor data
#                 yaw = sensorList[2]
#                 self.raw_data = sensorList
#                 if yaw > 180.0:
#                     yaw =  yaw - 360
                    
#                 self.yaw = yaw + self.init_heading 
            
#     def __del__(self):
#         self.ser.close()
import numpy as np
import ctypes as ct
import ctypes.wintypes as wt
from math import pi

#IMU REAL TIME PLOT


# 받기용 구조체  
class READ_DATA(ct.Structure):
    _fields_ = [ ("SIM_count", ct.c_int), 
                 ("SIM_Time", ct.c_double),
                 ("euler_x", ct.c_double),
                 ("euler_y", ct.c_double),
                 ("euler_z", ct.c_double),
                 ("acc_x", ct.c_double),
                 ("acc_y", ct.c_double),
                 ("acc_z", ct.c_double),
                 ("vel_x", ct.c_double),
                 ("vel_y", ct.c_double),
                 ("vel_z", ct.c_double),
                 ("rot_x", ct.c_double),
                 ("rot_y", ct.c_double),
                 ("rot_z", ct.c_double)]

     
class IMU_SM :
    
    # ============================================================================
    # Communication Variable Definition
    # ============================================================================
    def __init__(self):
        self.is_sharedmemory = False



    # ============================================================================
    # SharedMemory Open (파이썬과 C 코드 연결을 위함)
    # ============================================================================
    def sharedmemory_open(self):
        # Shared Memory Process
        self.FILE_MAP_ALL_ACCESS  = 0x000F001F
        self.FILE_MAP_READ        = 0x0004
        self.INVALID_HANDLE_VALUE = -1
        self.SHMEMSIZE            = 0x100
        self.PAGE_READWRITE       = 0x04
        self.TRUE  = 1
        self.FALSE = 0

        self.kernel32_dll               = ct.windll.kernel32
        self.msvcrt_dll                 = ct.cdll.msvcrt  # To be avoided

        self.CreateFileMapping          = self.kernel32_dll.CreateFileMappingW
        self.CreateFileMapping.argtypes = (wt.HANDLE, wt.LPVOID, wt.DWORD, wt.DWORD, wt.DWORD, wt.LPCWSTR)
        self.CreateFileMapping.restype  = wt.HANDLE

        self.OpenFileMapping            = self.kernel32_dll.OpenFileMappingW
        self.OpenFileMapping.argtypes   = (wt.DWORD, wt.BOOL, wt.LPCWSTR)
        self.OpenFileMapping.restype    = wt.HANDLE

        self.MapViewOfFile              = self.kernel32_dll.MapViewOfFile
        self.MapViewOfFile.argtypes     = (wt.HANDLE, wt.DWORD, wt.DWORD, wt.DWORD, ct.c_ulonglong)
        self.MapViewOfFile.restype      = wt.LPVOID

        self.memcpy                     = self.msvcrt_dll.memcpy
        self.memcpy.argtypes            = (ct.c_void_p, ct.c_void_p, ct.c_size_t)
        self.memcpy.restype             = wt.LPVOID

        self.UnmapViewOfFile            = self.kernel32_dll.UnmapViewOfFile
        self.UnmapViewOfFile.argtypes   = (wt.LPCVOID,)
        self.UnmapViewOfFile.restype    = wt.BOOL

        self.CloseHandle                = self.kernel32_dll.CloseHandle
        self.CloseHandle.argtypes       = (wt.HANDLE,)
        self.CloseHandle.restype        = wt.BOOL

        self.GetLastError               = self.kernel32_dll.GetLastError
        
        # 파일 이름 선언

        self.rfile_mapping_name_ptr = ct.c_wchar_p("Xsens_smdat_ReadData")

        # 파일 크기 선언
    
        self.rbyte_len = ct.sizeof(READ_DATA)    



        # r파일 맵핑 및 맵핑 객체 선언
        self.rmapping_handle = self.OpenFileMapping(self.FILE_MAP_ALL_ACCESS, False, self.rfile_mapping_name_ptr)
        if not self.rmapping_handle:
            print("Could not open file mapping object: {:d}".format(self.GetLastError()))
            raise ct.WinError()

        self.rmapped_view_ptr = self.MapViewOfFile(self.rmapping_handle, self.FILE_MAP_ALL_ACCESS, 0, 0, self.rbyte_len)
        if not self.rmapped_view_ptr:
            print("Could not map view of file: {:d}".format(self.GetLastError()))
            self.CloseHandle(self.rmapping_handle)
            raise ct.WinError()
        
        self.is_sharedmemory = True

        ## You can search it to find out the detailed structure of shared memory

    
    def recv_data(self):
        if self.is_sharedmemory==True:
            read_smdat = READ_DATA()
            rmsg_ptr   = ct.pointer(read_smdat)
            self.memcpy(rmsg_ptr,self.rmapped_view_ptr,self.rbyte_len)
            euler_z = (read_smdat.euler_z - 90 )                # 라디안
        return euler_z



    def sharedmemory_close(self):
        self.UnmapViewOfFile(self.rmapped_view_ptr)
        self.CloseHandle(self.rmapping_handle)


# if __name__ == "__main__":
    
#     imu = IMU_Parser("com20" , 0 )
#     # imu.recv_imu_data()
#     while True:
#         print(imu.raw_data)
#         time.sleep(0.05)



        