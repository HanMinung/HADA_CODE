
from multiprocessing import shared_memory
from winreg import KEY_ALL_ACCESS
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
            euler_z = (read_smdat.euler_z -90 )                # 라디안
        return euler_z



    def sharedmemory_close(self):
        self.UnmapViewOfFile(self.rmapped_view_ptr)
        self.CloseHandle(self.rmapping_handle)

class Check_IMU:

    def stabilizeIMU(self):

        IMU_REAL=IMU_SM()
        IMU_REAL.sharedmemory_open()   
        yaw=IMU_REAL.recv_data()
        
        while(yaw==0.0):
            
            yaw=IMU_REAL.recv_data()
            
            if(input==KEY_ALL_ACCESS):
                IMU_flag = 0
                break

        return IMU_flag

# class Update:

#     def __init__(self,):
        
#         # self.
        

#     # def YawBias(self):






        