
import numpy as np
import ctypes as ct
import ctypes.wintypes as wt
from math import pi

D2R=pi/180
R2D=180/pi
#GPS REAL TIME PLOT


# 받기용 구조체  
class READ_DATA(ct.Structure):
    _fields_ = [ ("SIM_count", ct.c_int), 
                 ("LatDegree", ct.c_int),
                 ("LatMinute", ct.c_double),
                 ("LonDegree", ct.c_int),
                 ("LonMinute", ct.c_double),
                 ("FixQual"  , ct.c_char),
                 ("N_Sat"    , ct.c_int)]

     
class GPS_SM :
    
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
        self.GPS_lat = 0   # UNIT: Degree
        self.GPS_lon = 0   # UNIT: Degree
        # 파일 이름 선언

        self.rfile_mapping_name_ptr = ct.c_wchar_p("NovATelGPS_smdat_ReadData")

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
            GPS_lat = (read_smdat.LatDegree +read_smdat.LatMinute/60)   # UNIT: Degree
            GPS_lon = (read_smdat.LonDegree +read_smdat.LonMinute/60)   # UNIT: Degree
            GPS_h   = 50.0                                              # UNIT: m로 추정..
            GPS_Qual= read_smdat.FixQual   
            if GPS_lon >120.0 and GPS_lon < 130:
                self.GPS_lat = GPS_lat
                self.GPS_lon = GPS_lon
        return self.GPS_lat,self.GPS_lon,GPS_h,GPS_Qual
        #GPSqual이 4,5가 될때까지 기다리는 코드 메인에서

    def sharedmemory_close(self):
        self.UnmapViewOfFile(self.rmapped_view_ptr)
        self.CloseHandle(self.rmapping_handle)
