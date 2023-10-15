import numpy as np
import ctypes as ct
import ctypes.wintypes as wt
from math import pi


class GPS_REAL_READ_DATA(ct.Structure):
    _fields_ = [ ("SIM_count", ct.c_int), 
                 ("LatDegree", ct.c_int),
                 ("LatMinute", ct.c_double),
                 ("LonDegree", ct.c_int),
                 ("LonMinute", ct.c_double),
                 ("FixQual"  , ct.c_char),
                 ("N_Sat"    , ct.c_int)]

class IMU_REAL_READ_DATA(ct.Structure):
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
    
class LIDAR_REAL_READ_DATA(ct.Structure):# For Real Lidar
    _fields_ = [("xCoordinate",ct.c_double*761),
                ("yCoordinate",ct.c_double*761),
                ("dist",ct.c_double*761),
                ("angle",ct.c_double*761)]
    
class MORAI_READ_DATA(ct.Structure):
    _fields_ = [("Roll",ct.c_double),
                ("Pitch",ct.c_double),
                ("Yaw",ct.c_double),
                ("Acc_x",ct.c_double),
                ("Acc_y",ct.c_double),
                ("Acc_z",ct.c_double),
                ("Latitude",ct.c_double),
                ("Longitude",ct.c_double),
                ("Distance",ct.c_double*761),
                ("Velocity",ct.c_double)]
    
class MORAI_WRITE_DATA(ct.Structure):   # Struct with Lidar
    _fields_ = [ ("Steering", ct.c_double), ("Velocity", ct.c_double),("Gear",ct.c_int),("Brake",ct.c_double)]    
    
class MORAI_SM:
    
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
        self.wfile_mapping_name_ptr = ct.c_wchar_p("ERP42_smdat_ReadData")
 
        # If MORAI
        self.rfile_mapping_name_ptr = ct.c_wchar_p("Hils_smdat_ReadData")

        self.wbyte_len = ct.sizeof(MORAI_WRITE_DATA)
        self.rbyte_len = ct.sizeof(MORAI_READ_DATA)    

        # w파일 맵핑 및 맵핑 객체 선언
        self.wmapping_handle = self.CreateFileMapping(self.INVALID_HANDLE_VALUE,0, self.PAGE_READWRITE, 0, self.wbyte_len, self.wfile_mapping_name_ptr)
        if not self.wmapping_handle:
            print("Could not open file mapping object: {:d}".format(self.GetLastError()))
            raise ct.WinError()

        self.wmapped_view_ptr = self.MapViewOfFile(self.wmapping_handle, self.FILE_MAP_ALL_ACCESS, 0, 0, self.wbyte_len)
        if not self.wmapped_view_ptr:
            print("Could not map view of file: {:d}".format(self.GetLastError()))
            self.CloseHandle(self.wmapping_handle)
            raise ct.WinError()


        # r파일 맵핑 및 맵핑 객체 선언
        self.rmapping_handle = self.OpenFileMapping(self.FILE_MAP_READ, False, self.rfile_mapping_name_ptr)
        if not self.rmapping_handle:
            print("Could not open file mapping object: {:d}".format(self.GetLastError()))
            raise ct.WinError()

        self.rmapped_view_ptr = self.MapViewOfFile(self.rmapping_handle, self.FILE_MAP_READ, 0, 0, self.rbyte_len)
        if not self.rmapped_view_ptr:
            print("Could not map view of file: {:d}".format(self.GetLastError()))
            self.CloseHandle(self.rmapping_handle)
            raise ct.WinError()
        
        self.is_sharedmemory = True

        ## You can search it to find out the detailed structure of shared memory

    def recv_data_MORAI(self):
        roll  = 0.0
        pitch = 0.0
        yaw   = 0.0
        acc_x = 0.0
        acc_y = 0.0
        acc_z = 0.0
        latitude  = 0.0
        longitude = 0.0
        distance = 0.0
        if self.is_sharedmemory==True:
            read_smdat = MORAI_READ_DATA()
            rmsg_ptr   = ct.pointer(read_smdat)
            self.memcpy(rmsg_ptr,self.rmapped_view_ptr,self.rbyte_len)
            roll           = read_smdat.Roll
            pitch          = read_smdat.Pitch
            yaw            = read_smdat.Yaw
            acc_x          = read_smdat.Acc_x
            acc_y          = read_smdat.Acc_y     
            acc_z          = read_smdat.Acc_z     
            latitude       = read_smdat.Latitude  
            longitude      = read_smdat.Longitude 
            distance       = read_smdat.Distance  
            for i in range(761):    # 정규화 (미터 단위로)
                if distance[i] >=5000:
                    distance[i] =5000
                
                distance[i] = distance[i]*0.002
        return latitude,longitude,yaw,distance
    
    def cmd_to_MORAI(self,gear,command_velocity,command_steering ,cmd_brake):
        write_smdat = MORAI_WRITE_DATA()
        write_smdat.Steering        = command_steering
        write_smdat.Velocity        = command_velocity
        write_smdat.Gear            = gear               # 0: 전진 1:중립 2:후진
        write_smdat.Brake           = cmd_brake
        wmsg_ptr                    = ct.pointer(write_smdat)
        self.memcpy(self.wmapped_view_ptr, wmsg_ptr, self.wbyte_len)
        

    def sharedmemory_close(self):
        self.UnmapViewOfFile(self.wmapped_view_ptr)
        self.CloseHandle(self.wmapping_handle)
        self.UnmapViewOfFile(self.rmapped_view_ptr)
        self.CloseHandle(self.rmapping_handle)



