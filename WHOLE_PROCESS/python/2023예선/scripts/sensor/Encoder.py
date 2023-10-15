import sys
import ctypes as ct
import ctypes.wintypes as wt
import msvcrt

from time import *
from ctypes import *
from ctypes.wintypes import *


class c2python(ct.Structure):         
    _fields_=[("Delcmd",ct.c_double),("Velocity",ct.c_double),("Brake",ct.c_int),("Gear",ct.c_int)]

c2python_smdat = c2python()            

class Python2C(ct.Structure):
        _fields_=[("Delcmd",ct.c_double),("Velcmd",ct.c_double),("Brakecmd",ct.c_int),("Gearcmd",ct.c_int)]
python2C_smdat = Python2C()


class EncoderSM:
    
    def __init__(self):

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

        ######################################################################################################################
        self.mapping_name_ptr_C2python = ct.c_wchar_p("c2python")
        self.msg_ptr_C2python = pointer(c2python_smdat)      
        self.byte_len_C2python = ct.sizeof(c2python)

        self.file_mapping_name_ptr_python2C = ct.c_wchar_p("python2c")
        self.msg_ptr_python2C = pointer(python2C_smdat)
        self.byte_len_python2C = ct.sizeof(Python2C)
        ######################################################################################################################
        # mapping_handle_C2python = CreateFileMapping(INVALID_HANDLE_VALUE, 0, PAGE_READWRITE, 0, byte_len_C2python, mapping_name_ptr_C2python)

        # print("Mapping object handle: 0x{:016X}".format(mapping_handle_C2python))
        # if not mapping_handle_C2python:
        #     print("Could not open file mapping object: {:d}".format(GetLastError()))
        #     raise ct.WinError()
        
        self.mapping_handle_C2python = self.OpenFileMapping(self.FILE_MAP_READ, self.FALSE,  self.mapping_name_ptr_C2python)

        print("Mapping object handle: 0x{:016X}".format(self.mapping_handle_C2python))
        if not self.mapping_handle_C2python:
            print("Could not open file mapping object: {:d}".format(self.GetLastError()))
            raise ct.WinError()

        self.mapped_view_ptr_C2python = self.MapViewOfFile(self.mapping_handle_C2python, self.FILE_MAP_READ, 0, 0, self.byte_len_C2python)

        print("Mapped view addr: 0x{:016X}".format(self.mapped_view_ptr_C2python))
        if not self.mapped_view_ptr_C2python:
            print("Could not map view of file: {:d}".format(self.GetLastError()))
            self.CloseHandle(self.mapping_handle_C2python)
            raise ct.WinError()
        print("Message size ({:d} bytes)".format(self.byte_len_C2python))
        
        self.memcpy(self.msg_ptr_C2python,self.mapped_view_ptr_C2python,self.byte_len_C2python)
        #######################################################################################################################

        self.mapping_handle_python2C = self.CreateFileMapping(self.INVALID_HANDLE_VALUE, 0, self.PAGE_READWRITE, 0, self.byte_len_python2C, self.file_mapping_name_ptr_python2C)

        print("Mapping object handle: 0x{:016X}".format(self.mapping_handle_python2C))
        if not self.mapping_handle_python2C:
            print("Could not open file mapping object: {:d}".format(self.GetLastError()))
            raise ct.WinError()

        self.mapped_view_ptr_python2C = self.MapViewOfFile(self.mapping_handle_python2C, self.FILE_MAP_ALL_ACCESS, 0, 0, self.byte_len_python2C)

        print("Mapped view addr: 0x{:016X}".format(self.mapped_view_ptr_python2C))
        if not self.mapped_view_ptr_python2C:
            print("Could not map view of file: {:d}".format(self.GetLastError()))
            self.CloseHandle(self.mapping_handle_python2C)
            raise ct.WinError()
        print("Message size ({:d} bytes)".format(self.byte_len_python2C))

        self.memcpy(self.mapped_view_ptr_python2C, self.msg_ptr_python2C, self.byte_len_python2C)  
        ########################################################################################################################
    
    def SMupdate(self):
        self.memcpy(self.msg_ptr_C2python,self.mapped_view_ptr_C2python,self.byte_len_C2python)
        self.memcpy(self.mapped_view_ptr_python2C, self.msg_ptr_python2C, self.byte_len_python2C)
        
    def recv_velocity(self):
        self.memcpy(self.msg_ptr_C2python,self.mapped_view_ptr_C2python,self.byte_len_C2python)
        vel = c2python_smdat.Velocity
        return vel
    
    def send_ctrl_cmd(self , gear , steer,vel,brake):
        python2C_smdat.Delcmd=steer   # [deg] Python2C test_smdat 변수 선언
        python2C_smdat.Velcmd=vel   # Python2C test_smdat 변수 선언
        python2C_smdat.Brakecmd=brake # Python2C test_smdat 변수 선언
        python2C_smdat.Gearcmd=gear  # Python2C test_smdat 변수 선언
        self.memcpy(self.mapped_view_ptr_python2C, self.msg_ptr_python2C, self.byte_len_python2C)
        
    def closeSM(self):
        self.UnmapViewOfFile(self.mapped_view_ptr_C2python)
        self.CloseHandle(self.mapping_handle_C2python)
        self.UnmapViewOfFile(self.mapped_view_ptr_python2C)
        self.CloseHandle(self.mapping_handle_python2C)

if __name__ == "__main__":
   
     
    n=0
    
    
    ERP=EncoderSM()

    while True:
        # ERP=EncoderSM()
        ERP.SMupdate()
        print(c2python_smdat.Delcmd)
        print(c2python_smdat.Velocity)
        python2C_smdat.Delcmd=28   # Python2C test_smdat 변수 선언
        python2C_smdat.Velcmd=0   # Python2C test_smdat 변수 선언
        python2C_smdat.Brakecmd=0 # Python2C test_smdat 변수 선언
        python2C_smdat.Gearcmd=0  # Python2C test_smdat 변수 선언
        n+=1
        # sleep(0.8)
        # if n==1000:
        #     break

    ERP.closeSM()
