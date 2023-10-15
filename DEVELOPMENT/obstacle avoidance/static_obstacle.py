from module import *
 
global lidarDist, azimuth
lidarDist  =   []
azimuth    =   np.linspace(-95 ,95 ,761)
 
unit       =   UNIT()
dynamic    =   DYNAMIC()
static     =   STATIC()
periph     =   PERIPHERAL()
realtime   =   REALTIME()
platform   =   ERP42() 


class READ_DATA(ct.Structure):
    
    _fields_ = [("xCoordinate",ct.c_double*761),("yCoordinate",ct.c_double*761),("dist",ct.c_double*761),("angle",ct.c_double*761)]
         
class lidarSM :
    
    def __init__(self):
        
        self.is_lidarSM = False
 
    def Lidar_SMopen(self) :
        
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
 
        self.rfile_mapping_name_ptr = ct.c_wchar_p("Lidar_smdat_ReadData")
 
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
        
        self.is_lidarSM = True
        
        print("Shared memory with lidar Interface program opened ...!")
        
    def importData(self):
        
        global lidarDist
 
        if self.is_lidarSM == True:
            
            read_smdat = READ_DATA()
            rmsg_ptr   = ct.pointer(read_smdat)
            self.memcpy(rmsg_ptr,self.rmapped_view_ptr,self.rbyte_len)
            lidarDist = read_smdat.dist[::-1]
            
    def sharedmemory_close(self):
        self.UnmapViewOfFile(self.wmapped_view_ptr)
        self.CloseHandle(self.wmapping_handle)
        self.UnmapViewOfFile(self.rmapped_view_ptr)
        self.CloseHandle(self.rmapping_handle)
 
class AVOIDANCE :
    
    def __init__(self) :
        
        self.dist_array       = np.zeros(static.lidar_num)
        
        self.array_OFF        = np.zeros(static.lidar_num)
        self.array_OFF_expand = np.zeros(static.lidar_num)
        self.array_SFF        = np.zeros(static.lidar_num)
        self.array_IFF        = np.zeros(static.lidar_num)
    
    
    def dataProcessing(self) :
        
        self.preProcessing(lidarDist)         # lidar raw data pre-processing
        
        self.generate_OFF_EXP()
        
        self.generate_SFF()
        
        self.generate_IFF()
        
        drawnow.drawnow(self.plotResult)
          
        
        
    def preProcessing(self, dist) :
        
        for Idx in range(static.lidar_num) :
            
            if Idx == 0 :
                
                if dist[Idx] == 0.0 : self.array_OFF[Idx] = 1.0
                
            else : 
                
                if dist[Idx] == 0.0 : self.array_OFF[Idx] = self.array_OFF[Idx-1]
            
            if dist[Idx] >= (static.range_max * 500) or dist[Idx] <= (static.range_min * 500) :
                
                self.array_OFF[Idx] = 1
                
            else : self.array_OFF[Idx] = dist[Idx] / (static.range_max * 500)

                    
    
        
    def generate_OFF_EXP(self) :
        
        for Idx in range(static.lidar_num) :
            
            r_i      = self.array_OFF[Idx]
            theta_i  = azimuth[Idx]                        # unit : degree
            k_local  = static.expand_r
            tmp      = static.expand_r / r_i
            
            if (tmp > 1.0) : 
                
                tmp = 1.0
                k_local = r_i
            
            theta = round((-unit.RAD2DEG(asin(tmp)) + theta_i) * 4 + 0.5) / 4
            
            while (theta < unit.RAD2DEG(asin(tmp)) + theta_i) :
                
                Index = (theta * 4) + 380
                
                if (Index >= 0 and Index < static.lidar_num) :
                    
                    D = unit.power(r_i) * unit.power(cos((unit.DEG2RAD(azimuth[Index])) - unit.DEG2RAD(theta_i))) - (unit.power(r_i) - unit.power(k_local))
                    
                    if (D < 0.0) : D = 0.0
                
                    r_tmp = r_i * cos(unit.DEG2RAD(azimuth[Index]) - unit.DEG2RAD(theta_i)) - sqrt(D)
                    
                    if(r_tmp < 0.0) : r_tmp = 0.0

                    if(r_tmp <= self.array_OFF[Index]) :
                        
                        self.array_OFF_expand[Index] = r_tmp
                
                theta = theta + 0.25
            
            
            
 
    def generate_SFF(self) :
        
        denom = 2 * unit.power(static.sigSFF)
        
        for Idx in range(static.lidar_num) :
            
            nom = -1.0 * unit.power((azimuth[Idx] - (static.deltaF)))                 # deltaF : 가이던스 산출 조향각
            
            self.array_SFF[Idx] = exp(nom / denom)
        
 
 
    def generate_IFF(self) :
        
        self.array_IFF = static.IFF_weight * self.array_OFF_expand + (1 - static.IFF_weight) * self.array_SFF
 
 
 
    def commandERP(self) :
        
        velocity = 0
        brake    = 0
        
        maxIdx   = np.argmax(self.array_IFF)
        steering = azimuth[maxIdx]
        
        return velocity, steering, brake
    
 
    def plotResult(self) :
        
        # plt.plot(azim, self.array_SFF, 'b-', markersize = 6)
        plt.plot(azimuth, self.array_OFF, 'b-', markersize = 6)
        # plt.plot(azim, self.array_OFF_expand, 'r-', markersize = 6)
        # plt.plot(azim, self.array_IFF, 'g-', markersize = 6)
        plt.xlabel('x coor [m]', fontsize = periph.fontSize)
        plt.ylabel('y coor [m]', fontsize = periph.fontSize)
        plt.xlim([-95, 95])
        plt.ylim([0, 1.5])
        plt.grid(True)  
        plt.tick_params(axis='both', labelsize=12)
       



 
if __name__ == "__main__" :
    
    lidar      =  lidarSM()
    project    =  AVOIDANCE()
        
    lidar.Lidar_SMopen()
    
    time_start = time.time()
    
    while (time_stime < realtime.time_final):
        
        loopStart = time.time()
        
        lidar.importData()
        
        project.dataProcessing()
        
        vel, steer, braek = project.commandERP()
        
        loopEnd = time.time()
        
        print(f"STEER : {steer}")
        print(f"LOOP TIME : {round(loopEnd - loopStart, 3)} [sec]")
                    
        while(1):
            
            time_curr = time.time()
            time_del = time_curr - time_start - time_stime
            
            if (time_del > realtime.time_ts) :
                
                realtime.time_cnt += 1
                time_stime = realtime.time_cnt * realtime.time_ts
                
                break
            
 
    lidar.sharedmemory_close()