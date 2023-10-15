from module import *

global lidarDist, azimuth
lidarDist  = []
azimuth    = np.linspace(0 ,180 ,761)

unit       =   UNIT()
dynamic    =   DYNAMIC()
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
        
    def import_data(self):
        
        global lidarDist

        if self.is_lidarSM == True:
            
            read_smdat = READ_DATA()
            rmsg_ptr   = ct.pointer(read_smdat)
            self.memcpy(rmsg_ptr,self.rmapped_view_ptr,self.rbyte_len)
            lidarDist  = read_smdat.dist
            
    def sharedmemory_close(self):
        self.UnmapViewOfFile(self.wmapped_view_ptr)
        self.CloseHandle(self.wmapping_handle)
        self.UnmapViewOfFile(self.rmapped_view_ptr)
        self.CloseHandle(self.rmapping_handle)
        


class AVOIDANCE :
    
    def __init__(self) :
        
        self.xPos          = []
        self.yPos          = []
        
        self.clusterX      = [] 
        self.clusterY      = []
        
            
        
    def pre_processing(self, distance, azimuth) :
        
        self.xPos, self.yPos = [], []
        
        for Idx in range(dynamic.lidar_num) :
            
            if distance[Idx] <= 1e-2 : 
                
                distance[Idx] = dynamic.lidar_num * 500
                
            if distance[Idx] > dynamic.lidar_num * 500 and Idx != 0 :
                
                distance[Idx] = distance[Idx - 1]
            
            distance[Idx] = distance[Idx] / 500 
            
            if distance[Idx] >= 1e-2 and distance[Idx] <= 0.99 * dynamic.range_max :
                                   
                self.xPos.append(distance[Idx] * cos(azimuth[Idx] * unit.D2R))
                self.yPos.append(distance[Idx] * sin(azimuth[Idx] * unit.D2R))
                
            else : continue

            

    def dbScan(self) :
    
        self.clusterX, self.clusterY = [], []
        
        stack   = np.stack((self.xPos, self.yPos), axis = 1)
        
        if stack.shape[0] > 0:
            
            cluster = DBSCAN(eps = dynamic.epsilon, min_samples = dynamic.minSample).fit(stack)
        
            labels = cluster.labels_
        
            clusterN = max(labels)
            
            for Idx in range(clusterN) :
            
                selected_x = np.array(self.xPos)[np.array(labels) == Idx]
                selected_y = np.array(self.yPos)[np.array(labels) == Idx]

                self.clusterX.append(np.mean(selected_x))
                self.clusterY.append(np.mean(selected_y))
            

    def print_result(self, loop_time) :
        
        print(f"LOOP TIME : {round(loop_time, 2)}")


        
    def data_processing(self) :
        
        self.pre_processing(lidarDist, azimuth)         # lidar raw data preprocessing
        
        
        
        self.dbScan()                                   # clustering

        plt.cla()
        plt.plot(self.clusterX  ,   self.clusterY   ,   'bo'    ,   markersize = 6)
        plt.pause(0.001)


if __name__ == "__main__" :
    
    lidar      =  lidarSM()
    project    =  AVOIDANCE()
    
    lidar.Lidar_SMopen()
    
    time_start = time.time()
    
    while (time_stime < realtime.time_final):
        
        loopStart = time.time()
        
        lidar.import_data()
        
        project.data_processing()

        loopEnd = time.time()
        
        project.print_result(loopEnd - loopStart)

        while(1):
            
            time_curr = time.time()
            time_del = time_curr - time_start - time_stime
            
            if (time_del > realtime.time_ts) :
                
                realtime.time_cnt += 1
                time_stime = realtime.time_cnt * realtime.time_ts
                
                break
            

    lidar.sharedmemory_close()