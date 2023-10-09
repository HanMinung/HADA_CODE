"""

    - Code description   : 3D lidar Camera calibration verification
    - 실시간 calibration 결과 검증 시각화
    - 3D lidar interface : C++
    
"""

from variable import *

global pointCloud

pointCloud = []
    
class POINT_CLOUD(ct.Structure):
    _fields_ = [("xCoordinate",ct.c_double), ("yCoordinate",ct.c_double), ("zCoordinate",ct.c_double)]

# POINT_CLOUD READ_DATA[12000]    
class READ_DATA(ct.Array):
    
    _length_ = 30000
    _type_ = POINT_CLOUD
    

class Velodyne_Sharedmemory :
    
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

        self.rfile_mapping_name_ptr = ct.c_wchar_p("Lidar_smdat_velodyne")

        self.rbyte_len = ct.sizeof(READ_DATA)   

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
        
    def sharedmemory_close(self):
        
        self.UnmapViewOfFile(self.wmapped_view_ptr)
        self.CloseHandle(self.wmapping_handle)
        self.UnmapViewOfFile(self.rmapped_view_ptr)
        self.CloseHandle(self.rmapping_handle)
        
        
    def Importdata(self):
        
        global pointCloud
        pointCloud = READ_DATA()
        rmsg_ptr   = ct.pointer(pointCloud)
        self.memcpy(rmsg_ptr, self.rmapped_view_ptr, self.rbyte_len)
        
 

class Projection :
    
    def __init__(self) :
        
        self.projectionX = []
        self.projectionY = []
        
        self.cam_point   = [548, 326]

        
    def writeCSV(self, filename = 'PCDdata.csv') :
        
        with open(filename, 'w', newline='') as f :
            
            writer = csv.writer(f)
            writer.writerow(['xCoor', 'yCoor', 'zCoor'])
            
            for Idx in range(nLidar) :
                
                writer.writerow([pointCloud[Idx].xCoordinate, pointCloud[Idx].yCoordinate, pointCloud[Idx].zCoordinate])
        
    
    def projectPCD(self) :
        
        self.projectionX = []
        self.projectionY = []
        
        for Idx in range(nLidar) :
            
            if pointCloud[Idx].xCoordinate == 0 and pointCloud[Idx].yCoordinate == 0 and pointCloud[Idx].zCoordinate : break

            pointX = pointCloud[Idx].xCoordinate
            pointY = pointCloud[Idx].yCoordinate
            pointZ = pointCloud[Idx].zCoordinate
            
            scaleFac = pointY + camRecede

            worldCoor   = np.array([[pointX],[pointY],[pointZ],[1]])
            pixelCoor   = 1/scaleFac * intMat @ extMat @ worldCoor

            pixelU = int(pixelCoor[0])
            pixelV = int(pixelCoor[1])

            if pixelU >= 0 and pixelU <= imgWidth and pixelV >= 0 and pixelV < imgHeight :
                
                self.projectionX.append(pixelU)
                self.projectionY.append(pixelV)

        # drawnow.drawnow(self.plotProjection)


    def plotProjection(self) :  
        
        plt.plot(self.cam_point[0], self.cam_point[1], 'r.', markersize = 12)
        # plt.plot(self.cam_point[2], self.cam_point[3], 'r.', markersize = 12)
        # plt.plot(self.cam_point[4], self.cam_point[5], 'r.', markersize = 12)
        plt.plot(self.projectionX, self.projectionY, 'b.')
        plt.xlabel('pixel X')
        plt.ylabel('pixel Y')
        plt.xlim([0, 640])
        plt.ylim([0, 480])
        plt.grid(True)
        plt.tick_params(axis = 'both', labelsize=7)
        

    
if __name__ == "__main__" :
    
    sharedMem  = Velodyne_Sharedmemory()
    projection = Projection()
    
    sharedMem.Lidar_SMopen()
    
    time_start = time.time()
    
    while (time_stime < time_final):
    
        loopStart = time.time()
                
        sharedMem.Importdata()
        
        projection.projectPCD()
        
        loopEnd = time.time()
        
        if loopEnd - loopStart != 0 :
            
            print(f"Loop freq : {round(1/(loopEnd - loopStart))} [HZ]")
        
        while(1):
            
            time_curr = time.time()
            
            time_del = time_curr - time_start - time_stime
            
            if (time_del > time_ts) :
                
                time_cnt += 1
                time_stime = time_cnt*time_ts
                
                break
            

    sharedMem.sharedmemory_close()
    
    