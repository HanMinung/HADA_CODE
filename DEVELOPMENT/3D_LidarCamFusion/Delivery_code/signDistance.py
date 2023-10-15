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
    
    _length_ = 12000
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
        
        
    def Import_data(self):
        
        global pointCloud
        pointCloud = READ_DATA()
        rmsg_ptr   = ct.pointer(pointCloud)
        self.memcpy(rmsg_ptr, self.rmapped_view_ptr, self.rbyte_len)
        
 

class Projection :
    
    def __init__(self) :
        
        self.pixel_U       = []                      # stores lidar point cloud data projected on to pixel coordinate
        self.pixel_V       = []
        
        self.Index         = 0
        self.dist          = 1000                    # distance to sign (initial value setting)
        self.prev_dist     = 1000
        
        self.cam_center    = []
        self.pixel_diff    = []
        self.pixel_select  = []
        
        self.is_stop       = 0
        self.steer         = 0
        self.brake         = 0
        
        self.restart       = 0
        self.cnt           = 0

        
    
    def project_PCD(self) :       
        
        self.pixel_diff    =  []
        self.pixel_U       =  []
        self.pixel_V       =  []
        self.pixel_select  =  []
        
        if len(self.cam_center) != 0 :      
        
            for Idx in range(nLidar) :                  # for loop (project point cloud data to pixel coordinate)
                
                if pointCloud[Idx].xCoordinate == 0 and pointCloud[Idx].yCoordinate == 0 and pointCloud[Idx].zCoordinate == 0 : break

                pointX = pointCloud[Idx].xCoordinate
                pointY = pointCloud[Idx].yCoordinate
                pointZ = pointCloud[Idx].zCoordinate
                
                scaleFac = pointY + camRecede           # scaling factor for projection

                worldCoor   = np.array([[pointX],[pointY],[pointZ],[1]])        # world coordinate matrix
                pixelCoor   = 1/scaleFac * intMat @ extMat @ worldCoor          # matrix calculation (3D --> 2D)

                self.pixel_U.append(int(pixelCoor[0]))                          # pixel x data appending
                self.pixel_V.append(int(pixelCoor[1]))                          # pixel y data appending
                
                x_diff = abs(self.pixel_U[Idx] - self.cam_center[0])            # calculation of x axis difference (bounding box center <--> projected point)
                y_diff = abs(self.pixel_V[Idx] - self.cam_center[1])            
                
                self.pixel_diff.append(x_diff + y_diff)                         # 가장 이상적인 포인트 검출을 위한 cost function...?
                
            self.Index = np.argmin(self.pixel_diff)                             # select minimum diffence value index
            
            self.pixel_select.append(self.pixel_U[self.Index])                  # 그때의 pixel 값들을 저장하나, 시각화를 위한 과정이지 사실 필요없음
            self.pixel_select.append(self.pixel_V[self.Index])
            
            # calculate distance to the sign
            self.dist = sqrt((pointCloud[self.Index].xCoordinate) **2 + (pointCloud[self.Index].yCoordinate) **2)
            
            self.prev_dist = self.dist              # Update
            
        else : self.dist = self.prev_dist

        self.flag_decision()

        # drawnow.drawnow(self.plotProjection)


    # receive object detection information (bounding box center)
    def receive_cam(self, shared_mem_name):
        
        self.cam_center = []
        
        try :
            
            shm = shared_memory.SharedMemory(name = shared_mem_name)
            shared_data = np.ndarray((9, ), dtype ='int', buffer = shm.buf) 
            
            self.cam_center.append(shared_data[1])
            self.cam_center.append(shared_data[2])
                
        except FileNotFoundError :
            
            print("cannot read sm data from 'CAMERA'...!")


    def Export_data(self) :
        
        self.sendto_cam(shared_memory_send_to_cam)          # send stop flag to cam
        
        self.sendto_main(shared_memory_send_to_main)        # send stop flag to main



    # send stop flag to camera process
    def sendto_cam(self, shared_mem_name) :
        
        try :
            
            shm = shared_memory.SharedMemory(name = shared_mem_name)
            shared_data = np.ndarray((1,), dtype ='int', buffer = shm.buf)

            shared_data[0] = self.is_stop
        
        except FileNotFoundError :
            
            print("cannot send sm data to 'CAMERA'")
        

    # send stop flag to main process (main : control platform)
    def sendto_main(self, shared_mem_name) :
        
        try : 
            
            shm = shared_memory.SharedMemory(name = shared_mem_name)
            shared_data = np.ndarray((1,), dtype ='int', buffer = shm.buf)

            shared_data[0] = self.is_stop
            
        except FileNotFoundError :
            
            print("cannot send sm data to 'MAIN'")



    def flag_decision(self) :
        
        if self.dist < dist_thresh :
            
            self.is_stop    = 1
            self.dist       = 1000
            self.prev_dist  = 1000
            
        if self.is_stop :
            
            self.check_restart()



    def check_restart(self) :
        
        self.restart += 1
            
        if(self.restart == 50) :                    # 5 seconds
                
            self.is_stop, self.restart   = 0, 0
            self.dist  , self.prev_dist = 1000, 1000



    def printResult(self, loop_time) :
        
        self.cnt += 1
        
        if self.cnt%5 == 0 :
            
            print("--------------------------------------------------")
            print(f"|       SIGN DIST     : {round(self.dist      , 2)}   [m]   ")
            # print(f"|       PREV DIST     : {round(self.prev_dist , 2)}   [m]   ")
            print(f"|       LOOP TIME     : {round(loop_time      , 2)}   [sec] ")
            print(f"|       IS STOP(1/0)  : {self.is_stop}")
            print(f"|       RESTART FLAG  : {self.restart}")
            print("--------------------------------------------------\n")



    def plotProjection(self) :  
        
        plt.plot(self.pixel_U          , self.pixel_V           , 'b.')
        plt.plot(self.pixel_select[0] , self.pixel_select[1]    , 'r.', markersize = 20)
        plt.plot(self.cam_center[0]       , self.cam_center[1]  , 'g.', markersize = 10)
        plt.xlabel('pixel X')
        plt.ylabel('pixel Y')
        plt.xlim([0, 640])
        plt.ylim([0, 480])
        plt.grid(True)
        plt.tick_params(axis = 'both', labelsize = 7)
        
        

    
if __name__ == "__main__" :
    
    sharedMem  = Velodyne_Sharedmemory()
    projection = Projection()
    
    sharedMem.Lidar_SMopen()
    shm_to_cam  = shared_memory.SharedMemory(name = shared_memory_send_to_cam , create = True, size = send_to_cam_mem_size)
    shm_to_main = shared_memory.SharedMemory(name = shared_memory_send_to_main, create = True, size = send_to_main_mem_size) 

    print("Shared memory with camera is opened ...!")
    
    time_start = time.time()
    
    while (time_stime < time_final):
    
        loopStart = time.time()
                
        sharedMem.Import_data()
        
        projection.receive_cam(shared_memory_receive_from_cam)
        
        projection.project_PCD()
        
        projection.Export_data()
        
        loopEnd = time.time()
        
        projection.printResult(loopEnd - loopStart)
        
        while(1):
            
            time_curr = time.time()
            
            time_del = time_curr - time_start - time_stime
            
            if (time_del > time_ts_main) :
                
                time_cnt += 1
                time_stime = time_cnt*time_ts_main
                
                break
            

    sharedMem.sharedmemory_close()
    shm_to_cam.close()