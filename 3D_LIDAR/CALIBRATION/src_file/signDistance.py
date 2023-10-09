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
        
        
    def Importdata(self):
        
        global pointCloud
        pointCloud = READ_DATA()
        rmsg_ptr   = ct.pointer(pointCloud)
        self.memcpy(rmsg_ptr, self.rmapped_view_ptr, self.rbyte_len)
        
 

class Projection :
    
    def __init__(self) :
        
        self.projectionX  = []
        self.projectionY  = []
        
        self.pixelU       = []
        self.pixelV       = []
        
        self.dist         = 1000
        self.prev_dist    = 1000
        
        self.camMid       = []
        self.pixel_diff   = []
        self.pixel_select = []
        
        self.isStop       = 0
        self.steer        = 0
        self.brake        = 0
        
        self.restart      = 0
        self.cnt          = 0

        
    def writeCSV(self, filename = 'PCDdata.csv') :
        
        with open(filename, 'w', newline='') as f :
            
            writer = csv.writer(f)
            writer.writerow(['xCoor', 'yCoor', 'zCoor'])
            
            for Idx in range(nLidar) :
                
                writer.writerow([pointCloud[Idx].xCoordinate, pointCloud[Idx].yCoordinate, pointCloud[Idx].zCoordinate])
        
    
    
    def projectPCD(self) :       
        
        self.pixel_diff   = []
        self.pixelU       = []
        self.pixelV       = []
        self.pixel_select = []
        
        if self.camMid[1] != 0 :
        
            for Idx in range(nLidar) :
                
                if pointCloud[Idx].xCoordinate == 0 and pointCloud[Idx].yCoordinate == 0 and pointCloud[Idx].zCoordinate : break

                pointX = pointCloud[Idx].xCoordinate
                pointY = pointCloud[Idx].yCoordinate
                pointZ = pointCloud[Idx].zCoordinate
                
                scaleFac = pointY + camRecede

                worldCoor   = np.array([[pointX],[pointY],[pointZ],[1]])
                pixelCoor   = 1/scaleFac * intMat @ extMat @ worldCoor

                self.pixelU.append(int(pixelCoor[0]))
                self.pixelV.append(int(pixelCoor[1]))
                
                x_diff = abs(self.pixelU[Idx] - self.camMid[0]) 
                y_diff = abs(self.pixelV[Idx] - self.camMid[1])
                
                self.pixel_diff.append(x_diff + y_diff)
            
            Index = np.argmin(self.pixel_diff)
            
            self.pixel_select.append(self.pixelU[Index])
            self.pixel_select.append(self.pixelV[Index])
            
            self.dist = sqrt((pointCloud[Index].xCoordinate) **2 + (pointCloud[Index].yCoordinate) **2)
            
            self.prev_dist = self.dist              # Update
            
            
        else : self.dist = self.prev_dist
            
        velocity, steer, brake = self.command_erp()

        # drawnow.drawnow(self.plotProjection)

        return velocity, steer, brake

        
    def receiveObject(self, shared_mem_name):
        
        try :
            
            self.camMid = []
            
            shm = shared_memory.SharedMemory(name = shared_mem_name)
            shared_data = np.ndarray((9, ), dtype ='int', buffer = shm.buf) 
            
            self.camMid.append(shared_data[1])
            self.camMid.append(shared_data[2])
            
        except FileNotFoundError :
            
            print("cannot read camera shared memory data...!")


    def sendtoCam(self, shared_mem_name) :
        
        try :
            
            shm = shared_memory.SharedMemory(name = shared_mem_name)
            shared_data = np.ndarray((1,), dtype ='int', buffer = shm.buf)

            shared_data[0] = self.isStop
        
        except FileNotFoundError :
            
            print("cannot send stop flag to camera process...!")
        


    def command_erp(self) :
        
        if self.dist < dist_thresh :
            
            self.isStop = 1
        
        self.check_condition()
            
        return self.velcmd, self.steer, self.brake



    def check_condition(self) :
        
        if self.isStop :
            
            self.brake  = 200
            self.velcmd = 0
            self.steer  = 0
            
            self.check_restart()
        
        else : 
            
            self.brake  = 0
            self.velcmd = vel_cmd
            self.steer  = 0



    def check_restart(self) :
        
        self.restart += 1
            
        if(self.restart == 50) :                    # 정차 후 5초 후에 다시 출발
                
            self.isStop, self.restart = 0, 0
            self.dist, self.prev_dist = 1000, 1000


    def printResult(self, loop_time) :
        
        self.cnt += 1
        
        if self.cnt%5 == 0 :
            
            print("--------------------------------------------------")
            print(f"|       SIGN DIST     : {round(self.dist      , 2)}   [m]   ")
            # print(f"|       PREV DIST     : {round(self.prev_dist , 2)}   [m]   ")
            print(f"|       LOOP TIME     : {round(loop_time      , 2)}   [sec] ")
            print(f"|       IS STOP(1/0)  : {self.isStop}")
            print(f"|       RESTART FLAG  : {self.restart}")
            print(f"|       VELOCITY      : {self.velcmd}")
            print("--------------------------------------------------\n")


    def plotProjection(self) :  
        
        plt.plot(self.pixelU          , self.pixelV         , 'b.')
        plt.plot(self.pixel_select[0] , self.pixel_select[1], 'r.', markersize = 20)
        # plt.plot(self.camMid[0]       , self.camMid[1]      , 'g.', markersize = 10)
        plt.xlabel('pixel X')
        plt.ylabel('pixel Y')
        plt.xlim([0, 640])
        plt.ylim([0, 480])
        plt.grid(True)
        plt.tick_params(axis = 'both', labelsize = 7)
        

    
if __name__ == "__main__" :
    
    sharedMem  = Velodyne_Sharedmemory()
    projection = Projection()
    command    = erpSerial(device_name)
    
    sharedMem.Lidar_SMopen()
    shm = shared_memory.SharedMemory(name = shared_memory_send, create = True, size = send_mem_size)
    
    print("Shared memory with camera is opened ...!")
    
    time_start = time.time()
    
    while (time_stime < time_final):
    
        loopStart = time.time()
                
        sharedMem.Importdata()
        
        projection.receiveObject(shared_memory_receive)
        
        velocity, steer, brake = projection.projectPCD()
        
        projection.sendtoCam(shared_memory_send)
        
        command.send_ctrl_cmd(int(velocity), int(steer), int(brake))
        
        loopEnd = time.time()
        
        projection.printResult(loopEnd - loopStart)
        
        while(1):
            
            time_curr = time.time()
            
            time_del = time_curr - time_start - time_stime
            
            if (time_del > time_ts) :
                
                time_cnt += 1
                time_stime = time_cnt*time_ts
                
                break
            

    sharedMem.sharedmemory_close()
    shm.close()