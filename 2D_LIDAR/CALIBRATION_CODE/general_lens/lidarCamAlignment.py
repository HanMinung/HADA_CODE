# -------------------------------------------------------------------
#
#   @   HADA code  :   Track guidance
#   @   Update     :   2023.6.03
#   @   Purpose    :   Lidar Cam calibration
#
# -------------------------------------------------------------------

from globVariable import *

global lidarDist, azim
lidarDist  = []
azim = np.linspace(-7.5 ,183.5 ,761)

class READ_DATA(ct.Structure):
    
    _fields_ = [("xCoordinate",ct.c_double*761),("yCoordinate",ct.c_double*761),("dist",ct.c_double*761),("angle",ct.c_double*761)]
         
class Lidar_SharedMem :
    
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

    def Yolo_SMopen(self) :
        
        self.is_yoloSM  = True
        print("Shared memory with YOLO program opened ...!")
        
    def receiveDist(self):
        
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


class PROJECTION :

    def __init__(self) :
        
        self.projectionX    = []
        self.projectionY    = []
        self.lidarY         = []
        self.lidarX         = []

        self.lidarxList     = []
        self.lidaryList     = []
        
        self.candidates     = []
        self.distance       = 0
        self.yoloBuffer     = []
        self.maxDiff        = 25
        self.isyoloReady    = False
        self.steerCmd       = 0
        
        self.markerSize     = 25
        self.userFont       = cv.FONT_HERSHEY_COMPLEX
        
        self.Ux             = 0
        self.Uy             = 0 
        self.prevUx         = 0
        self.prevUy         = 0
        self.prevsteerCmd   = 0
        self.startFlag      = 0
        
        
    def polar2xy(self, dist, azi) : 

        n = len(azi)
        x = np.zeros(n)
        y = np.zeros(n)

        for i in range(n) :

            x[i] = dist[i] * cos(azi[i] * D2R)
            y[i] = dist[i] * sin(azi[i] * D2R)

        return x, y



    def circumCircle(self, P1, P2, P3) :
        
        ax, ay = P1
        bx, by = P2
        cx, cy = P3

        d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))

        self.Ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
        self.Uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d

        self.prevUx, self.prevUy = self.Ux, self.Uy
    
    
    
    def findSteer(self) :
        
        objectN = len(self.lidarxList)

        if objectN == 1 :
            
            self.steerCmd = self.prevsteerCmd
            self.Ux = self.prevUx
            self.Uy = self.prevUy
            
        elif objectN == 2 :
            
            self.Ux = (self.lidarxList[0] + self.lidarxList[1]) / 2
            self.Uy = (self.lidaryList[0] + self.lidaryList[1]) / 2
            
            self.prevUx, self.prevUy = self.Ux, self.Uy
            
            self.steerCmd = round(90 - atan2(self.Uy,self.Ux) * R2D, 2)
        
        elif objectN >= 3 :
            
            distances = [sqrt(x**2 + y**2) for x, y in zip(self.lidarxList, self.lidaryList)]
            sortedObject = np.argsort(distances)[:3]  

            P1 = (self.lidarxList[sortedObject[0]], self.lidaryList[sortedObject[0]])
            P2 = (self.lidarxList[sortedObject[1]], self.lidaryList[sortedObject[1]])
            P3 = (self.lidarxList[sortedObject[2]], self.lidaryList[sortedObject[2]])

            self.circumCircle(P1, P2, P3)

            self.steerCmd = round(90 - atan2(self.Uy,self.Ux) * R2D, 2)
        
        if(self.steerCmd > 28) : self.steerCmd = maxsteerR
        
        elif(self.steerCmd < -28) : self.steerCmd = maxsteerL 
        
        self.prevsteerCmd = self.steerCmd
        
        
        


    def recieveYolo(self, frame) :
        
        shm = shared_memory.SharedMemory(name = "HADA3_CAM")
        self.yoloBuffer = np.ndarray((12,), dtype='int', buffer = shm.buf)
        
        for i in range(0, len(self.yoloBuffer), 2) :
            
            if(self.yoloBuffer[i] < 50) : self.yoloBuffer[i] = self.yoloBuffer[i] - 25
            
            if(self.yoloBuffer[i] > 590) : self.yoloBuffer[i] = self.yoloBuffer[i] + 25
            
            cv.circle(frame, (self.yoloBuffer[i], self.yoloBuffer[i+1]), 5, colorYellow, -1)    
        
        """
            - 어떤 프로그램을 먼저 실행시키냐에 따라 오류가 발생하는 것을 방지하고자, startflag를 사용
        """        

        if self.yoloBuffer[0] is not 0 :
            
            self.isyoloReady = True
            
            if self.startFlag > 0 :
                
                self.lidarxList, self.lidaryList = [], []
            
                for Idx in range(0, len(self.yoloBuffer), 2) :

                    objectcenX, objectcenY = self.yoloBuffer[Idx], self.yoloBuffer[Idx+1]

                    self.candidates = []

                    if objectcenX == 0 or objectcenY == 0 : continue

                    for i in range(761):

                        if abs(self.projectionX[i] - objectcenX) < self.maxDiff and self.projectionY[i] > objectcenY - 30:

                            self.candidates.append(i)

                            if len(self.candidates) == 4 and self.lidarY[i] != 0 :
                                  
                                self.lidarxList.append(self.lidarX[i])
                                self.lidaryList.append(self.lidarY[i])
                                    
                                break
                        
            # steering determination
            self.findSteer()
             
            cv.putText(frame, f'steer : {self.steerCmd} [deg]', (20, 40), self.userFont, 1, colorWhite, 1)
            
            # drawnow.drawnow(self.plotLidar)
            
            self.startFlag += 1
            
        return self.steerCmd
            
            
    def plotLidar(self) :
        
        plt.plot(self.Ux, self.Uy, 'bo', markersize = self.markerSize)
        plt.plot(self.lidarxList, self.lidaryList, 'ro', markersize = self.markerSize)
        plt.xlabel('x coor [m]', fontsize = 25)
        plt.ylabel('y coor [m]', fontsize = 25)
        plt.xlim([-3, 3])
        plt.ylim([0, 6])
        plt.grid(True)
        plt.tick_params(axis='both', labelsize=20)
        


    def lidarCamProjection(self, frame) :                                 

        self.projectionX = []
        self.projectionY = []

        # lidar raw data preprocessing
        for i, dis in enumerate(lidarDist):

            lidarDist[i] = lidarDist[i] / 500
            
            if lidarDist[i] == 0 and i != 0 : 
                
                lidarDist[i] = lidarDist[i-1]
            
            if lidarDist[i] >= 8 :
                
                lidarDist[i] = 8
            

        self.lidarX , self.lidarY = self.polar2xy(lidarDist,azim)                      

        for i in range(len(azim)):

            pixelXY  = 0
            pointcloudXY = 0

            lidarX = self.lidarX[i]
            lidarY = self.lidarY[i]
            lidarZ = 0
            scale = lidarY + realRecede            

            pointcloudXY = np.array([[lidarX],[lidarY],[lidarZ],[1]])
            pixelXY = 1/scale * intMat @ extMat @ pointcloudXY

            pixelU = int(pixelXY[0])
            pixelV = int(pixelXY[1])

            self.projectionX.append(pixelU)
            self.projectionY.append(pixelV)
            
            cv.circle(frame, (pixelU, pixelV), 2, colorWhite, -1)    




if __name__ == "__main__" : 
    
    sim      =  Lidar_SharedMem()
    project  =  PROJECTION()
    command  =  erpSerial(device)
    
    sim.Lidar_SMopen()
    sim.Yolo_SMopen()
    
    time_start = time.time()

    blackorgImg = np.zeros((480, 640, 3), dtype = np.uint8)

    while (time_stime < time_final):
        
        loopStart = time.time()
                
        blackImg = np.copy(blackorgImg)
        
        sim.receiveDist()
        
        steerCMD = project.recieveYolo(blackImg)
            
        project.lidarCamProjection(blackImg)
        
        cv.imshow("test",blackImg)
                
        # erp command
        command.send_ctrl_cmd(int(velCMD), int(steerCMD))
        
        if cv.waitKey(1) & 0xFF == ord('q') : break
                    
        loopEnd = time.time()
        
        print(f"loop freq : {round(1/(loopEnd - loopStart))} [HZ]" )
                    
        while(1):
            
            time_curr = time.time()
            time_del = time_curr - time_start - time_stime
            
            if (time_del > time_ts) :
                
                time_cnt += 1
                time_stime = time_cnt*time_ts
                
                break
            

    sim.sharedmemory_close()
