from globVariable import *

global lidar_dist, azim
lidar_dist = []
azim = np.linspace(-7.5 ,183.5 ,761)


class READ_DATA(ct.Structure):
    
    _fields_ = [("xCoordinate",ct.c_double*761),("yCoordinate",ct.c_double*761),("dist",ct.c_double*761),("angle",ct.c_double*761)]
         
         
class Lidar_SharedMem :
    
    def __init__(self):
        
        self.is_lidarSM = False
        self.is_yoloSM  = False

    def lidar_sharedmem_open(self) :
        
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
        # self.rfile_mapping_name_ptr = ct.c_wchar_p("Hils_smdat_ReadData")

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


    def yolo_sharedmem_open(self) :
        
        self.is_yoloSM  = True
        print("Shared memory with YOLO program opened ...!")
        
        
    def receive_lidar(self):
        
        global lidar_dist

        if self.is_lidarSM == True:
            
            read_smdat = READ_DATA()
            rmsg_ptr   = ct.pointer(read_smdat)
            self.memcpy(rmsg_ptr,self.rmapped_view_ptr,self.rbyte_len)
            lidar_dist   = read_smdat.dist
     
    def sharedmemory_close(self):
        self.UnmapViewOfFile(self.wmapped_view_ptr)
        self.CloseHandle(self.wmapping_handle)
        self.UnmapViewOfFile(self.rmapped_view_ptr)
        self.CloseHandle(self.rmapping_handle)


class PROJECTOR :

    def __init__(self) :
        
        self.projectionX = []
        self.projectionY = []
        self.lidarX      = []
        self.lidarY      = []
        self.candidates  = []
        self.avgDist     = 0
        self.distance    = 0
        self.distances   = []
        self.distVal     = 0
        self.maxDiff     = 20
        
        
    def polar2xy(self, dist, azi) : 

        n = len(azi)
        x = np.zeros(n)
        y = np.zeros(n)

        for i in range(n) :

            x[i] = dist[i] * cos(azi[i] * D2R)
            y[i] = dist[i] * sin(azi[i] * D2R)

        return x, y



    def lidar_pixel_projection(self, frame) :                                 

        self.projectionX = []
        self.projectionY = []

        for i, dis in enumerate(lidar_dist):

            lidar_dist[i] = lidar_dist[i] / 500

        self.lidarX , self.lidarY = self.polar2xy(lidar_dist,azim)                      

        for i in range(len(azim)):

            XY_im  = 0
            XY_cam = 0

            lx = self.lidarX[i]
            ly = self.lidarY[i]
            lz = 0
            Cz = ly + realRecede

            XY_cam = np.array([[lx],[ly],[lz],[1]])
            XY_im = 1/Cz * modified_intMat @ extrinsic_mat @ XY_cam

            xx = int(XY_im[0])
            yy = int(XY_im[1])

            self.projectionX.append(xx)
            self.projectionY.append(yy)

            cv.circle(frame, (round(xx) ,round(yy)), 3, colorYellow)
            
        

if __name__ == "__main__" :
    
    sim     = Lidar_SharedMem()
    project = PROJECTOR()
    
    sim.lidar_sharedmem_open()
    sim.yolo_sharedmem_open()

    time_start = time.time()
    
    webcam = cv.VideoCapture(cv.CAP_DSHOW +  1)
    
    if not webcam.isOpened():
        
        print("Could not open webcam")
        exit()

    new_int_matrix, roi = cv.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (img_width, img_height), 1, (img_width, img_height))

    while (time_stime < time_final and webcam.isOpened()):
        
        ret, frame = webcam.read()

        undistorted_frame = cv.undistort(frame, camera_matrix, dist_coeffs, None, new_int_matrix)

        sim.receive_lidar()
        
        if ret:
            
            project.lidar_pixel_projection(undistorted_frame)
           
            # 객체 인식 데이터 수신
        
            # 그거에 맞게 가장 이상적인 point 검출
            
            # classification
            
            # plotting
           
            cv.imshow("test",undistorted_frame)
                
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
          
        while(1):
            
            time_curr = time.time()
            time_del = time_curr - time_start - time_stime
            
            if (time_del > time_ts) :
                
                time_cnt += 1
                time_stime = time_cnt*time_ts
                
                break
            

    sim.sharedmemory_close()
