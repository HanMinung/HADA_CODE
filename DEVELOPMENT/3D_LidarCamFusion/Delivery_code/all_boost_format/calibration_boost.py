from module import *
from HADA import cameraPy, lidar3DPy

sm         =   SHAREDMEM()
genlens    =   CALIBRATION_WITHOUT_LENS()
widelens   =   CALIBRATION_WIDE_LENS()
realtime   =   REALTIME()

class PROCESSOR :
    
    def __init__(self) :
        
        self.velodyne         =  lidar3DPy.driver()
        
        self.point_cloud_data =  None
        self.point_cloud_X    =  []
        self.point_cloud_Y    =  []
        self.point_cloud_Z    =  []
        
        self.pixel_xlist      =  []
        self.pixel_ylist      =  []

        self.object_center    =  []

        
    
    def Initialize(self) :
        
        self.velodyne.run()
        
        print("Initialization completed ...!")
        
        
    def Get_pcd_data(self) :
        
        self.point_cloud_data = self.velodyne.get_data()
        
    
    def Project_pcd(self) :
        
        self.pixel_xlist = []
        self.pixel_ylist = []
        
        for Idx in range(len(self.point_cloud_data)) :
            
            point_X = self.point_cloud_data[Idx][0]
            point_Y = self.point_cloud_data[Idx][1]
            point_Z = self.point_cloud_data[Idx][2]
            
            if point_X == 0 and point_Y == 0 and point_Z == 0 : break
    
            scale_factor = point_Y + widelens.cam_recede

            world_coor   = np.array([[point_X], [point_Y], [point_Z], [1]])
            pixel_coor   = 1/scale_factor * widelens.wide_intrinsic_mat @ genlens.extrinsic_mat @ world_coor

            pixel_U = pixel_coor[0]
            pixel_V = pixel_coor[1]
            
            if pixel_U >= 0 and pixel_U <= 640 and pixel_V >= 0 and pixel_V <= 480 :   
                            
                self.pixel_xlist.append(int(pixel_U))
                self.pixel_ylist.append(int(pixel_V))
        
        if len(self.pixel_xlist) != 0 : 
            
            print(self.pixel_xlist[np.argmin(self.pixel_xlist)])
            print(self.pixel_xlist[np.argmax(self.pixel_xlist)])
            print(self.pixel_ylist[np.argmin(self.pixel_ylist)])
            print(self.pixel_ylist[np.argmax(self.pixel_ylist)])

        # print(len(self.pixel_xlist))


    def Main_loop(self) :
        
        self.Get_pcd_data()
        
        self.Project_pcd()      
        
        drawnow.drawnow(self.plot_result)  
        
        
    
    
    def plot_result(self) :
        
        plt.plot(self.pixel_xlist, self.pixel_ylist, 'bo', markersize = 2)
        plt.xlim([0 , 640])
        plt.ylim([0 , 480])
        plt.grid(True)
        
    
    
    def Print_result(self, loop_time) :
        
        if loop_time != 0 :
            
            print(f"LOOP FREQUENCY  :  {round(1/loop_time)}    [HZ]")

    


if __name__ == "__main__" :
   
    process = PROCESSOR()
    
    process.Initialize()
    
    time_start = time.time()
    
    while (time_stime < realtime.time_final):
    
        loopStart = time.time()
                
        process.Main_loop()
        
        loopEnd = time.time()
        
        process.Print_result(loopEnd - loopStart)
        
        while(1):
            
            time_curr = time.time()
            
            time_del = time_curr - time_start - time_stime
            
            if (time_del > realtime.time_ts_main) :
                
                realtime.time_cnt += 1
                time_stime = realtime.time_cnt * realtime.time_ts_main
                
                break
    