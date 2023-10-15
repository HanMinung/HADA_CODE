from module import *

global azimuth
azimuth = np.linspace(-95 ,95 ,761)

sm    =    SHAREDMEM()
rt    =    REALTIME()
pr    =    PERIPHERAL()
so    =    STATIC()

class PROCESS :

    def __init__(self) :
        
        self.lidar_dist = np.zeros(761)


    def receive_from_lidar(self, shm_name) :

        self.lidar_dist = np.zeros(761)
        
        try :
            
            shm = shared_memory.SharedMemory(name = shm_name)
            shared_data = np.ndarray((761, ), dtype ='double', buffer = shm.buf) 
                
            self.lidar_dist = np.copy(shared_data)
                
        except FileNotFoundError :
            
            print("cannot read sm data from 'MORAI'...!")    
            
    
    
    def data_preprocessing(self) :
        
        for Idx in range(so.lidar_num) :
            
            if Idx == 0 :
                
                if self.lidar_dist[Idx] == 0.0 : self.lidar_dist[Idx] = 1.0
                
            else : 
                
                if self.lidar_dist[Idx] == 0.0 : self.lidar_dist[Idx] = self.lidar_dist[Idx-1]
            
            if self.lidar_dist[Idx] >= (so.range_max * 500) or self.lidar_dist[Idx] <= (so.range_min * 500) :
                
                self.lidar_dist[Idx] = 1
                
            else : self.lidar_dist[Idx] = self.lidar_dist[Idx] / (so.range_max * 500)
    

    
    def main_processing(self) :
        
        self.data_preprocessing()
        
        drawnow.drawnow(self.plot_result)
    
    
    
    def plot_result(self) :
        
        plt.plot(azimuth, self.lidar_dist, 'r-', pr.colorBlue)
        plt.xlim([-95, 95])
        plt.ylim([0, 1.2])
        plt.grid(True)
        
        
    
    
    def print_result(self, loop_time) :
        
        print(f"LOOP TIME  :  {round(loop_time, 3)} [sec]")
        
    
    

if __name__ == "__main__" :                   
    
    process = PROCESS()

    time_start = time.time()
    
    while (rt.time_stime < rt.time_final) :
        
        loop_start = time.time()
        
        process.receive_from_lidar(sm.from_morai_to_main)
        
        process.main_processing()
        
        loop_end = time.time()
        
        process.print_result(loop_end - loop_start)
        
        while(1):
            
            time_curr = time.time()
            time_del = time_curr - time_start - rt.time_stime
            
            if (time_del > rt.time_ts):
                
                rt.time_cnt+=1
                rt.time_stime = rt.time_cnt * rt.time_ts
                
                break

