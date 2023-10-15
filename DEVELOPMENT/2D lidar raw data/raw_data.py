from module import *


if __name__ == '__main__' :
    
    LiDAR = LiDAR_SHAREDMEM()
    
    LiDAR.lidar_sharedmem_open()
    
    while(True) :
        
        LiDAR.receive_lidar()
        
        print(LiDAR.lidar_dist[380])