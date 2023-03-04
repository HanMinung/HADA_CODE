import sys
from multiprocessing import Process
import time

def _print(content) :
     
    print(content)


if __name__ == "__main__":
    
    time_curr = 0
    time_cnt = 0
    time_stime = 0
    time_ts = 1      
    time_final = 5
    
    if sys.argv[1] == 'go':
        
        time_start = time.time()
        
        while (time_stime < time_final) :
        
            # processA = Process(target = _print, args = (''.join(sys.argv[2])))
            # processA.start()
            
            _print(sys.argv[2])
            
            while(1) :
            
                time_curr = time.time()
                time_del = time_curr - time_start - time_stime
                
                if (time_del > time_ts):
                    
                    time_cnt+=1
                    time_stime = time_cnt*time_ts
                    
                    break
    
    print("All task finished ...!")