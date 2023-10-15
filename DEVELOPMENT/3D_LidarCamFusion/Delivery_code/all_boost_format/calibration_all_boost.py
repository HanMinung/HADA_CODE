from module import *

deliv     =  DELIVERY()
realtime  =  REALTIME()
platform  =  ERP42()


if __name__ == "__main__" :
    
    command    = erpSerial(platform.device_num)
    
    time_start = time.time()

    while (realtime.time_stime < realtime.time_final):
    
        loopStart = time.time()
        
        deliv.delivery_mission()
        
        command.send_ctrl_cmd(deliv.velcmd, deliv.steercmd, deliv.brakecmd)

        loopEnd = time.time()
                
        while(1) :
            
            time_curr = time.time()
            
            time_del = time_curr - time_start - realtime.time_stime
            
            if (time_del > realtime.time_ts) :
                
                realtime.time_cnt += 1
                realtime.time_stime = realtime.time_cnt * realtime.time_ts
                
                break