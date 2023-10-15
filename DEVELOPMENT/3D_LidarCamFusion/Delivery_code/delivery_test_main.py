from variable import *


class CONTROL :
    
    def __init__(self) :
        
        self.stop_flag = 0
        self.speed_cmd = 0
        self.steer_cmd = 0
        self.brake_cmd = 0
        
        self.print_cnt = 0
        
        
        
    def receive_from_lidar(self) :
        
        try :
            
            shm = shared_memory.SharedMemory(name = shared_memory_send_to_main)
            shared_data = np.ndarray((1, ), dtype ='int', buffer = shm.buf) 
            
            self.stop_flag = shared_data[0]
            
        except FileNotFoundError :
            
            print("cannot read sm data from 'POST PROCESSING CODE'...!")
        


    def control_erp(self) :
        
        if self.stop_flag == 0 :
            
            self.speed_cmd  =  vel_cmd
            self.steer_cmd  =  0
            self.brake_cmd  =  0
        
        if self.stop_flag != 0 :
            
            self.speed_cmd  =  0
            self.steer_cmd  =  0
            self.brake_cmd  =  200
            
        return self.speed_cmd, self.steer_cmd, self.brake_cmd



    def print_result(self, loop_time) :
        
        self.print_cnt += 1
        
        if self.print_cnt % 70 == 0 :
            
            print("------------------------------------------------------------")
            print(f"     STOP FLAG FROM LIDAR : {self.stop_flag}              ")
            print(f"     VELOCITY             : {self.speed_cmd}        [km/h]")
            print(f"     STEER                : {self.steer_cmd}        [deg] ")
            print(f"     BRAKE                : {self.brake_cmd}              ")
            print(f"     LOOP TIME            : {round(loop_time, 2)}      [sec] ")
            print("------------------------------------------------------------")



if __name__ == "__main__" :
    
    command = erpSerial(device_name)
    control = CONTROL()
    
    time_start = time.time()
    
    while (time_stime < time_final):
    
        loopStart = time.time()
        
        control.receive_from_lidar()
                
        speed, steer, brake = control.control_erp()
        
        command.send_ctrl_cmd(int(speed), int(steer), int(brake))
        
        loopEnd = time.time()
        
        control.print_result(loopEnd - loopStart)
        
        while(1):
            
            time_curr = time.time()
            
            time_del = time_curr - time_start - time_stime
            
            if (time_del > time_ts_main) :
                
                time_cnt += 1
                time_stime = time_cnt*time_ts_main
                
                break
    