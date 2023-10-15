#!/usr/bin/python

from module import *

DATA_QUEUE = Queue(-1)

def capture(port, data_queue):
    
    soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    soc.bind(('', port))
    
    try:
        while True:
            
            try:
                data = soc.recv(2000)
                if len(data) > 0:
                    assert len(data) == 1206, len(data)
                    data_queue.put({'data': data, 'time': time.time()})
                    
            except Exception as e:
                
                print(dir(e), e.message, e.__class__.__name__)
                traceback.print_exc(e)
                
    except KeyboardInterrupt as e:
        
        print(e)                               
            

def save_package(dirs, data_queue):
    
    idx = 1
    buffer = bytearray()
    
    try:
        
        if os.path.exists(dirs) is False:
            os.makedirs(dirs)
            
        cnt = 0
        fp = None
        
        START_TIME = time.monotonic()
        
        while True:
            
            if data_queue.empty():
                pass
            
            else:
                msg = data_queue.get()
                
                data = msg['data']              # 1206 byte
                ts = msg['time']                # 24 byte
                
                buffer.extend(data)
                
                if(time.monotonic() - START_TIME >= framecut) :
                    
                    path = os.path.join(dirs, 'frame_' + str(idx) + '.bin')
                    fp = open(path, 'ab')
                    
                    while len(buffer) >= 1206 :
                        
                        chunk = buffer[:1206]
                        buffer = buffer[1206:]
                        
                        fp.write(b'%.6f'%ts)
                        fp.write(chunk)
                        
                    idx += 1
                    START_TIME = time.monotonic()
                    
                    # print("Bin file writing completed ...!")
                    
                if fp == None or cnt == 1000000:
                    
                    if fp != None:
                        fp.close()

                    cnt = 0

                cnt += 1
                
    except KeyboardInterrupt as e:
        
        print(e)
        
    finally:
        
        if fp != None:
            fp.close()



class DECODE :

    def __init__(self) :
        
        self.pcd_idx = 0
        self.csv_idx = 0
        self.scan_idx = 0
        self.remove_idx = 1
        
    def unpack(self, *bin_dirs) :
        
        global count

        bin_dirs = ''.join(bin_dirs)
        points = []
        prev_azimuth = None

        files = glob.glob(bin_dirs + '/*.bin')
        idx = len(files)

        if (idx < 3) :
            pass
        
        else :
            
            STIME = time.time()
            
            d = files[idx - 2]
            d = open(d, 'rb').read()
            n = len(d)
                
            for offset in range(0, n, 1223) :

                ts = d[offset : offset + 17]
                data = d[offset + 17 : offset + 1223]
                
                timestamp, factory = struct.unpack_from("<IH", data, offset=1200)
                assert factory == 0x2237, hex(factory)        # 0x22=VLP-16, 0x37=Strongest Return
                timestamp = float(ts)
                seq_index = 0

                for offset in range(0, 1200, 100):

                    flag, azimuth = struct.unpack_from("<HH", data, offset)
                    assert flag == 0xEEFF, hex(flag)

                    for step in range(2) :

                        seq_index += 1
                        azimuth += step
                        azimuth %= ROTATION_MAX_UNITS

                        if prev_azimuth is not None and azimuth < prev_azimuth :
                            
                            file_fmt = os.path.join(bin_dirs, 'XYZ_' + '%Y-%m-%d_%H%M')
                            path = datetime.now().strftime(file_fmt)

                            try:
                                if os.path.exists(path) is False:
                                    os.makedirs(path)

                            except Exception as e:
                                print(e)

                            if not points:
                                timestamp_str = b'%.6f' % time.time()

                            else:
                                timestamp_str = b'%.6f' % points[0][3]
                            
                            arr = np.array(points)
                            
                            # Preprocessing 
                            # arr = arr[arr[:,1] >= 0]
                            
                            self.pcd_idx += 1
                            writePCDFile("{}/pcdindex_{}.pcd".format(path, self.pcd_idx), arr[:,0], arr[:,1], arr[:,2], arr[:,3])

                            # self.csv_idx +=1
                            # writeCSV("{}/csvindex_{}.csv".format(path, self.csv_idx), points)
                            
                            # removeFile(bin_dirs, self.remove_idx)
                            # self.remove_idx += 1
                            
                            FTIME = time.time()
                            print('number of points : {}  |   TASK TIME : {}'.format(len(arr), FTIME - STIME))
                            
                            points = []

                        prev_azimuth = azimuth
                        arr = struct.unpack_from('<' + "HB" * 16, data, offset + 4 + step * 48)

                        for i in range(NUM_LASERS):

                            time_offset = (55.296 * seq_index + 2.304 * i) / 1000000.0

                            if arr[i * 2] != 0 :

                                points.append(calc(arr[i * 2], azimuth, i, timestamp + time_offset))

                    
                            

def removeFile(dirs, idx) :
    
    filename = 'frame_' + str(idx) + '.bin'
    file = os.path.join(dirs, filename)
    
    os.remove(file)



def calc(dis, azimuth, laser_id, timestamp):
    
    R = dis * DISTANCE_RESOLUTION
    omega = LASER_ANGLES[laser_id] * np.pi / 180.0
    alpha = azimuth / 100.0 * np.pi / 180.0
    X = R * np.cos(omega) * np.sin(alpha)
    Y = R * np.cos(omega) * np.cos(alpha)
    Z = R * np.sin(omega)
    
    return [X, Y, Z, timestamp]
    


if __name__ == "__main__" :
    
    decode = DECODE()
    top_dir = 'binfile_' + datetime.now().strftime('%H_%M_%S')
    
    printSetting()
    
    processA = Process(target = capture, args = (PORT, DATA_QUEUE))
    processA.start() 
    processB = Process(target = save_package, args = (sys.argv[1] + '/' + top_dir, DATA_QUEUE))
    processB.start()
    
    time_start = time.time()
    
    while (time_stime < time_final) :
        
        decode.unpack(os.path.join(sys.argv[1], top_dir))
        
        while(1) :
             
            time_curr = time.time()
            time_del = time_curr - time_start - time_stime
            
            if (time_del > time_ts):
                
                time_cnt += 1
                time_stime = time_cnt * time_ts
                
                break
    
    processA.terminate()
    processB.terminate()
    
    print("PROCESS A - B TERMINATED ...!\n")
    print("PROGRAM FINISHED ...!\n")
    input("PRESS ENTER TO EXIT THE PROGRAM ...!")
    