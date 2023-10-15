from module import *
from HADA import lidar3DPy

# HADA 인식안되는거 해결해야함

sm = SHAREDMEM()
deliv = DELIVERY()
realtime = REALTIME()


class PROCESSOR:

    def __init__(self):
        self.velodyne = lidar3DPy.driver()
        self.projector = None

        self.cam_middle = []
        self.sign_dist = 0

    def Initialize(self):
        self.velodyne.run()
        self.projector = lidar3DPy.projector()

        print("Initialization completed ...!")

    def receive_cam(self, shared_mem_name):
        self.cam_middle = []

        try:
            shm = shared_memory.SharedMemory(name=shared_mem_name)
            shared_data = np.ndarray((9,), dtype='int', buffer=shm.buf)

            # print(shared_data)

            self.cam_middle.append(shared_data[1])
            self.cam_middle.append(shared_data[2])

        except FileNotFoundError:
            print("cannot read sm data from 'CAMERA'...!")

    def get_sign_dist(self):
        if len(self.cam_middle) != 0:
            self.sign_dist = self.projector.project_PCD(self.velodyne, self.cam_middle[0], self.cam_middle[1], True)

    def Main_loop(self):
        self.receive_cam(sm.from_cam_to_delivry)

        self.get_sign_dist()

        deliv.sign_stop()

    def Print_result(self, loop_time):
        if loop_time != 0:
            print("-------------------------------------------------------")
            print(f"LOOP FREQUENCY  :  {round(1 / loop_time)}    [HZ]")
            print(f"SIGN DIST       :  {round(self.sign_dist, 2)}   [m]")


if __name__ == "__main__":
    process = PROCESSOR()

    process.Initialize()

    time_start = time.time()

    while (time_stime < realtime.time_final):
        loopStart = time.time()

        process.Main_loop()

        loopEnd = time.time()

        process.Print_result(loopEnd - loopStart)

        while (1):
            time_curr = time.time()

            time_del = time_curr - time_start - time_stime

            if (time_del > realtime.time_ts_main):
                realtime.time_cnt += 1
                time_stime = realtime.time_cnt * realtime.time_ts_main

                break
