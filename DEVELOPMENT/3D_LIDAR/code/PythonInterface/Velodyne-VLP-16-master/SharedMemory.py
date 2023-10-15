from module import *

def receive_data(shm_name) :
    
    print("RECEIVING DATA...!")
    
    shm = shared_memory.SharedMemory(name = shm_name)
    
    shared_data = np.ndarray((4,), dtype = 'float64', buffer = shm.buf)

    print("RECEIVED DATA:", shared_data[:])