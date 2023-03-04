# POOL : 처리할 일들을 뿌려놓고 프로세스 별로 할당량을 명기적으로 정한다.
# PID : process identification number

from multiprocessing import Pool
import time
import os
import math

def f(x):
    
    print("값", x, "에 대한 작업 Pid = ",os.getpid())
    time.sleep(1)
    return x*x

if __name__ == '__main__':
    
    p = Pool(3)
    startTime = time.time()
    
    print(p.map(f, range(0,10))) 
    
    endTime = time.time()
    print("TASK TIME", (endTime - startTime))
