import sys
from multiprocessing import Process

def adder(fac1, fac2) : 
    
    sum = fac1 + fac2
    print(sum)

def sub(fac1, fac2) :
    
    sub = fac1 - fac2
    print(sub)


if __name__ == '__main__' :
    
    if(sys.argv[1] == 'calculation') :
        
        processA = Process(target = adder, args = (5, 3))
        processA.start()
        
        processB = Process(target = sub, args = (5, 3))
        processB.start()