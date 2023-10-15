from multiprocessing import Process, Queue

def func(list, q) :
    
    for x in list :
        q.put(x)
        
if __name__ == '__main__' : 
    
    q = Queue()
    p = Process(target = func, args = (["h", "e", "l", "l", "o"], q))
    p.start()
    p.join()
    
    for x in range(q.qsize()) : 
        
        print(q.get())