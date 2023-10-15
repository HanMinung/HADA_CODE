from multiprocessing import Process, Pipe

def func0(pipe) : 
    
    pipe.send("hello world")
    pipe.close
    
def func1(pipe) :
    
    print(pipe) 
    
if __name__ == '__main__' :
    
    a_pipe, b_pipe = Pipe()
    p0 = Process(target = func0, args = (a_pipe, ))
    p0.start()
    
    p1 = Process(target = func1, args = (b_pipe.recv(),))
    p1.start()
    
    p0.join()
    p1.join()