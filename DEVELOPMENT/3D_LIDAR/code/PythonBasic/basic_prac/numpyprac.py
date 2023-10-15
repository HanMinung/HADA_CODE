import numpy as np

def writePCDFile(fname,x,y,z,i):
    
    numPoints= len(x)
    
    with open(fname, 'w') as fp:
        
        fp.write("VERSION .7\n")
        fp.write("FIELDS x y z intensity\n")
        fp.write("SIZE 8 8 8 8\n")
        fp.write("TYPE F F F F\n")
        fp.write("COUNT 1 1 1 1\n")
        fp.write("WIDTH "+str(numPoints)+"\n")
        fp.write("HEIGHT 1\n")
        fp.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        fp.write("POINTS "+str(numPoints)+"\n")
        fp.write("DATA ascii\n")
        
        for index in range(numPoints):
            txtLine = "{} {} {} {}\n".format(x[index],y[index],z[index], i[index] )
            fp.write(txtLine)
            
        pass


def calc(dis) :
    
    X = dis * np.cos(30)
    Y = dis * np.sin(30)
    Z = dis * np.tan(30)
    I = 3
    
    return [X, Y, Z, I]


if __name__ == "__main__" : 
    
    fname = 'result.pcd'
    dis = 1
    XYZI = []
    
    for idx in range(10) :
        
        dis += 1
        XYZI.append(calc(dis))  
  
    arr = np.array(XYZI)
    
    X = arr[:,0]
    Y = arr[:,1]
    Z = arr[:,2]
    I = arr[:,3]    
    # print(XYZI)
    writePCDFile(fname,X,Y,Z,I)