from module import *

def writePCDFile(fname,x,y,z,i):
    
    numPoints= len(x)
    
    with open(fname, 'w') as fp:
        
        fp.write("VERSION .7\n")
        fp.write("FIELDS x y z intensity\n")
        fp.write("SIZE 8 8 8 8\n")
        fp.write("TYPE F F F F\n")
        fp.write("COUNT 1 1 1 1\n")
        fp.write("WIDTH "+ str(numPoints)+"\n")
        fp.write("HEIGHT 1\n")
        fp.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        fp.write("POINTS "+ str(numPoints)+"\n")
        fp.write("DATA ascii\n")
        
        for index in range(numPoints):
            txtLine = "{} {} {} {}\n".format(x[index],y[index],z[index], i[index] )
            fp.write(txtLine)
            
        pass
    
    
def calc(dis, azimuth, laser_id, timestamp):
    
    R = dis * DISTANCE_RESOLUTION
    omega = LASER_ANGLES[laser_id] * np.pi / 180.0
    alpha = azimuth / 100.0 * np.pi / 180.0
    X = R * np.cos(omega) * np.sin(alpha)
    Y = R * np.cos(omega) * np.cos(alpha)
    Z = R * np.sin(omega)
    
    return [X, Y, Z, timestamp]
