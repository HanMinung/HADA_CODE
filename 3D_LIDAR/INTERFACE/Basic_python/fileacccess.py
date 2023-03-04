import os
import os.path
import glob

path = 'C:/Users/hanmu/Desktop/Camera/python_basic/basic_prac/test_2'

# print(files[0])

idx = -5

for i in range(7) :
    
    # files = os.listdir(path)
    files = glob.glob(path + '/*.bin')
    # print(files)
    
    idx += 1
    
    if (idx < 0) :
        print("PASS..!")
        pass
    
    else :
        file = open(files[idx], 'rb').read()
        print(type(file))
        print("===============")
        