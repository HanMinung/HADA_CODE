import os
import sys

if __name__ == "__main__" :
    
    top_dir = 'binfile_14_23_08'
    path = os.path.join(sys.argv[1], top_dir)
    file = 'frame_1.bin'
    file = os.path.join(path, file)
    
    os.remove(file)