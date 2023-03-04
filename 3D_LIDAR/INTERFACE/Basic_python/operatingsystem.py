import os
import time
from datetime import datetime

dir = 'C:/Users/hanmu/Desktop/Camera/pythonprac/test'

for idx in range(3) :
    
    file_fmt = os.path.join(dir, '%Y-%m-%d_%H%M')
    path = str(datetime.now().strftime(file_fmt)) + '_' + str(idx) +'.txt'

    print('save to', path)
    fp = open(path, 'ab')