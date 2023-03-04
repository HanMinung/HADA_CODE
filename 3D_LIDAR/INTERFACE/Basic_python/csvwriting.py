import csv
import os

w = [[1,2,3], [4,5,6], [4,1,3], [4,3,5], [1,1,1], [4,2,6]]

path = 'C:/Users/hanmu/Desktop/Camera/python_basic/basic_prac/test_1'
filename = 'test.csv'
file = os.path.join(path, filename)

# with open(file,'w', newline = '') as f : 
    
#     wt = csv.writer(f)
    
#     for i in w : 
        
#         wt.writerow(i)
        
    

os.remove(file)