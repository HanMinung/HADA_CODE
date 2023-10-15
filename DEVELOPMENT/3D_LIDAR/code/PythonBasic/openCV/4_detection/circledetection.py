import cv2
import numpy as np

image = "C:/Users/hanmu/Desktop/3D_LIDAR/calibration/image,PCD/image2.jpg"
centers = []

src = cv2.imread(image)

x_roi, y_roi, w_roi, h_roi = cv2.selectROI(src)
roi = src[y_roi:y_roi+h_roi, x_roi:x_roi+w_roi]

gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=50, param1=50, param2=30, minRadius=0, maxRadius=0)

if circles is not None:
    
    circles = np.round(circles[0, :]).astype("int")
    
    for (x, y, r) in circles:
        
        centers.append((x + x_roi,y + y_roi))
        cv2.circle(src, (x + x_roi, y + y_roi), r, (0, 255, 0), 2)
        cv2.circle(src, (x + x_roi, y + y_roi), 2, (255, 255, 255), 10)
        
print("Center coordinates of all detected circles")

for idx in centers : 
    print(idx)

cv2.imshow('original', src)
cv2.waitKey(0)
cv2.destroyAllWindows()