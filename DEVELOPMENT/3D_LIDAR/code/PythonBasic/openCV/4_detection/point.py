import cv2
import numpy as np

image = "C:/Users/hanmu/Desktop/3D_LIDAR/calibration/image,PCD/image.png"

src = cv2.imread(image)

# dst = cv2.resize(src, dsize=(640, 480), interpolation=cv2.INTER_AREA)

def get_coordinates(event, x, y, flags, param):
    
    if event == cv2.EVENT_LBUTTONDBLCLK:
        
        print("Pixel coordinates (x, y):", x, y)

cv2.namedWindow("image")

cv2.setMouseCallback("image", get_coordinates)

cv2.imshow("image", src)

cv2.waitKey(0)

cv2.destroyAllWindows()