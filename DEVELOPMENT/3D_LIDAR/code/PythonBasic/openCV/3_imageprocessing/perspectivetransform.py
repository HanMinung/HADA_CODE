import cv2
import numpy as np

image = image = "C:/openCV_image/harvest.webp"

def re_size(src) : 
    
    return cv2.resize(src, dsize = (400,300))

src = cv2.imread(image, cv2.IMREAD_COLOR)
height, width, channel = src.shape

srcPoint = np.array([[300, 200], [400, 200], [500, 500], [200, 500]], dtype = np.float32)
dstPoint = np.array([[0, 0], [width, 0], [width, height], [0, height]], dtype = np.float32)

matrix = cv2.getPerspectiveTransform(srcPoint, dstPoint)
dst = cv2.warpPerspective(src, matrix, (width, height))

cv2.imshow("src",re_size(src))
cv2.imshow("dst", re_size(dst))
cv2.waitKey()
cv2.destroyAllWindows()