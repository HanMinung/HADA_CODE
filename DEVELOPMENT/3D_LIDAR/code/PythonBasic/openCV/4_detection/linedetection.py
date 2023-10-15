import cv2
import numpy as np

image = "C:/openCV_image/linedetection.webp"

src = cv2.imread(image)
dst = src.copy()

# 전처리를 진행하기 위해 gray scale image / canny edge image를 사용
gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
canny = cv2.Canny(gray, 5000, 1500, apertureSize = 5, L2gradient = True)
lines = cv2.HoughLinesP(canny, 0.8, np.pi/180, 90, minLineLength = 10, maxLineGap = 100)

for i in lines:
    cv2.line(dst, (int(i[0][0]), int(i[0][1])), (int(i[0][2]), int(i[0][3])), (0, 0, 255), 2)

cv2.imshow("dst", cv2.resize(dst, dsize = (1000,800)))
cv2.waitKey(0)
cv2.destroyAllWindows()