import cv2
import numpy as np

# 0 - 255 까지 90000개의 points
gray = np.linspace(0, 255, num=90000, endpoint = True, retstep = False, dtype = np.uint8).reshape(300, 300, 1)
color = np.zeros((300,300,3), np.uint8)

#  color : 색상 그라데이션 이미지
#  0 ~ 150행에 gray 값을 할당, red 채널의 150 ~ 300 열에 gray 값을 할당
color[0:150, :,0] = gray[0:150, :,0]
color[:, 150:300, 2] = gray[:, 150:300, 0]

x, y, c = 200, 100, 0
access_gray  = gray[y, x, c]
access_color_blue = color[y, x, c]
access_color = color[y, x]

cv2.imshow("gray", gray)
cv2.imshow("color", color)
cv2.waitKey(0)
cv2.destroyAllWindows()