import cv2
import numpy as np

def mouse_event(event, x, y, flags, param):
    global radius
    
    if event == cv2.EVENT_FLAG_LBUTTON:    
        cv2.circle(param, (x, y), radius, (255, 0, 0), 2)
        cv2.imshow("draw", src)

    elif event == cv2.EVENT_MOUSEWHEEL:
        if flags > 0:
            radius += 1
        elif radius > 1:
            radius -= 1

# 반지름을 저장할 변수와 원본 이미지를 선언
radius = 3
src = np.full((500, 500, 3), 255, dtype=np.uint8)

cv2.imshow("draw", src)
cv2.setMouseCallback("draw", mouse_event, src)
cv2.waitKey()