import cv2
import numpy as np

image = "C:/openCV_image/morphology.webp"

def __resize(src) :
    return cv2.resize(src, dsize = (1000,480))

src = cv2.imread(image)

kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (9, 9))
dilate = cv2.dilate(src, kernel, anchor=(-1, -1), iterations = 5)
erode = cv2.erode(src, kernel, anchor=(-1, -1), iterations = 5)

dst = np.concatenate((src, dilate, erode), axis = 1)

cv2.imshow('dst', __resize(dst))
cv2.waitKey(0)
cv2.destroyAllWindows()