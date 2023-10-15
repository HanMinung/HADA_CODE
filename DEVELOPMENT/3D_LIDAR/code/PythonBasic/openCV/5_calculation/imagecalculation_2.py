import numpy as np
import cv2

image = "C:/openCV_image/pencil.webp"

src = cv2.imread(image)
number = np.ones_like(src) * 127

_max = cv2.max(src, number)
_min = cv2.min(src, number)
_abs = cv2.absdiff(src, number)
compare = cv2.compare(src, number, cv2.CMP_GT)

src = np.concatenate((src, src, src, src), axis = 1)
number = np.concatenate((number, number, number, number), axis = 1)
dst = np.concatenate((_max, _min, _abs, compare), axis = 1)

dst = np.concatenate((src, number, dst), axis = 0)

cv2.imshow("dst", cv2.resize(dst, dsize = (800,600)))
cv2.waitKey(0)
cv2.destroyAllWindows()