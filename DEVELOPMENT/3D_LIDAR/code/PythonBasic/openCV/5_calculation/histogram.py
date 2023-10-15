import numpy as np
import cv2

image = "C:/openCV_image/histogram.webp"

src = cv2.imread(image)
gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
result = np.zeros((src.shape[0], 256), dtype=np.uint8)

hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
cv2.normalize(hist, hist, 0, result.shape[0], cv2.NORM_MINMAX)

for x, y in enumerate(hist):
    cv2.line(result, (int(x), result.shape[0]), (int(x), result.shape[0] - int(y)), 255)

dst = np.hstack([gray, result])

cv2.imshow("dst", cv2.resize(dst, dsize = (800,480)))
cv2.waitKey(0)
cv2.destroyAllWindows()