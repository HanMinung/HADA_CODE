import cv2

image = "C:/openCV_image/cornertracking.webp"

def __resize(src) :
    return cv2.resize(src, dsize = (600,480))

src = cv2.imread(image, cv2.IMREAD_COLOR)
dst = src.copy()
gray = cv2.cvtColor(src, cv2.COLOR_RGB2GRAY)

corners = cv2.goodFeaturesToTrack(gray, 100, 0.01, 5, blockSize=3, useHarrisDetector = True, k=0.03)


for i in corners:
    cv2.circle(dst, tuple(i[0]), 3, (0, 0, 255),cv2.FILLED, cv2.LINE_4)

cv2.imshow("src", __resize(src))
cv2.imshow("dst", __resize(dst))
cv2.waitKey(0)
cv2.destroyAllWindows()