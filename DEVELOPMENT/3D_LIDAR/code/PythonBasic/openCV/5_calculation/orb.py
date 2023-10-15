import cv2
import numpy as np

src = cv2.imread("C:/openCV_image/apple_books.webp")
org = src.copy()
gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
target = cv2.imread("C:/openCV_image/apple.webp",cv2.IMREAD_GRAYSCALE)

def __resize(src) : 
    return cv2.resize(src, dsize = (1000,600))

orb = cv2.ORB_create(
    nfeatures = 40000,
    scaleFactor = 1.2,
    nlevels = 8,
    edgeThreshold = 31,
    firstLevel = 0,
    WTA_K = 2,
    scoreType = cv2.ORB_HARRIS_SCORE,
    patchSize = 31,
    fastThreshold = 20,
)


kp1, des1 = orb.detectAndCompute(gray, None)
kp2, des2 = orb.detectAndCompute(target, None)

bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(des1, des2)
matches = sorted(matches, key=lambda x: x.distance)

for i in matches[:100]:
    idx = i.queryIdx
    x1, y1 = kp1[idx].pt
    cv2.circle(src, (int(x1), int(y1)), 3, (255, 0, 0), 3)

com = np.concatenate((org, src), axis = 1)
cv2.imshow("src", __resize(com))
cv2.waitKey()


