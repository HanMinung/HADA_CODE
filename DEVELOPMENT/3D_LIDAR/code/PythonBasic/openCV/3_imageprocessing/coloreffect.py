import cv2

image = "C:/openCV_image/bird.webp"

src = cv2.imread(image,cv2.IMREAD_ANYCOLOR)
cvt = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
rev = cv2.bitwise_not(src)

src = cv2.resize(src, dsize=(480,320))
cvt = cv2.resize(cvt, dsize=(480,320))
rev = cv2.resize(rev, dsize=(480,320))

cv2.imshow("src",src)
cv2.imshow("cvt",cvt)
cv2.imshow("rev",rev)

cv2.waitKey()
cv2.destroyAllWindows()