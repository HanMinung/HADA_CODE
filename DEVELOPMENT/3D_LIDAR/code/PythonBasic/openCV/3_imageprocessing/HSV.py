import cv2

image = "C:/openCV_image/tomato.webp"

src = cv2.imread(image,cv2.IMREAD_ANYCOLOR)

hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(hsv)

cv2.imshow("src",cv2.resize(src, dsize = (340,300)))
# cv2.imshow("hsv",cv2.resize(hsv, dsize = (480,420)))
cv2.imshow("H",cv2.resize(h, dsize = (340,300)))
cv2.imshow("S",cv2.resize(s, dsize = (340,300)))
cv2.imshow("V",cv2.resize(v, dsize = (340,300)))

cv2.waitKey()
cv2.destroyAllWindows()

