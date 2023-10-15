import cv2

image = "C:/openCV_image/geese.webp"

ksize, point = (9, 9), (-1, -1)

src = cv2.imread(image, cv2.IMREAD_COLOR)

dst = cv2.blur(src, ksize, anchor = point, borderType=cv2.BORDER_DEFAULT)

src = cv2.resize(src, dsize = (480,380))
dst = cv2.resize(dst, dsize = (480,380))

cv2.imshow("src",src)
cv2.imshow("dst", dst)
cv2.waitKey()
cv2.destroyAllWindows()