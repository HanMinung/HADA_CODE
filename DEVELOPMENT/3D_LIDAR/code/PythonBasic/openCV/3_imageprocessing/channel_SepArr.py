import cv2

image = "C:/openCV_image/meat.webp"

src = cv2.imread(image, cv2.IMREAD_ANYCOLOR)
b, g, r = cv2.split(src)
inverse = cv2.merge((b, g, r))

cv2.imshow("b", cv2.resize(b, dsize = (400,300)))
cv2.imshow("g", cv2.resize(g, dsize = (400,300)))
cv2.imshow("r", cv2.resize(r, dsize = (400,300)))
cv2.imshow("inv", cv2.resize(inverse, dsize = (400,300)))
cv2.waitKey()
cv2.destroyAllWindows()