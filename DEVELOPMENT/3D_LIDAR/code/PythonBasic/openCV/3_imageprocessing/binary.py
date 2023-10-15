import cv2

image = "C:/openCV_image/bird.webp"

thresh, maxval = 100, 255

src = cv2.imread(image, cv2.IMREAD_COLOR)
gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

# 수식 : dst = (src>thresh)? maxval : 0
ret, dst = cv2.threshold(gray, thresh, maxval, cv2.THRESH_BINARY)

cv2.imshow("dst", dst)
cv2.waitKey()
cv2.destroyAllWindows()