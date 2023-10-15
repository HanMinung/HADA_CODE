import cv2

image = "C:/openCV_image/tomato.webp"

src = cv2.imread(image, cv2.IMREAD_ANYCOLOR)
hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(hsv)

# 빨간색 영역을 검출
lower_red = cv2.inRange(hsv, (0, 100, 100), (5, 255, 255))
upper_red = cv2.inRange(hsv, (170, 100, 100), (180, 255, 255))
added_red = cv2.addWeighted(lower_red, 1.0, upper_red, 1.0, 0.0)

red = cv2.bitwise_and(hsv, hsv, mask = added_red)
red = cv2.cvtColor(red, cv2.COLOR_HSV2BGR)

cv2.imshow("src", cv2.resize(src, dsize = (480,300)))
cv2.imshow("red", cv2.resize(red, dsize = (480,300)))
cv2.waitKey()
cv2.destroyAllWindows()