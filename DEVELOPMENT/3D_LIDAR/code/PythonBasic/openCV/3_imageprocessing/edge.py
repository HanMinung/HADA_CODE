import cv2

image = "C:/openCV_image/weat.webp"

src = cv2.imread(image,cv2.IMREAD_ANYCOLOR)

gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

sobel = cv2.resize(cv2.Sobel(gray, cv2.CV_8U, 1, 0, 3), dsize = (480,320))
laplacian = cv2.resize(cv2.Laplacian(gray, cv2.CV_8U, ksize = 3), dsize = (480,320))
canny = cv2.resize(cv2.Canny(src, 100, 255), dsize = (480,320))

cv2.imshow("sobel", sobel)
cv2.imshow("laplacian", laplacian)
cv2.imshow("canny", canny)
cv2.waitKey()
cv2.destroyAllWindows()