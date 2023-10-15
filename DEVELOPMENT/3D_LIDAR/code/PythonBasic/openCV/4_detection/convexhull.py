import cv2

image = "C:/openCV_image/convexhull.webp"

def __resize(src) :
    return cv2.resize(600,540)

src = cv2.imread(image, cv2.IMREAD_COLOR)
dst = src.copy()

gray = cv2.cvtColor(src, cv2.COLOR_RGB2GRAY)
ret, binary = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)

# contour
contours, hierarchy = cv2.findContours(binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

for i in contours :
    hull = cv2.convexHull(i, clockwise = True)
    cv2.drawContours(dst, [hull], 0, (0, 0, 255), 2)

cv2.imshow("dst", dst)
cv2.waitKey(0)
cv2.destroyAllWindows()