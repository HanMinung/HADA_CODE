import cv2

image = "C:/openCV_image/adaptivethreshhold.webp"

def __resize(src) :
    return cv2.resize(src, dsize = (600, 480))

src = cv2.imread(image, cv2.COLOR_BGR2GRAY)

gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 467, 37)

cv2.imshow("adaptive binary", __resize(binary))
cv2.waitKey(0)
cv2.destroyAllWindows()