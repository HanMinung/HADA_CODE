import cv2

image_1 = "C:/openCV_image/hats.webp"
image_2 = "C:/openCV_image/hat.webp"

def __resize(src) :
    return cv2.resize(src, dsize = (600, 480))

src = cv2.imread(image_1, cv2.IMREAD_GRAYSCALE)
templit = cv2.imread(image_2, cv2.IMREAD_GRAYSCALE)
dst = cv2.imread(image_1)

result = cv2.matchTemplate(src, templit, cv2.TM_SQDIFF_NORMED)

minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(result)
x, y = minLoc
h, w = templit.shape

dst = cv2.rectangle(dst, (x, y), (x +  w, y + h) , (0, 0, 255), 3)
cv2.imshow("dst", __resize(dst))
cv2.waitKey(0)
cv2.destroyAllWindows()