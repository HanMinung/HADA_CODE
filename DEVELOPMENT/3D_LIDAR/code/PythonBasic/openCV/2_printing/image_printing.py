import cv2

path = "C:/openCV_image/lunar.webp"         # absolute path

image = cv2.imread(path,cv2.IMREAD_ANYCOLOR)
cv2.imshow("MOON",image)
cv2.waitKey()
cv2.destroyAllWindows()

height, width, channel = image.shape
print(height, width, channel) 