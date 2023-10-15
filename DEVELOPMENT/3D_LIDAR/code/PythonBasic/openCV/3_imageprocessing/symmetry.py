import cv2

image = "C:/openCV_image/cup.webp"

src = cv2.imread(image,cv2.IMREAD_ANYCOLOR)

XY, X, Y = -1, 0, 1                 # FOR symmetry
height, width, source = src.shape   # FOR rotation

rot_ang, rot_sca = 90, 1
# Symmetry
SYM = cv2.flip(src,X)

# Creating rotation matrix
matrix = cv2.getRotationMatrix2D((width/2,height/2),rot_ang,rot_sca)
# 아핀 변환 함수
ROT = cv2.warpAffine(src,matrix,(width,height))

cv2.imshow("Original", src)
cv2.imshow("Symmetry", SYM)
cv2.imshow("Rotation", ROT)

cv2.waitKey()
cv2.destroyAllWindows()