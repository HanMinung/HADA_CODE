import cv2
import numpy as np

image = "C:/openCV_image/pencil.webp"

src = cv2.imread(image)

# 원본 이미지와 연산 값을 선언
# 연산 이미지 : 회색 이미지, 검은색 이미지를 사용
number1 = np.ones_like(src) * 127
number2 = np.ones_like(src) * 2

add = cv2.add(src, number1)
sub = cv2.subtract(src, number1)
mul = cv2.multiply(src, number2)
div = cv2.divide(src, number2)

src = np.concatenate((src,src,src,src), axis = 1)
number = np.concatenate((number1, number1, number2, number2), axis =1 )
dst = np.concatenate((add, sub, mul, div), axis = 1)

dst = np.concatenate((src, number, dst), axis = 0)

cv2.imshow("dst", cv2.resize(dst, dsize = (500,400)))
cv2.waitKey(0)
cv2.destroyAllWindows()
