import cv2

image = "C:/openCV_image/contour.webp"
B, G, R = 0, 0, 255
thresh, maxval = 127, 255

def __resize(src) :
    return cv2.resize(src, dsize = (480,440))

src = cv2.imread(image, cv2.IMREAD_COLOR)

# 이진화 처리를 하여, 반전시켜 검출하려는 물체를 하얀색의 성질을 띄도록 변환
gray = cv2.cvtColor(src, cv2.COLOR_RGB2GRAY)
ret, binary = cv2.threshold(gray, thresh, maxval, cv2.THRESH_BINARY)
binary = cv2.bitwise_not(binary)

# cv2.imshow("threshhold result", __resize(binary))
# cv2.waitKey()

contours, hierarchy = cv2.findContours(binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

for i in range(len(contours)):
    
    cv2.drawContours(src, [contours[i]], 0, (B, G, R), 2)
    cv2.putText(src, str(i), tuple(contours[i][0][0]), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 255, 0), 1)
    print(i, hierarchy[0][i])
    cv2.imshow("src", src) 
    cv2.waitKey(0)