import cv2

imgOR = cv2.imread('WIN_20180113_10_19_22_Pro.jpg')

img = cv2.resize(imgOR, (1600,900))

imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

BGR_Val = img[100]

print(BGR_Val)