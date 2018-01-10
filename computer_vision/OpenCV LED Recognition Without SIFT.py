import cv2
import numpy as mp
import matplotlib.pyplot as plt

MIN_MATCH_COUNT = 20
MAX_H_VALUE_FOR_RED = 17
MIN_H_VALUE_FOR_BLUE = 97

imgCL_Original = cv2.imread('Final Light Test.jpg',cv2.IMREAD_COLOR) #RED LED is on the LEFT
imgGS_Original = cv2.imread('Final Light Test.jpg',cv2.IMREAD_GRAYSCALE) #RED LED is on the LEFT

#imgCL_Original = cv2.imread('Final Test 2.jpg',cv2.IMREAD_COLOR) #RED LED is on the RIGHT
#imgGS_Original = cv2.imread('Final Test 2.jpg',cv2.IMREAD_GRAYSCALE) #RED LED is on the RIGHT

#imgCL_Original = cv2.imread('Blue LED.jpg',cv2.IMREAD_COLOR) #Only Blue LED Present
#imgGS_Original = cv2.imread('Blue LED.jpg',cv2.IMREAD_GRAYSCALE) #Only Blue LED Present

#imgCL_Original = cv2.imread('Blue LED.jpg',cv2.IMREAD_COLOR) #No LED Present
#imgGS_Original = cv2.imread('Blue LED.jpg',cv2.IMREAD_GRAYSCALE) #No LED Present

LED_CL_Training_Original =  cv2.imread('IMG_20180107_122328.jpg',cv2.IMREAD_COLOR)
LED_GS_Training_Original =  cv2.imread('IMG_20180107_122328.jpg',cv2.IMREAD_GRAYSCALE)

imgCL = cv2.resize(imgCL_Original, (1600,900))
imgGS = cv2.resize(imgGS_Original, (1600,900))
LED_CL_Training = cv2.resize(LED_CL_Training_Original, (1600,900))
LED_GS_Training = cv2.resize(LED_GS_Training_Original, (1600,900))

retvalGS, thresholdGS = cv2.threshold(imgGS, 190, 255, cv2.THRESH_BINARY)
retval, thresholdCL = cv2.threshold(imgCL, 190, 255, cv2.THRESH_BINARY)
retvalGST, thresholdGST = cv2.threshold(LED_GS_Training, 220, 255, cv2.THRESH_BINARY)
retvalCLT, thresholdCLT = cv2.threshold(LED_CL_Training, 220, 255, cv2.THRESH_BINARY)


maskedTest = cv2.bitwise_and(imgCL,imgCL,mask = thresholdGS)
masked_LED_Training = cv2.bitwise_and(LED_CL_Training,LED_CL_Training,mask = thresholdGST)


orb = cv2.ORB_create()
kp1, des1 = orb.detectAndCompute(masked_LED_Training, None)
kp2, des2 = orb.detectAndCompute(maskedTest, None)

bf = cv2.BFMatcher()
#bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)

matches = bf.knnMatch(des1,des2, k = 2)
#matches = bf.match(des1,des2)
#matches = sorted(matches, key = lambda m:m.distance)



print ('Total Matches ', len(matches))

tp = []
qp = []

goodMatchs = []
for m,n in matches:
	if m.distance < 0.7*n.distance:
		goodMatchs.append([m])	
			
		tp.append(kp1[m.trainIdx].pt)
		qp.append(kp2[m.queryIdx].pt)

print ('Good Matches ', len(goodMatchs))
#print(qp)


#Matched_Images = cv2.drawMatches(masked_LED_Training,kp1,maskedTest,kp2, matches[:100], goodMatchs, flags = 2)
Matched_Images = cv2.drawMatchesKnn(masked_LED_Training, kp1, maskedTest, kp2, goodMatchs, None, flags = 2)
imgHSV = cv2.cvtColor(imgCL,cv2.COLOR_BGR2HSV)
if len(goodMatchs) > MIN_MATCH_COUNT:
	print ("LED Strip Present")
	points = []
	point_color = []
	RED_POINTS = []
	RED_X_Values = []
	BLUE_POINTS = []
	BLUE_X_Values = []
	for point in qp:
		x = round(point[0]),round(point[1])
		#points.append(x)
		cv2.circle(maskedTest,x,3,(0,0,255),-1)
		HSV_color = (imgHSV[x[1], x[0]])
		point_color.append([HSV_color])
		#print(x)
		#print(HSV_color)
		if HSV_color[0] < MAX_H_VALUE_FOR_RED:
			RED_POINTS.append(x)
			RED_X_Values.append(x[0])
			cv2.putText(maskedTest,"RED Match", x , cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),2,cv2.LINE_AA)
		elif HSV_color[0] > MIN_H_VALUE_FOR_BLUE:
			BLUE_POINTS.append(x)
			BLUE_X_Values.append(x[0])
			cv2.putText(maskedTest,"BLUE Match", x , cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0),2,cv2.LINE_AA)
		#else:
			#print('Extraneous Point')

	print('Blue Matches', BLUE_POINTS)
	print('Red Matches', RED_POINTS)

	MEAN_RED_X_VALUES = sum(RED_X_Values)/len(RED_X_Values)
	MEAN_BLUE_X_VALUES = sum(BLUE_X_Values)/len(BLUE_X_Values)
	print('Mean Red x values: ', MEAN_RED_X_VALUES)
	print('Mean Blue x values: ', MEAN_BLUE_X_VALUES)
	if MEAN_RED_X_VALUES < MEAN_BLUE_X_VALUES:
		print('The Red LED Strip Is On the Left')
		print('The Blue LED Strip Is On the Right')
	else:
		print('The Blue LED Strip Is On the Left')
		print('The Red LED Strip Is On the Right')
		
	#print(point_color)	
	Matched_HSV_H_Values = []
	for HSV_Value in point_color:
		blah = HSV_Value[0]
		H = blah[0]
		Matched_HSV_H_Values.append(H)
		#print(H)

	Matched_HSV_H_Values = sorted(Matched_HSV_H_Values)
	#print(Match_HSV_H_Values)

else:
	print ("LED Strip Not Present")



#cv2.imshow('COLOR', thresholdCL)
#cv2.imshow('GrayScale', thresholdGS)
cv2.imshow('MaskedImg_CL',maskedTest)
#cv2.imshow('masked_LED_Training',masked_LED_Training)
#cv2.imshow('Matched_Images',Matched_Images)
