import numpy as numpy
import cv2 as cv



def getContours(binary_image):      
    contours, hierarchy = cv.findContours(binary_image.copy(), 
                                            cv.RETR_EXTERNAL,
	                                        cv.CHAIN_APPROX_SIMPLE)
    return contours

def get_contour_center(contour):
    M = cv.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy

def draw(contours):
    for c in contours:
        # if(cv.contourArea(c)>100):
            # print(cv.contourArea(c))
            # cv.drawContours(img,c,-1, [0, 255, 0], 3)
            cx, cy = get_contour_center(c)
            print(str(cx)+" "+str(cy))
            # pose_estimation(cx,cy)
            # turn_estimation(cx)

yellow_lower=(0, 123, 165)
yellow_upper=(25.1,216,255)


# yellow_lower=(20,143,221)
# yellow_upper=(24,195,255)













img=cv.imread("yellow_detect.jpeg")
hsv=cv.cvtColor(img,cv.COLOR_BGR2HSV)
blurred = cv.GaussianBlur(hsv, (5, 5), 0)
mask=cv.inRange(blurred,yellow_lower,yellow_upper)
contours=getContours(mask)
draw(contours)
cv.imshow("display Img",img)
cv.imshow("mask",mask)
k=cv.waitKey(0)
