import numpy as np
import cv2



h_s = 30#shirina zoni vdol kraya kadra
r_s = 15#shirina zoni proverki tcentra linii
calibration_line_tikcness = 200# nastroyka chuvstvitelnosti k chernomu (tolshine linii)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

ret, image = cap.read()



'''
h = image.shape[0]
w = image.shape[1]
x_center = int(w/2)
y_center = int(h/2)
print('h, w', h, w)
'''

def line_detect(image, threshold_image, x_1, y_1, x_2, y_2, color):
    x_cent = int((x_1 + x_2)/2)
    y_cent = int((y_1 + y_2)/2)
    #print('h, w', h, w)
    #print('x_cent', x_cent)
    #print('y_cent', y_cent)

    x1_cent = x_cent - r_s
    if x1_cent < 0:
       x1_cent = 0
    if x1_cent > w:
       x1_cent = w

    x2_cent = x_cent + r_s
    if x2_cent < 0:
       x2_cent = 0
    if x2_cent > w:
       x2_cent = w

    y1_cent = y_cent - r_s
    if y1_cent < 0:
       y1_cent = 0
    if y1_cent > h:
       y1_cent = h

    y2_cent = y_cent + r_s
    if y2_cent < 0:
       y2_cent = 0
    if y2_cent > h:
       y2_cent = h

    if x1_cent > x2_cent:
       mem = x2_cent
       x2_cent = x1_cent
       x1_cent = mem
    if y1_cent > y2_cent:
       mem = y2_cent
       y2_cent = y1_cent
       y1_cent = mem

    #print('x1_cent : x2_cent, y1_cent : y2_cent', x1_cent , x2_cent, y1_cent , y2_cent)
    slase_cent = threshold_image[y1_cent  : y2_cent, x1_cent : x2_cent]
    #print('type(slase_cent)', type(slase_cent))
    #print('type(slase_cent)', slase_cent)
    slase_cent_1 = np.copy(slase_cent)
    #print('type(slase_cent_1)', type(slase_cent_1))
    slase_cent_1 = cv2.bitwise_not(slase_cent_1)
    #print('type(slase_cent_1)', type(slase_cent_1))

    #cv2.imshow('slase_cent_1', slase_cent_1)
    #cv2.waitKey(100)

    slase_cent_sum = int(np.sum(slase_cent)/255)
    #print('color', color)
    #print('slase_cent_sum', slase_cent_sum)
    calibration_line_cent = int(abs((x2_cent - x1_cent))*abs((y2_cent - y1_cent))/3)
    #print('calibration_line_cent', calibration_line_cent)
    if slase_cent_sum >= calibration_line_cent: 
       point = 1
       cv2.circle(image, (x_cent, y_cent), 10, color, -1)
    else: 
       point = 0

    #threshold_image[y_cent-int(slase_cent_1.shape[0]/2) : y_cent + slase_cent_1.shape[0] -int(slase_cent_1.shape[0]/2), x_cent - int(slase_cent_1.shape[1]/2) : x_cent+slase_cent_1.shape[1]- int(slase_cent_1.shape[1]/2)] = slase_cent_1
    return point, x_cent, y_cent

def line_show(image, threshold_image, x_1, y_1, x_2, y_2, color):
    point_1, x_cent_1, y_cent_1 = line_detect(image, threshold_image, x_1, y_1, x_2, y_2, color)#midle
    point_2, x_cent_2, y_cent_2 = line_detect(image, threshold_image, x_1, y_1, x_cent_1, y_cent_1, color)#start to midle
    point_3, x_cent_3, y_cent_3 = line_detect(image, threshold_image, x_cent_1, y_cent_1, x_2, y_2, color)#midle to end

    point_4, x_cent_4, y_cent_4 = line_detect(image, threshold_image, x_1, y_1, x_cent_2, y_cent_2, color)#start to (start to midle)
    point_5, x_cent_5, y_cent_5 = line_detect(image, threshold_image, x_cent_2, y_cent_2, x_cent_1, y_cent_1, color)
    point_6, x_cent_6, y_cent_6 = line_detect(image, threshold_image, x_cent_1, y_cent_1, x_cent_3, y_cent_3, color)
    point_7, x_cent_7, y_cent_7 = line_detect(image, threshold_image, x_cent_3, y_cent_3, x_2, y_2, color)


    point_all = np.array([point_1, point_2, point_3, point_4, point_5, point_6, point_7])

    point_sum = np.sum(point_all)
    if point_sum >= point_all.shape[0]-3: # - kolvo tochek, kot moget ne bit
       
       line = cv2.line(image, (x_1, y_1), (x_2, y_2), color, 1)
       cv2.imshow('threshold_image', threshold_image)
 

  

while True:
    ret, image = cap.read()
    if ret == True:
       #cv2.imshow('camera', image)
       gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
       ret, threshold_image = cv2.threshold(gray_image,80,255,0)
       threshold_image = cv2.bitwise_not(threshold_image)
       F_left = threshold_image[0 : h, 0 : h_s]
       F_top = threshold_image[0 : h_s, 0 : w]
       F_right = threshold_image[0 : h, (w - h_s) : w]
       F_bottom = threshold_image[(h - h_s) : h, 0 : w]

       # calculate mometn of image
       moments_F_top = cv2.moments(F_top, 1)
       dM01_F_top = moments_F_top['m01']
       dM10_F_top = moments_F_top['m10']
       dArea_F_top = moments_F_top['m00']

       moments_F_bottom = cv2.moments(F_bottom, 1)
       dM01_F_bottom = moments_F_bottom['m01']
       dM10_F_bottom = moments_F_bottom['m10']
       dArea_F_bottom = moments_F_bottom['m00']

       moments_F_left = cv2.moments(F_left, 1)
       dM01_F_left = moments_F_left['m01']
       dM10_F_left = moments_F_left['m10']
       dArea_F_left = moments_F_left['m00']

       moments_F_right = cv2.moments(F_right, 1)
       dM01_F_right = moments_F_right['m01']
       dM10_F_right = moments_F_right['m10']
       dArea_F_right = moments_F_right['m00']
       
       x_top = 0
       y_top = 0
       x_bottom = 0
       y_bottom = 0
       x_right = 0
       y_right = 0
       x_left = 0
       y_left = 0


       if dArea_F_top > calibration_line_tikcness:# 
           x_top = int(dM10_F_top / dArea_F_top)
           y_top = int(dM01_F_top / dArea_F_top)
           cv2.circle(image, (x_top, y_top), 10, (128,128,128), -1)
       if dArea_F_bottom > calibration_line_tikcness:
           x_bottom = int(dM10_F_bottom / dArea_F_bottom) 
           y_bottom = int(dM01_F_bottom / dArea_F_bottom) + (h - h_s)
           cv2.circle(image, (x_bottom, y_bottom), 10, (128,128,128), -1)
       if dArea_F_right > calibration_line_tikcness:
           x_right = int(dM10_F_right / dArea_F_right) + (w - h_s)
           y_right = int(dM01_F_right / dArea_F_right)
           cv2.circle(image, (x_right, y_right), 10, (128,128,128), -1)
       if dArea_F_left > calibration_line_tikcness:
           x_left = int(dM10_F_left / dArea_F_left)
           y_left = int(dM01_F_left / dArea_F_left)
           cv2.circle(image, (x_left, y_left), 10, (128,128,128), -1)

       if dArea_F_top > calibration_line_tikcness and dArea_F_bottom > calibration_line_tikcness:
           color = (255,0,0)#red
           line_show(image, threshold_image, x_top, y_top, x_bottom, y_bottom, color)

       if dArea_F_left > calibration_line_tikcness and dArea_F_right > calibration_line_tikcness:
           color = (250,255,0)#yellow
           line_show(image, threshold_image, x_left, y_left, x_right, y_right, color)

       if dArea_F_left > calibration_line_tikcness and dArea_F_bottom > calibration_line_tikcness:
           color = (0,255,0)#green
           line_show(image, threshold_image, x_left, y_left, x_bottom, y_bottom, color)

       if dArea_F_right > calibration_line_tikcness and dArea_F_bottom > calibration_line_tikcness:
           color = (0,0,255)#blue
           line_show(image, threshold_image, x_right, y_right, x_bottom, y_bottom, color)

       if dArea_F_left > calibration_line_tikcness and dArea_F_top > calibration_line_tikcness:
           color = (242,0,255)#purple
           line_show(image, threshold_image, x_left, y_left, x_top, y_top, color)

       if dArea_F_right > calibration_line_tikcness and dArea_F_top > calibration_line_tikcness:
           color = (0,255,255)#light blue
           line_show(image, threshold_image, x_right, y_right, x_top, y_top, color)

       #cv2.imshow('F_left', F_left)
       #cv2.imshow('F_top', F_top)
       #cv2.imshow('F_right', cv2.bitwise_not(F_right))
       #cv2.imshow('F_bottom', F_bottom)

       cv2.imshow('camera', image)
       cv2.waitKey(100)
    else:
       pass


#cv2. destroyAllWindows()

