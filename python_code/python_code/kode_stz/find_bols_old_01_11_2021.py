#coding=utf8
import numpy as np
import cv2
import serial
import time



#######################################
#######Настройка цветовых фильтров#####
#######################################

# HSV for blue 
hsv_min_blue = np.array((0, 0, 0), np.uint8)
hsv_max_blue = np.array((360, 255, 50), np.uint8)




##################################
#Просто цвета вывода рамок для найденых объектов,
#после настройки можно удалить####
##################################
color_orange_blue = (0, 128, 255)


#######################################
#####конфигурируем и стартуем камеру###
#######################################
    

Width = 640
Height = 480

for i in range(4):
   print('i', i)
   cap = cv2.VideoCapture(int(i))#объект камеры создается тут, иногда (например при повторном включении за сеанс или при подключении второй камеры) номер девайса (CAM_ID) должен быть изменен на другой
   cap.set(cv2.CAP_PROP_FRAME_WIDTH, Width)#объекту камеры назначается ширина ROI в пикселях
   cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Height)#объекту камеры назначается высота ROI в пикселях
   time.sleep(3)
   ret, img = cap.read()
   if ret == True:
       print('ret', ret)
       break
   else:
       print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')




###################################
#######Назначаем номера цветов#####
###################################
Red_ID = 0
Green_ID = 1
Blue_ID = 2



count_frame = 0

start_time = time.time()

while True:
    ret, img = cap.read()
    img_0 = np.copy(img)
    img_line = np.copy(img)
    # преобразуем RGB картинку в HSV модель
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # применяем цветовой фильтр
    thresh_blue = cv2.inRange(hsv, hsv_min_blue, hsv_max_blue)


    contours_blue, hierarchy = cv2.findContours(thresh_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.imshow('thresh_blue', thresh_blue)
    #print('len(contours_blue)', len(contours_blue))    

    '''
    if len(contours_blue) != 0:
       for c in contours_blue:
           x, y, w, h = cv2.boundingRect(c)
           if w*h > 6000:                  
              cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)
              x_cent = x + int(w/2)
              y_cent = y + int(h/2)                 
              cv2.circle(img, (x_cent, y_cent), 5, color_orange_blue, 2)
              cv2.putText(img, "%d-%d" % (x_cent, y_cent), (x+10,y-10),cv2.FONT_HERSHEY_SIMPLEX, 1, color_orange_blue, 2)
    '''

    ###########################################################
    bilateral_filtered_image = cv2.bilateralFilter(img_0, 5, 175, 175)
    detected_image = cv2.Canny(bilateral_filtered_image, 75, 200) # 75, 200)

    cv2.imshow('RAW', img_0)
    cv2.imshow('bilateral_filtered_image', bilateral_filtered_image)
    cv2.imshow('detected_image', detected_image)

    contours, hierarchy = cv2.findContours(detected_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contour_list = []
    for contour in contours:
        approx = cv2.approxPolyDP(contour,0.01*cv2.arcLength(contour,True),True)
        area = cv2.contourArea(contour)
        if ((len(approx) > 11) & (area > 777) ):
           contour_list.append(contour)
    
    cv2.drawContours(img, contour_list,  -1, (255,0,0), 2)
    cv2.imshow('result', img)


    gray_image = cv2.cvtColor(img_line, cv2.COLOR_BGR2GRAY)
    ret, threshold_image = cv2.threshold(gray_image,80,255,0)
    threshold_image = cv2.bitwise_not(threshold_image)

    cv2.imshow('threshold_image', threshold_image) 



 
    ch = cv2.waitKey(1)

    count_frame = count_frame + 1
    if ch == 27:
        end_time = time.time()
        print("Работа успешно завершена, средний FPS = ", count_frame/(end_time - start_time))
        break
  
cap.release()
cv2.destroyAllWindows()


