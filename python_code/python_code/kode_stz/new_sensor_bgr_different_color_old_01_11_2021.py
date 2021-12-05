#coding=utf8
import numpy as np
import cv2
import serial
import time
import sys
import cord_corr
from cord_corr import calc_cord_object

#######################################
########## Ф У Н К Ц И И ##############
#######################################




########Работа с портом######
def OPEN_SERIAL_PORT():
    ser = serial.Serial(
		# port='/dev/ttyACM0', #port='/dev/ttyACM0',
        port = 'com8',
		baudrate = 115200,
		bytesize = serial.EIGHTBITS,
		parity = serial.PARITY_NONE,
		stopbits = serial.STOPBITS_ONE,
		xonxoff = False,
		rtscts = False,
		dsrdtr = False,
		timeout = 0.1)
    return ser

###### Отправка сообщений по СОМ порту #######
def SEND_MESANGE_TO_COM_PORT(Color_ID, x_cord, y_cord):

    send_cord_X = x_cord#Здесь можно преобразовать координаты перед отправкой
    send_cord_Y = y_cord#Здесь можно преобразовать координаты перед отправкой
    msg="<"+"1"+","+str(Color_ID)+","+str(send_cord_X)+","+str(send_cord_Y)+">"
    print("Send data in COM : ", "Color_ID = ", Color_ID, ",", "x_cord = ", x_cord, ",", "y_cord = ", y_cord)

    # ser.write(msg.encode())

######### Детектор линии#########
def line_detect(img, threshold_image, x_1, y_1, x_2, y_2, color):
    x_cent = int((x_1 + x_2) / 2)
    y_cent = int((y_1 + y_2) / 2)
    # print('h, w', h, w)
    # print('x_cent', x_cent)
    # print('y_cent', y_cent)

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

    # print('x1_cent : x2_cent, y1_cent : y2_cent', x1_cent , x2_cent, y1_cent , y2_cent)
    slase_cent = threshold_image[y1_cent: y2_cent, x1_cent: x2_cent]
    # print('type(slase_cent)', type(slase_cent))
    # print('type(slase_cent)', slase_cent)
    slase_cent_1 = np.copy(slase_cent)
    # print('type(slase_cent_1)', type(slase_cent_1))
    slase_cent_1 = cv2.bitwise_not(slase_cent_1)
    # print('type(slase_cent_1)', type(slase_cent_1))

    # cv2.imshow('slase_cent_1', slase_cent_1)
    # cv2.waitKey(100)

    slase_cent_sum = int(np.sum(slase_cent) / 255)
    # print('color', color)
    # print('slase_cent_sum', slase_cent_sum)
    calibration_line_cent = int(abs((x2_cent - x1_cent)) * abs((y2_cent - y1_cent)) / 3)
    # print('calibration_line_cent', calibration_line_cent)
    if slase_cent_sum >= calibration_line_cent:
        point = 1
        cv2.circle(img, (x_cent, y_cent), 10, color, -1)
    else:
        point = 0

    # threshold_image[y_cent-int(slase_cent_1.shape[0]/2) : y_cent + slase_cent_1.shape[0] -int(slase_cent_1.shape[0]/2), x_cent - int(slase_cent_1.shape[1]/2) : x_cent+slase_cent_1.shape[1]- int(slase_cent_1.shape[1]/2)] = slase_cent_1
    return point, x_cent, y_cent

############# Рисование линии ##################
def line_show(img, threshold_image, x_1, y_1, x_2, y_2, color):
    point_1, x_cent_1, y_cent_1 = line_detect(img, threshold_image, x_1, y_1, x_2, y_2, color)  # midle
    point_2, x_cent_2, y_cent_2 = line_detect(img, threshold_image, x_1, y_1, x_cent_1, y_cent_1,
                                              color)  # start to midle
    point_3, x_cent_3, y_cent_3 = line_detect(img, threshold_image, x_cent_1, y_cent_1, x_2, y_2, color)  # midle to end

    point_4, x_cent_4, y_cent_4 = line_detect(img, threshold_image, x_1, y_1, x_cent_2, y_cent_2,
                                              color)  # start to (start to midle)
    point_5, x_cent_5, y_cent_5 = line_detect(img, threshold_image, x_cent_2, y_cent_2, x_cent_1, y_cent_1, color)
    point_6, x_cent_6, y_cent_6 = line_detect(img, threshold_image, x_cent_1, y_cent_1, x_cent_3, y_cent_3, color)
    point_7, x_cent_7, y_cent_7 = line_detect(img, threshold_image, x_cent_3, y_cent_3, x_2, y_2, color)

    point_all = np.array([point_1, point_2, point_3, point_4, point_5, point_6, point_7])

    point_sum = np.sum(point_all)
    if point_sum >= point_all.shape[0] - 3:  # - kolvo tochek, kot moget ne bit

        line = cv2.line(img, (int(x_1), int(y_1)), (int(x_2), int(y_2)), color, 1)
    return img

###################################
###Вычисление центра перекрестка###
###################################
def calc_center_crossroads(x_left, y_left, x_right, y_right, x_bottom, y_bottom):
    d_x = x_right - x_left
    d_y = y_right - y_left
    x_cross = ((y_bottom - y_left) * d_x * d_y + (d_x ** 2) * x_bottom + (d_y ** 2) * x_left) / (d_y ** 2 + d_x ** 2)
    if d_x != 0:
        y_cross = (d_y / d_x) * (x_cross - x_left) + y_left
    else:
        y_cross = (d_x / d_y) * (x_cross - x_bottom) + y_bottom
    #print('x_cross, y_cross', x_cross, y_cross)
    # x_cross = int(x_cross)
    return x_cross, y_cross

###################################
###Вычисление координат вектора####
###################################
def calc_vector(M0, M1): # M0, M1 - массивы numpy
    x = M1[0] - M0[0]
    y = M0[1] - M1[1]
    # return np.array(M1 - M0)
    return np.array([x, y])
################################################
###Определение знака векторного произведения####
###Испл. для определения стороны расположения###
####### объекта относиительно прямой############
def sign_vector_multiplication(a, b):
    applicate = a[0] * b[1] - a[1] * b[0]
    #print("applicate =", applicate)
    if applicate > 0:
        return -1 # объект слева
    elif applicate < 0:
        return 1 # объект справа
    else:
        return 0 # объект на линии движения

###################################
####Анализ проезда перекрестка#####
###################################
def scenario_crossroads(top, bottom, left, right, cord):
    # Возможные варианты:
    # 0 - Прямо
    # 1 - Влево
    # 2 - Вправо
    # 3 - Назад (разворот)
    #####################
    if left[0] == 0 and left[1] == 0 and right[0] == 0 and right[1] == 0:
        return 0        # перекреска нет
    else:
        a = calc_vector(left, right)
        b = calc_vector(left, cord)
        sign = sign_vector_multiplication(a, b);
        #print("1) a, b -->", a, b, sign)
        if sign > 0:
            a = calc_vector(bottom, top)
            b = calc_vector(bottom, cord)
            #print("2) a, b -->", a, b, sign)
            sign = sign_vector_multiplication(a, b)
            if sign < 0:
                return 1    # поворот налево
            else:
                return 2    # поворот направо
        else:
            return 0        # прямо



####################################################################
####################################################################
####################################################################
#######################################
#######Настройка цветовых фильтров#####
#######################################
# HSV for blue
hsv_min_blue = np.array((85, 55, 115), np.uint8)
hsv_max_blue = np.array((125, 255, 255), np.uint8)
# HSV for green
hsv_min_green = np.array((35, 55, 115), np.uint8)   # 35,55,115
hsv_max_green = np.array((83, 255, 255), np.uint8)  # 83,255,255
# HSV for red
hsv_min_red_1 = np.array((0, 55, 115), np.uint8)
hsv_max_red_1 = np.array((10, 255, 255), np.uint8)
hsv_min_red_2 = np.array((160, 55, 115), np.uint8)
hsv_max_red_2 = np.array((179, 255, 255), np.uint8)

################################################
#####Настройка поиска линии#####################
###########################№####################
h_s = 30#shirina zoni vdol kraya kadra
r_s = 15#shirina zoni proverki tcentra linii
calibration_line_tikcness = 200# 200nastroyka chuvstvitelnosti k chernomu (tolshine linii)

##################################
#Просто цвета вывода рамок для найденых объектов,
#после настройки можно удалить####
##################################
color_orange_blue = (0, 128, 255)
color_purple_green = (255, 0, 139)
color_yellow_red = (0,255,255)

#######################################
#####конфигурируем и стартуем камеру###
#######################################
'''
0. CAP_PROP_POS_MSEC Current position of the video file in milliseconds.
1. CAP_PROP_POS_FRAMES 0-based index of the frame to be decoded/captured next.
2. CAP_PROP_POS_AVI_RATIO Relative position of the video file
3. CAP_PROP_FRAME_WIDTH Width of the frames in the video stream.
4. CAP_PROP_FRAME_HEIGHT Height of the frames in the video stream.
5. CAP_PROP_FPS Frame rate.
6. CAP_PROP_FOURCC 4-character code of codec.
7. CAP_PROP_FRAME_COUNT Number of frames in the video file.
8. CAP_PROP_FORMAT Format of the Mat objects returned by retrieve() .
9. CAP_PROP_MODE Backend-specific value indicating the current capture mode.
10. CAP_PROP_BRIGHTNESS Brightness of the image (only for cameras).
11. CAP_PROP_CONTRAST Contrast of the image (only for cameras).
12. CAP_PROP_SATURATION Saturation of the image (only for cameras).
13. CAP_PROP_HUE Hue of the image (only for cameras).
14. CAP_PROP_GAIN Gain of the image (only for cameras).
15. CAP_PROP_EXPOSURE Exposure (only for cameras).
16. CAP_PROP_CONVERT_RGB Boolean flags indicating whether images should be converted to RGB.
17. CAP_PROP_WHITE_BALANCE Currently unsupported
18. CAP_PROP_RECTIFICATION Rectification flag for stereo cameras (note: only supported by DC1394 v 2.x backend currently)
'''


Width = 640
Height = 480
for i in range(0, 4):   # Перебор камер (0 ... 4)
   print('i', i)
   cap = cv2.VideoCapture(int(i)) 
	    # cv2.VideoCapture(int(i), cv2.CAP_DSHOW)    #объект камеры
	    # создается тут, иногда (например при
            # повторном включении за сеанс или при подключении второй камеры) номер девайса
            # (CAM_ID) должен быть изменен на другой
            # параметр "cv2.CAP_DSHOW" отвечает за выдачу предупреждения "terminating async callback"
   cap.set(cv2.CAP_PROP_FRAME_WIDTH, Width)#объекту камеры назначается ширина ROI в пикселях
   cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Height)#объекту камеры назначается высота ROI в пикселях

   #print(cap.get(cv2.CAP_PROP_FOURCC))#команда дает текущий Формат кодека кадра, для awer media - YUY2 (соответствие PC98).
   
   #sys.exit()
   time.sleep(3)
   ret, img = cap.read()
   if ret == True:
       print('ret', ret)
       break
   else:
       print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')

#######################################
#####Стартуем порт#####################
#######################################
# ser = OPEN_SERIAL_PORT()
# try:
#     ser.isOpen()
# except:
#     print("serial not open")
#     exit(0)

###################################
#######Назначаем номера цветов#####
###################################
Red_ID = 0
Green_ID = 1
Blue_ID = 2

time_send_delay = 5


count_frame = 0


start_time = time.time()

while True:
    ret, img = cap.read()
    img[450:, :, :] = 255# 
    #cv2.imshow('RAW', img)
    # преобразуем RGB картинку в HSV модель
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_line = np.copy(img)

    #############################################################
    # поиск маркеров Red_ID = 0, Green_ID = 1, Blue_ID = 2
    #############################################################
    # применяем цветовой фильтр
    thresh_blue = cv2.inRange(hsv, hsv_min_blue, hsv_max_blue)
    thresh_green = cv2.inRange(hsv, hsv_min_green, hsv_max_green)
    thresh_red_1 = cv2.inRange(hsv, hsv_min_red_1, hsv_max_red_1)
    thresh_red_2 = cv2.inRange(hsv, hsv_min_red_2, hsv_max_red_2)
    thresh_red = cv2.bitwise_or(thresh_red_1, thresh_red_2)

    contours_blue, hierarchy = cv2.findContours(thresh_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.imshow('thresh_blue', thresh_blue)
    #print('len(contours_blue)', len(contours_blue))
    contours_green, hierarchy = cv2.findContours(thresh_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.imshow('thresh_green', thresh_green)
    # print('len(contours_green)', len(contours_green))
    contours_red, hierarchy = cv2.findContours(thresh_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #cv2.imshow('thresh_red', thresh_red)
    #print('len(contours_red)', len(contours_red))
    Blue = []
    Green = []
    Red = []
    if len(contours_blue) != 0:
        
        for c in contours_blue:
            x, y, w, h = cv2.boundingRect(c)
            if w > 40 and h > 40:
                cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)
                x_cent = x + int(w/2)
                y_cent = y + int(h/2)
                Blue.append([x_cent, y_cent])
                cv2.circle(img, (x_cent, y_cent), 5, color_orange_blue, 2)
                cv2.putText(img, "%d-%d" % (x_cent, y_cent), (x+10,y-10),cv2.FONT_HERSHEY_SIMPLEX, 1, color_orange_blue, 2)
                SEND_MESANGE_TO_COM_PORT(Blue_ID, x_cent, y_cent)
        print("Blue = ", Blue, len(Blue))

    if len(contours_green) != 0:
        n_green = 0
        
        for c in contours_green:
            x, y, w, h = cv2.boundingRect(c)
            if w > 40 and h > 40:
                cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)
                x_cent = x + int(w/2)
                y_cent = y + int(h/2)
                Green.append([x_cent, y_cent])
                cv2.circle(img, (x_cent, y_cent), 5, color_purple_green, 2)
                cv2.putText(img, "%d-%d" % (x_cent, y_cent), (x+10,y-10),cv2.FONT_HERSHEY_SIMPLEX, 1, color_purple_green, 2)
                SEND_MESANGE_TO_COM_PORT(Green_ID, x_cent, y_cent)
        print("Green = ", Green, len(Green))

    if len(contours_red) != 0:
        
        for c in contours_red:
            x, y, w, h = cv2.boundingRect(c)
            if w > 40 and h > 40:
                cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)
                x_cent = x + int(w/2)
                y_cent = y + int(h/2)
                Red.append([x_cent, y_cent])
                cv2.circle(img, (x_cent, y_cent), 5, color_yellow_red, 2)
                cv2.putText(img, "%d-%d" % (x_cent, y_cent), (x+10,y-10),cv2.FONT_HERSHEY_SIMPLEX, 1, color_yellow_red, 2)
                SEND_MESANGE_TO_COM_PORT(Red_ID, x_cent, y_cent)
        print("Red = ", Red, len(Red))


    ################################################################
    # Поиск линий
    ################################################################

    gray_image = cv2.cvtColor(img_line, cv2.COLOR_BGR2GRAY)
    th = 80 # 80 порог преобразования в чернобелое изображение
    ret, threshold_image = cv2.threshold(gray_image, th, 255, 0)
    threshold_image = cv2.bitwise_not(threshold_image)
    #cv2.imshow('threshold_image', cv2.line(threshold_image, (int(0), int(460)), (int(639), int(460)), (255,255,255), 1)) # вывод черно-белого изобажения

    h = img.shape[0]
    w = img.shape[1]

    F_left = threshold_image[0 : h, 0 : h_s]
    F_top = threshold_image[0 : h_s, 0 : w]
    F_right = threshold_image[0 : h, (w - h_s) : w]
    F_bottom = threshold_image[(h - (h_s+30)) : (h-30), 0 : w]
    
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

    # print('dArea_F_top', dArea_F_top)
    # print('dArea_F_bottom', dArea_F_bottom)
    # print('dArea_F_right', dArea_F_right)
    # print('dArea_F_left', dArea_F_left)


    if dArea_F_top > calibration_line_tikcness:# 
           x_top = int(dM10_F_top / dArea_F_top)
           y_top = int(dM01_F_top / dArea_F_top)
           cv2.circle(img, (x_top, y_top), 10, (128,128,128), -1)

    if dArea_F_bottom > calibration_line_tikcness:
           x_bottom = int(dM10_F_bottom / dArea_F_bottom) 
           y_bottom = int(dM01_F_bottom / dArea_F_bottom) + (h - h_s)
           cv2.circle(img, (x_bottom, y_bottom), 10, (128,128,128), -1)

    if dArea_F_right > calibration_line_tikcness:
           x_right = int(dM10_F_right / dArea_F_right) + (w - h_s)
           y_right = int(dM01_F_right / dArea_F_right)
           cv2.circle(img, (x_right, y_right), 10, (128,128,128), -1)

    if dArea_F_left > calibration_line_tikcness:
           x_left = int(dM10_F_left / dArea_F_left)
           y_left = int(dM01_F_left / dArea_F_left)
           cv2.circle(img, (x_left, y_left), 10, (128,128,128), -1)

    #print("x_top, y_top", x_top, y_top)
    #print("x_bottom, y_bottom", x_bottom, y_bottom)
    #print("x_left, y_left", x_left, y_left)
    #print("x_right, y_right", x_right, y_right)
    if (x_left != 0 and y_left != 0 and
        x_right != 0 and y_right != 0 and
        x_top != 0 and y_top !=0 and
        x_bottom != 0 and y_bottom != 0):
        rotation = 0
        for i in range(len(Green)):
            rotation += scenario_crossroads(np.array([x_top, y_top]), np.array([x_bottom, y_bottom]),
                                  np.array([x_left, y_left]), np.array([x_right, y_right]),
                                  np.array([Green[i][0], Green[i][1]]))
        print("X Поворот -->", rotation)

    ################################################
    # T-образзный перекресток ВНИЗ
    if (x_left != 0 or y_left != 0) and (x_right != 0 or y_right != 0) and (x_bottom != 0 or y_bottom != 0) and (x_top == 0 or y_top == 0):
        x_cross, y_cross = calc_center_crossroads(x_left, y_left, x_right, y_right, x_bottom, y_bottom)
        cv2.circle(img, (int(x_cross), int(y_cross)), 15, (128,128,128), -1)
        color = (0,0,255)#red
        img = line_show(img, threshold_image, x_cross, y_cross, x_bottom, y_bottom, color)
        rotation = 0
        for i in range(len(Green)):
            rotation += scenario_crossroads(np.array([x_cross, y_cross], dtype=int), np.array([x_bottom, y_bottom]),
                                np.array([x_left, y_left]), np.array([x_right, y_right]),
                                np.array([Green[i][0], Green[i][1]]))
        print("*Поворот -->", rotation)
    # T-образзный перекресток ВЛЕВО
    if (x_left != 0 or y_left != 0) and (x_top != 0 or y_top != 0) and (x_bottom != 0 or y_bottom != 0) and (x_right == 0 or y_right == 0):
        rotation = 0
        x_cross, y_cross = calc_center_crossroads(x_bottom, y_bottom, x_top, y_top, x_left, y_left)
        cv2.circle(img, (int(x_cross), int(y_cross)), 15, (128,128,128), -1)
        
        color = (0,0,255)#red
        img = line_show(img, threshold_image, x_cross, y_cross, x_left, y_left, color)
        for i in range(len(Green)):
            rotation += scenario_crossroads(np.array([x_top, y_top]), np.array([x_bottom, y_bottom]),
                                                 np.array([x_left, y_left]), np.array([x_cross, y_cross], dtype=int),
                                                 np.array([Green[i][0], Green[i][1]]))
        print("**Поворот -->", rotation)
    # T-образзный перекресток ВПРАВО
    if (x_right != 0 or y_right != 0) and (x_top != 0 or y_top != 0) and (x_bottom != 0 or y_bottom != 0) and (x_left == 0 or y_left == 0):
        x_cross, y_cross = calc_center_crossroads(x_bottom, y_bottom, x_top, y_top, x_right, y_right)
        cv2.circle(img, (int(x_cross), int(y_cross)), 15, (128,128,128), -1)
        color = (0,0,255)#red
        img = line_show(img, threshold_image, x_cross, y_cross, x_right, y_right, color)
        rotation = 0
        for i in range(len(Green)):
            rotation += scenario_crossroads(np.array([x_top, y_top]), np.array([x_bottom, y_bottom]),
                                                    np.array([x_cross, y_cross], dtype=int), np.array([x_right, y_right]),
                                                    np.array([Green[i][0], Green[i][1]]))
        print("***Поворот -->", rotation)
    print('calc_cord_object(x_cross, y_cross)', calc_cord_object(x_cross, y_cross))
    if dArea_F_top > calibration_line_tikcness and dArea_F_bottom > calibration_line_tikcness:
           color = (255,0,0)#red
           img = line_show(img, threshold_image, x_top, y_top, x_bottom, y_bottom, color)

    if dArea_F_left > calibration_line_tikcness and dArea_F_right > calibration_line_tikcness:
           color = (250,255,0)#yellow
           img = line_show(img, threshold_image, x_left, y_left, x_right, y_right, color)

    if dArea_F_left > calibration_line_tikcness and dArea_F_bottom > calibration_line_tikcness:
           color = (0,255,0)#green
           img = line_show(img, threshold_image, x_left, y_left, x_bottom, y_bottom, color)

    if dArea_F_right > calibration_line_tikcness and dArea_F_bottom > calibration_line_tikcness:
           color = (0,0,255)#blue
           img = line_show(img, threshold_image, x_right, y_right, x_bottom, y_bottom, color)

    if dArea_F_left > calibration_line_tikcness and dArea_F_top > calibration_line_tikcness:
           color = (242,0,255)#purple
           img = line_show(img, threshold_image, x_left, y_left, x_top, y_top, color)

    if dArea_F_right > calibration_line_tikcness and dArea_F_top > calibration_line_tikcness:
           color = (0,255,255)#light blue
           img = line_show(img, threshold_image, x_right, y_right, x_top, y_top, color)



    cv2.circle(img, (320, 240), 3, (0, 255, 0), -1)

    cv2.imshow('camera', img)

    ch = cv2.waitKey(1)

    count_frame = count_frame + 1
    #zapis poslednih 100 kadrov
    '''
    img_name = '/home/nano/Documents/img/'+str(count_frame)+'.png'

    cv2.imwrite(img_name, img)
    
    if count_frame == 100:
       count_frame = 0
    '''
    if ch == 27:
        end_time = time.time()
        print("Работа успешно завершена, средний FPS = ", count_frame/(end_time - start_time))
        break
  
cap.release()
cv2.destroyAllWindows()


