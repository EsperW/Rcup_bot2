import numpy as np
import cv2



cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

ret, image = cap.read()
image_size = image.shape
mask_arr_2D = np.zeros((image_size[0], image_size[1]))
mask_arr_2D[:-20, :] = 1
mask_arr_3D = np.zeros((image_size))
mask_arr_3D[:-20, :, :] = 1




while True:
    ret, image = cap.read()
    if ret == True:
       gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
       th = 80 # 80 порог преобразования в чернобелое изображение
       ret, threshold_image = cv2.threshold(gray_image, th, 255, 0)
       threshold_image = cv2.bitwise_not(threshold_image)
       threshold_image = np.ma.masked_array(threshold_image, mask = mask_arr_2D)
       cv2.imshow('threshold_image', threshold_image) # вывод черно-белого изобажения
       print('threshold_image', threshold_image)
       cv2.waitKey(100)
       ch = cv2.waitKey(50)
       if ch == 27:
        break


    else:
       pass


#cv2. destroyAllWindows()

