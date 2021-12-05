import cv2
import numpy as np
from scipy.signal import find_peaks
import matplotlib.pylab as plt
import math
def polar_image_fun_simple(img):
#--- the following holds the square root of the sum of squares of the image dimensions ---
#--- this is done so that the entire width/height of the original image is used to express the complete circular range of the resulting polar image ---
    value = np.sqrt(((img.shape[0])**2.0)+((img.shape[1]/2)**2.0))

    polar_image = cv2.linearPolar(img,(img.shape[0]/2, img.shape[1]), value, cv2.WARP_FILL_OUTLIERS)
    polar_image = cv2.rotate(polar_image, cv2.cv2.ROTATE_90_CLOCKWISE)
    polar_image = cv2.rotate(polar_image, cv2.cv2.ROTATE_90_CLOCKWISE)
    polar_image = cv2.rotate(polar_image, cv2.cv2.ROTATE_90_CLOCKWISE)
    polar_image = cv2.flip(polar_image, 1)
    return polar_image

def polar_image_fun(img, my_mask, my_calibration, len_FOV):
#--- the following holds the square root of the sum of squares of the image dimensions ---
#--- this is done so that the entire width/height of the original image is used to express the complete circular range of the resulting polar image ---
    value = np.sqrt(((img.shape[0])**2.0)+((img.shape[1]/2)**2.0))

    polar_image = cv2.linearPolar(img,(img.shape[0]/2, img.shape[1]), value, cv2.WARP_FILL_OUTLIERS)
    polar_image = cv2.rotate(polar_image, cv2.cv2.ROTATE_90_CLOCKWISE)
    polar_image = cv2.rotate(polar_image, cv2.cv2.ROTATE_90_CLOCKWISE)
    polar_image = cv2.rotate(polar_image, cv2.cv2.ROTATE_90_CLOCKWISE)
    polar_image = cv2.flip(polar_image, 1)


    #cv2.imshow('polar_image 1', polar_image)


    '''
    #polar_image = cv2.cvtColor(polar_image,cv2.COLOR_BGR2GRAY)
    ret, mask = cv2.threshold(polar_image, 0, 255, cv2.THRESH_BINARY)
    polar_image_1 = cv2.bitwise_not(mask)
    polar_image = polar_image + my_mask

    cv2.imshow('polar_image_1', polar_image)
    cv2.waitKey(0)
    '''


    #polar_image = cv2.bitwise_not(polar_image)
    #print('np.amax(polar_image)', np.amax(polar_image))
    #print('polar_image', polar_image)
    #cv2.imshow('polar_image_calibr', polar_image)
    polar_image = polar_image / 128
    #print(np.amax(polar_image))
    polar_image = polar_image.T * np.flip(my_calibration)
    polar_image = polar_image.T
    polar_image = polar_image.T
    polar_image = polar_image.T
    polar_image = polar_image.astype(np.uint8)
    #print(np.amax(polar_image))

    cv2.imshow('polar_image_calibr', polar_image)
    max_all = np.array([])

    for i in range(polar_image.shape[0]):
       m = np.amax(polar_image[:, i])
       max_all = np.append(max_all, m)

    max_all = max_all.astype(np.uint8)


    #print(r_all)
    #print(np.amax(r_all))

    #print(r_all.shape)
    ones_my = np.zeros((max_all.shape[0], max_all.shape[0]))

    angle_ax = np.arange(len_FOV-1)

    radius = 1
    color = (255, 255, 255)
    thickness = 1





    for i in range(angle_ax.shape[0]):
      center_coordinates = (i, max_all[i])
      ones_my = cv2.circle(ones_my, center_coordinates, radius, color, thickness)
    hist = np.copy(ones_my)
    cv2.imshow('hist', hist)


    ones_my = np.zeros((max_all.shape[0], max_all.shape[0]))
    #max_all_diff = np.diff(max_all)
    peaks, _ = find_peaks(max_all, distance=360)
    print('peaks', peaks)


    max_Line = np.amax(max_all)
    #print('max_Line', max_Line)

    if peaks.shape == (0,):
        max_Line = np.amax(max_all)
        #print('max_Line', max_Line)
    else:
     for i in range(peaks.shape[0]):
          center_coordinates = (peaks[i], max_all[peaks[i]])
          #print('r_all_d[i]', max_all[i])
          ones_my = cv2.circle(ones_my, center_coordinates, radius, color, thickness)
    peaks_img = np.copy(ones_my)

    cv2.imshow('peaks_img', peaks_img)

    '''
    r_all = np.array([])
    for i in range(polar_image.shape[0]):
      r = np.sum(polar_image[:, i])
      r_all = np.append(r_all, r)
    r_all = r_all*(1000/np.amax(r_all))
    r_all = r_all.astype(np.uint8)
    #print(r_all)
    ones_my = np.zeros((r_all.shape[0], np.amax(r_all)))
    r_all_d = np.diff(r_all)
    for i in range(angle.shape[0]-1):
     center_coordinates = (i, r_all_d[i])
     ones_my = cv2.circle(ones_my, center_coordinates, radius, color, thickness)
    sum_img_normalised = cv2.resize(ones_my, (500, 500))
    '''
    polar_image = polar_image.astype(np.uint8)

    if peaks.shape != (0,):
        lenght_move = np.amax(max_all)
        angle_move_derees = peaks#!!!int(np.argmax(max_all) * (360/polar_image.shape[0]))
    else:
        lenght_move = np.amax(max_all)#max_all[np.argmax(max_all)]
    angle_move_derees = np.argmax(max_all)# * (360/polar_image.shape[0])

    #######
    y_line_point_2 = int(np.amax(max_all) - np.amax(max_all)/2)
    x_line_point_2 = polar_image[y_line_point_2, :]
    list, = np.where(x_line_point_2 != 0)
    x_line_point_2 = np.array(list, dtype='int')
    angle_point_2 = np.mean(x_line_point_2)
    print('angle_point_2', angle_point_2)
    try:
        angle_point_2 = int(x_line_point_2[0]+1)
        print('angle_point_2', angle_point_2)
    except:
        print('except because second point not found')
        angle_point_2 = angle_move_derees
        print('angle_point_2 = angle_move_derees -->', angle_point_2)
    print('y_line_point_2', y_line_point_2)

    cv2.waitKey(1)


    lenght_move = -lenght_move + y_line_point_2
    print('lenght_move, angle_move_derees', lenght_move, angle_move_derees)

    return lenght_move, angle_move_derees
