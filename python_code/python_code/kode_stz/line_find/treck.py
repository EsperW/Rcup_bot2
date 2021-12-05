import cv2
import numpy as np
from scipy.signal import find_peaks
import matplotlib.pylab as plt
import math

radius = 1
color = (255, 255, 255)
thickness = 10
points = 0

def treck_point_gen(scene_size, num_point):
    global points
    scene = np.zeros((scene_size))
    scene = scene.astype(np.uint8)
    #points = np.random.rand(num_point, num_point)
    #scale = scene_size[0]-254
    #points = points*scale
    #points = points.astype(np.int)
    # cv2.imshow('scene', cv2.resize(scene, (500, 500)))
    points = np.array([[1000, 5000], [9000, 9000]])
    points_1 = np.array([[1000, 9000], [1000, 5000]])
    x, y = (9000, 9000)
    print('x, y', x, y)

    '''
    x_2, y_2 = points[0, :]
    if y < y_2:
        mem = points[0, :]
        points[0, :] = points[1, :]
        points[1, :] = mem
        
    '''
    #scene = cv2.line(scene, points[1, :], points[0, :], color, thickness)
    scene = cv2.line(scene, (5000, 5000), (9000, 1000), color, thickness)
    scene = cv2.line(scene, (9000, 9000), (5000, 5000), (128, 128, 128), thickness)


    print('x, y', x, y)
    return points, scene

def treck_sin_gen(scene_size, num_point):
    x = np.linspace(-np.pi, np.pi, 201)
    plt.plot(x, np.sin(x))
    plt.xlabel('Angle [rad]')
    plt.ylabel('sin(x)')
    plt.axis('tight')
    #plt.show()
treck_sin_gen(1,1)



def rotate_image(image, angle):
    #cv2.imshow('FOV_1', image)
    image_center = np.array(image.shape[1::-1]) / 2
    #print('image_center', image_center)
    x_center = int(image_center[0])
    y_center = int(image_center[1])
    image_center = (x_center, y_center)
    #print('image_center', image_center)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    image = image.astype(np.float32)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    #cv2.imshow('FOV_rot', result)
    return result

def get_FOV(scene, scene_track, x, y, angle, len_FOV, step_move):
    global points
    if step_move == 0:
        x, y = points[1, :]
        print('x, y', x, y)
        x_2, y_2 = points[0, :]
        angle = math.atan((y_2-y)/(x_2-x))
        #print('angle', angle)
        if y<y_2:
            x, y = points[0, :]
            print('x, y', x, y)
            x_2, y_2 = points[1, :]
        color = (255, 255, 255)
        thickness = 3



        scene_track = cv2.circle(scene_track, (x, y), 30, color, thickness, -1)
        scene_track = cv2.circle(scene_track, (x, y), 10, (0,0,0), thickness, -1)
        FOV = scene[x - len_FOV : x + len_FOV, y - len_FOV : y + len_FOV]
        #cv2.imshow('FOV_1', FOV)
        #print('FOV.shape', FOV.shape)
        if angle >= 0:
            angle = math.degrees(angle)+90#270 for len_FOV  254
        else:
            angle = math.degrees(angle)+270
        #print('angle', angle)
        FOV = rotate_image(FOV, angle)
        #cv2.imshow('FOV_rot_1', FOV)
        FOV = FOV[int(len_FOV/2):len_FOV+ int(len_FOV/2) , int(len_FOV/2):len_FOV+ int(len_FOV/2) ]
        #print('FOV.shape', FOV.shape)
        return FOV, y, x, angle#x\y reversive..
    else:
        color = (255, 255, 255)
        thickness = 3


        scene_track = cv2.circle(scene_track, (x, y), 30, color, thickness, -1)
        scene_track = cv2.circle(scene_track, (x, y), 10, (0,0,0), thickness, -1)
        FOV = scene[y - len_FOV: y + len_FOV, x - len_FOV:x + len_FOV]
        #print('FOV.shape', FOV.shape)
        angle = math.radians(angle)
        #cv2.imshow('FOV_1', FOV)
        #print('FOV.shape', FOV.shape)
        if angle >= 0:
            angle = math.degrees(angle)+270#ok
        else:
            angle = math.degrees(angle)+90
        #print('angle', angle)
        FOV = rotate_image(FOV, angle)
        #cv2.imshow('FOV_rot_1', FOV)
        FOV = FOV[int(len_FOV/2):len_FOV+ int(len_FOV/2) , int(len_FOV/2):len_FOV+ int(len_FOV/2) ]
        #print('FOV.shape', FOV.shape)
    return FOV







cv2.waitKey(0)




