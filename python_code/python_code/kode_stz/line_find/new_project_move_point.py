import time
import numpy as np
import cv2
import math
import random


def give_scene(scene_size):#
    scene = np.zeros((scene_size))
    scene = scene.astype(np.uint8)
    return scene





def Object_Model(lenght_move, angle_move_derees, scene):
    color = (255, 255, 255)
    thickness = 2
    global t
    global start_x
    global start_y

    if angle_move_derees - 360 > 0:
        N_360_angle_move_derees = int(angle_move_derees / 360)
        if angle_move_derees > 0:
            angle_move_derees = angle_move_derees - 360*N_360_angle_move_derees
        if angle_move_derees < 0:
            angle_move_derees = angle_move_derees - 360 * N_360_angle_move_derees


    print('angle_move_derees_Obj', angle_move_derees)
    print('start_x, start_y', start_x, start_y)
    scene = cv2.circle(scene, (int(start_x), int(start_y)), 5, color, thickness, -1)
    x_move_dt = lenght_move * math.cos(math.radians(angle_move_derees)) * dt_Obj

    print(math.cos(math.radians(angle_move_derees)))
    x_move = start_x + x_move_dt
    y_move_dt = lenght_move * math.sin(math.radians(angle_move_derees)) * dt_Obj
    print('x_move_dt', x_move_dt)
    print('y_move_dt', y_move_dt)

    y_move = start_y - y_move_dt
    print('x_move', x_move)
    print('y_move', y_move)


    t = t + dt_Obj
    #S = math.sqrt((x_move - start_x_1) ** 2 + (y_move - start_y_1) ** 2)
    scene = cv2.circle(scene, (int(x_move), int(y_move)), 5, color, thickness, -1)
    scene_show = cv2.resize(scene, (500, 500))
    cv2.imshow('World', scene_show)
    cv2.waitKey(1)
    print(t)
    if y_move <= 0:
        print('final_time -->', t)
        cv2.waitKey(0)
    if x_move - start_x == 0:
        x_move = x_move
    try:
        current_angle_0 = math.degrees(math.atan((y_move - start_y) / (x_move - start_x)))
        if angle_move_derees > 0 and  angle_move_derees < 90:
            current_angle_0 = - round(current_angle_0, 3) #- 1
        if angle_move_derees > 90 and angle_move_derees < 180:
            current_angle_0 = 180 - round(current_angle_0, 3) #- 1
        if angle_move_derees == 180 and int(current_angle_0) == 0:
            current_angle_0 = 180
        if angle_move_derees > 180 and angle_move_derees < 270:
            current_angle_0 = 180 - round(current_angle_0) #+ 1
        if angle_move_derees >270:
            current_angle_0 = 360 - round(current_angle_0, 3)

    except:
        if angle_move_derees == 0:
            current_angle_0 = 0
        if angle_move_derees == 90:
            current_angle_0 = 90
        if angle_move_derees == 270:
            current_angle_0 = 270

    print('current_angle_0', current_angle_0)
    start_x = x_move
    start_y = y_move
    #time.sleep(0.01)
    return scene, y_move, current_angle_0


def PID_1(angle_ref, current_angle_0):
    global fi_last
    global err_last
    global angle_move_derees_I_last
    kp = 1.000
    ki = 0
    kd = 0
    print('angle_ref', angle_ref)
    err = angle_ref - current_angle_0
    print('err', err)
    angle_move_derees_P = err

    angle_move_derees_I = angle_move_derees_I_last + err  # 0.879

    angle_move_derees_D = err - err_last # 0.8

    # 0.6, 0.9, 0.8 - time = 880

    angle_move_derees = angle_move_derees_I*ki + angle_move_derees_P*kp + angle_move_derees_D*kd



    if angle_move_derees == 360:
        angle_move_derees = 0
    if angle_move_derees == -360:
        angle_move_derees = 0

    fi_last = angle_move_derees
    angle_move_derees_I_last = angle_move_derees_I
    err_last = err

    print('angle_move_derees_PID', angle_move_derees)
    angle_move_derees = current_angle_0 + angle_move_derees
    return angle_move_derees




scene_size = (5000, 5000)
dt_Obj = 1
lenght_move = 2
angle_move_derees = 0
angle_ref = 315
start_x = 2500
start_y = 2500
t = 0
fi_last = 0
angle_move_derees_I_last = 0
err_last = 0

def main(lenght_move, angle_move_derees, scene_size):
    global fi_last
    global err_last
    global angle_move_derees_I_last
    scene = give_scene(scene_size)
    print('angle_move_derees_0', angle_move_derees)
    while True:
        error_fi_rand = random.uniform(-10, 10)
        print('error_fi_rand', error_fi_rand)
        angle_move_derees_sum_err = angle_move_derees + error_fi_rand
        error_fi = angle_ref - (angle_move_derees- angle_move_derees_sum_err)#будущяя ошибка
        scene, y_move, current_angle_0 = Object_Model(lenght_move, angle_move_derees_sum_err, scene)
        angle_move_derees = PID_1(angle_ref, current_angle_0)

        if y_move <= 0:
            break











main(lenght_move, angle_move_derees, scene_size)
