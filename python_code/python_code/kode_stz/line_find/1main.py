import cv2
import numpy as np
from scipy.signal import find_peaks
import matplotlib.pylab as plt
import treck
import line_radar
from line_radar import  polar_image_fun as RL
import math
import time
import simple_pid as pid

len_FOV = 360

my_mask = np.zeros((len_FOV, len_FOV))

my_mask = line_radar.polar_image_fun_simple(my_mask)
#my_mask = cv2.cvtColor(my_mask,cv2.COLOR_BGR2GRAY)
ret, mask = cv2.threshold(my_mask, 0, 255, cv2.THRESH_BINARY)
my_mask = cv2.bitwise_not(mask)
#cv2.imshow('my_mask', my_mask)





my_calibration = np.arange(0, 255, 255/len_FOV)
#print('my_calibration', my_calibration)

##########
scene_size = (10000, 10000)
num_point = 2
Points, scene = treck.treck_point_gen(scene_size, num_point)
print('scene.shape', scene.shape)
scene_track = np.copy(scene)

step = 0
start_x = 0
start_y = 0
lenght_move = 0
angle_move_derees = 0
angle_line = 0
tic = time.time()


pid.sample_time = 0.01  # Update every 0.01 seconds
pid.setpoint = 10

#ref_angle = angle line
I_last = 0
err_last = 0
def my_PID(ref_angle, angle_In, FOV_angle):
    dt = 1#1
    kp = 2
    ki = 0.5
    kd = 0#0
    P = ref_angle - FOV_angle
    P_out = P*kp

    I = I_last + (ref_angle - FOV_angle)*dt
    I_out = I*ki
    PI_out = I_out + P_out

    err = ref_angle - FOV_angle
    D = (err-err_last)/dt
    D_out = D*kd
    PID_out = I_out + P_out +D*kd
    return PI_out

last_ref_angle = 0
def ref_angle(FOV_angle, angle_In):
    global last_ref_angle
    if FOV_angle > angle_In:
        print('111')
        ref_angle = FOV_angle + angle_In
        #if abs(last_ref_angle - ref_angle) > 3
        #    last_ref_angle = ref_angle
    if FOV_angle < angle_In:
        ref_angle = FOV_angle - angle_In
        #if abs(last_ref_angle - ref_angle) > 3
       #     last_ref_angle = ref_angle

        print('222')
    else:
        ref_angle = 0
        print('333')
    ref_angle = 360 - 45
    return ref_angle

while True:
    if step == 0:
        FOV, start_x, start_y, angle_line = treck.get_FOV(scene, scene_track, 0, 0, 0, len_FOV, step)
        print(angle_line)
        cv2.imshow('FOV', FOV)
        scene_track_vis = cv2.resize(scene_track, (500, 500))
        cv2.imshow('scene_track_vis', scene_track_vis)
        lenght_move, angle_move_derees_in = RL(FOV, my_mask, my_calibration, len_FOV)
        FOV_angle = 45
        angle_move = angle_move_derees_in-FOV_angle
        print('New_angle_move_derees', angle_move_derees_in)
        cv2.waitKey(1000)
        pass
    else:
        print('FOV_angle_PID', FOV_angle)
        print('angle_move_PID', angle_move)

        ref_angle_val = ref_angle(FOV_angle, angle_move)
        start_time_PID = time.now()
        angle_move_derees_PID = my_PID(ref_angle_val, angle_move, FOV_angle)
        stop_time_PID = time.now()
        print('angle_move_derees_PID', angle_move_derees_PID)

        #print('angle_move_derees out line detect', angle_move_derees_in)
        angle_move_derees_out = FOV_angle + angle_move_derees_in
        #print('angle_move_derees real', angle_move_derees)
        error_1 = angle_move_derees_out + FOV_angle
        error_2 = 90 - angle_move_derees_in
        gain_1 = 0.5
        gain_2 = 0.5
        if error_1 > 0:
            angle_move_derees_out = angle_move_derees_out + gain_1*error_1
        if error_1 < 0:
            angle_move_derees_out = angle_move_derees_out - gain_1*error_1
        #print('angle_move_derees_out', angle_move_derees_out)
        if error_2 > 0:
            angle_move_derees_out = angle_move_derees_out - gain_2*error_2
        if error_2 < 0:
            angle_move_derees_out = angle_move_derees_out + gain_2*error_2
        #print('error_1', error_1)
        #print('error_2', error_2)

        #print('angle_move_derees_out', angle_move_derees_out)
        '''
        #idet pryamo
        if angle_move_derees  < 90:
            angle_move_derees = -int(angle_move_derees/2)
            #if FOV_angle < 10:
                #FOV_angle = 0
        if angle_move_derees > 90:
            angle_move_derees = int(angle_move_derees/2)
            #if FOV_angle < 10:
                ##FOV_angle = 0
        '''

        if error_1 > 0:
            x_move = int(start_x - 0.01*lenght_move*math.cos(math.radians(angle_move_derees_PID)))
            y_move = int(start_y + 0.01 * lenght_move * math.sin(math.radians(angle_move_derees_PID)))
        if error_1 < 0:
            x_move = int(start_x + 0.01*lenght_move*math.cos(math.radians(angle_move_derees_PID)))
            y_move = int(start_y - 0.01 * lenght_move * math.sin(math.radians(angle_move_derees_PID)))



        #print('delta x', x_move-start_x)
        #print('delta y', y_move-start_y)
        try:

            FOV_angle = math.atan((y_move - start_y) / (x_move - start_x))
            FOV_angle = math.degrees(FOV_angle)
            if FOV_angle<1:
                FOV_angle = 0
            print('try')
        except:
            print('except')
            FOV_angle = 0

        print('FOV_angle', FOV_angle)
        FOV_angle


        #print('y_move', y_move)
        print('x_move', x_move, 'y_move', y_move)
        FOV = treck.get_FOV(scene, scene_track, x_move, y_move, FOV_angle, len_FOV, step)
        cv2.imshow('FOV', FOV)
        scene_track_vis = cv2.resize(scene_track, (500, 500))
        cv2.imshow('scene_track_vis', scene_track_vis)
        name_step_vis = 'scene_track_vis' + str(step)
        #cv2.imwrite(name_step_vis, scene_track_vis)
        angle_move_derees_last = angle_move_derees_out
        lenght_move, angle_move_derees_in = RL(FOV, my_mask, my_calibration, len_FOV)
        #print('angle_move_derees', angle_move_derees)
        ####gesteresis#######
        #angle_move_derees_last = angle_move_derees
        '''
        if abs(angle_move_derees - angle_move_derees_last) <= 25 and abs(angle_move_derees - angle_move_derees_last) !=0:
            angle_move_derees_last = angle_move_derees  # angle_move_derees
            print('New_angle_move_derees', angle_move_derees)
        if abs(angle_move_derees - angle_move_derees_last) >= 25:
            angle_move_derees_last = angle_move_derees
            print('New_angle_move_derees', angle_move_derees)
        '''

        coeff_gain = 1

        # 0.5, 0.6, 3, 9

        if start_y <= 0:
            toc = time.time()
            print('!!!!!!!!!!!!!!!!!!Success!!!!!!!!!!!!!!!!!')
            print('time_way -->', toc-tic)
            cv2.waitKey(0)
            break
        cv2.waitKey(1)
        start_x = x_move
        start_y = y_move



    step +=1



