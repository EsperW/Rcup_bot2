import matplotlib.pylab as plt
import cv2
import numpy as np
import matplotlib.pylab as plt
#import treck
import line_radar
from line_radar import  polar_image_fun as RL
import math
import time


radius = 1
color = (255, 255, 255)
thickness = 10
points = 0


def treck_point_gen(scene_size, num_point):
    global points
    scene = np.zeros((scene_size))
    scene = scene.astype(np.uint8)
    # points = np.random.rand(num_point, num_point)
    # scale = scene_size[0]-254
    # points = points*scale
    # points = points.astype(np.int)
    # cv2.imshow('scene', cv2.resize(scene, (500, 500)))
    points = np.array([[1500, 1500], [2000, 1000]])#    points = np.array([[1000, 5000], [9000, 9000]])
    points_1 = np.array([[2000, 2000], [1500, 1500]])#    points_1 = np.array([[1000, 9000], [1000, 5000]])




    scene = cv2.line(scene, (1500, 1500), (2000, 1000), color, thickness)
    scene = cv2.line(scene, (2000, 2000), (1500, 1500), (128, 128, 128), thickness)
    x, y = (2000, 2000)#    x, y = (9000, 9000)

    print('x, y', x, y)
    return points, scene


def treck_sin_gen(scene_size, num_point):
    x = np.linspace(-np.pi, np.pi, 201)
    plt.plot(x, np.sin(x))
    plt.xlabel('Angle [rad]')
    plt.ylabel('sin(x)')
    plt.axis('tight')
    # plt.show()


treck_sin_gen(1, 1)


def rotate_image(image, angle):
    # cv2.imshow('FOV_1', image)
    image_center = np.array(image.shape[1::-1]) / 2
    # print('image_center', image_center)
    x_center = int(image_center[0])
    y_center = int(image_center[1])
    image_center = (x_center, y_center)
    # print('image_center', image_center)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    image = image.astype(np.float32)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    # cv2.imshow('FOV_rot', result)
    return result


def get_FOV(scene, scene_track, x, y, angle, len_FOV, step_move):
    global points
    if step_move == 0:
        x, y = (2000, 2000)  # x, y = (9000, 9000)
        print('x, y', x, y)
        x_2, y_2 = (1500, 1500)
        angle = math.atan((y_2 - y) / (x_2 - x))
        # print('angle', angle)
        if y < y_2:
            x, y = points[0, :]
            print('x, y', x, y)
            x_2, y_2 = points[1, :]
        color = (255, 255, 255)
        thickness = 3

        scene_track = cv2.circle(scene_track, (x, y), 30, color, thickness, -1)
        scene_track = cv2.circle(scene_track, (x, y), 10, (0, 0, 0), thickness, -1)
        FOV = scene[x - len_FOV: x + len_FOV, y - len_FOV: y + len_FOV]
        # cv2.imshow('FOV_1', FOV)
        # print('FOV.shape', FOV.shape)
        if angle >= 0:
            angle = math.degrees(angle) + 270  # 270 for len_FOV  254
        else:
            angle = math.degrees(angle) + 90
        # print('angle', angle)
        FOV = rotate_image(FOV, angle)
        # cv2.imshow('FOV_rot_1', FOV)
        FOV = FOV[int(len_FOV / 2):len_FOV + int(len_FOV / 2), int(len_FOV / 2):len_FOV + int(len_FOV / 2)]
        # print('FOV.shape', FOV.shape)
        return FOV, y, x, angle  # x\y reversive..
    else:
        color = (255, 255, 255)
        thickness = 3

        scene_track = cv2.circle(scene_track, (x, y), 30, color, thickness, -1)
        scene_track = cv2.circle(scene_track, (x, y), 10, (0, 0, 0), thickness, -1)
        FOV = scene[y - len_FOV: y + len_FOV, x - len_FOV:x + len_FOV]
        # print('FOV.shape', FOV.shape)
        angle = math.radians(angle)
        # cv2.imshow('FOV_1', FOV)
        # print('FOV.shape', FOV.shape)
        if angle >= 0:
            angle = math.degrees(angle) + 270  # ok
        else:
            angle = math.degrees(angle) + 90
        # print('angle', angle)
        FOV = rotate_image(FOV, angle)
        # cv2.imshow('FOV_rot_1', FOV)
        FOV = FOV[int(len_FOV / 2):len_FOV + int(len_FOV / 2), int(len_FOV / 2):len_FOV + int(len_FOV / 2)]
        # print('FOV.shape', FOV.shape)
    return FOV


#####################################################################################################

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
scene_size = (3000, 3000)
num_point = 2
Points, scene = treck_point_gen(scene_size, num_point)
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
    dt = 2
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
        ref_angle = FOV_angle - angle_In
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
    ref_angle = 90
    return ref_angle
dt = 0.05
Tau = 10
while True:
    if step == 0:
        FOV, start_x, start_y, angle_line = get_FOV(scene, scene_track, 0, 0, 0, len_FOV, step)
        print(angle_line)
        cv2.imshow('FOV', FOV)
        scene_track_vis = cv2.resize(scene_track, (500, 500))
        cv2.imshow('scene_track_vis', scene_track_vis)
        lenght_move, angle_move_derees_in = RL(FOV, my_mask, my_calibration, len_FOV)
        FOV_angle = 45
        angle_move = angle_move_derees_in-FOV_angle
        print('New_angle_move_derees', angle_move_derees_in)
        cv2.waitKey(2000)
        pass
    else:
        print('FOV_angle_PID', FOV_angle)
        print('angle_move_PID', angle_move)

        ref_angle_val = ref_angle(FOV_angle, angle_move)

        start_time_PID = time.time()
        angle_move_derees_PID = my_PID(ref_angle_val, angle_move, FOV_angle)
        print('angle_move_derees_PID', angle_move_derees_PID)
        stop_time_PID = time.time()


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

        #if error_1 > 0:
        start_x_1 = start_x
        start_y_1 = start_y

        S = 0
        t = 0
        for i in range(int(Tau)):
            x_move_dt =  lenght_move*math.cos(math.radians(angle_move_derees_PID))*dt
            #print('x_move_dt', x_move_dt)
            x_move = start_x +  (x_move_dt*t)
            #print('x_move', x_move)
            y_move_dt =  lenght_move * math.sin(math.radians(angle_move_derees_PID))*dt
            y_move = start_y - (y_move_dt*t)
            t = t + dt
            S = math.sqrt((x_move-start_x_1)**2 + (y_move-start_y_1)**2)
            #print('y_move', y_move)
            #print('x_move', x_move)
            #print('S', S)

        x_move = int(x_move)
        y_move = int(y_move)


        v = S/t
        print('t', t, 'S', S, 'v', v)

        print('delta x', x_move-start_x_1)
        print('delta y', y_move-start_y_1)
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
        FOV = get_FOV(scene, scene_track, x_move, y_move, FOV_angle, len_FOV, step)
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










#####################################################################################################





cv2.waitKey(0)




