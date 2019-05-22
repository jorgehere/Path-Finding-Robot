import vrep
import numpy as np
import math
import time
import pandas as pd
from logic import *
from agents import *
from utils import *
from node import *


global state_, block_, x_, y_, dir_, x, y, e, w, n, s, theta 
state_ = 1      # 1 - move forward on dir_
                # 2 - check block_ wall and update KB
                # 3 - Search path
x_ = 0
y_ = 0
x = np.array([])
y = np.array([])
e = np.array([])
w = np.array([])
n = np.array([])
s = np.array([])

PI = math.pi
sensor_h = []
sensor_val = np.array([])
sensor_loc = np.array([-PI/2, -50/180.0*PI, -30/180.0*PI, -10/180.0*PI, 10/180.0*PI, 30/180.0*PI, 50/180.0*PI,
                       PI/2, PI/2, 130/180.0*PI, 150/180.0*PI, 170/180.0*PI, -170/180.0*PI, -150/180.0*PI, -130/180.0*PI, -PI/2])


def find_wall(sensor_val, sensor_loc, clientID, left_motor_handle, right_motor_handle):
    global theta
    # check w[block_]
    set_acc = 0.13
    reset_acc = 0.55
    if (sensor_val[0] < set_acc and sensor_val[15]< set_acc):
        theta = dir_ + 90 
        set_wall(theta, 1)
    elif (sensor_val[0] > reset_acc or sensor_val[15]> reset_acc):
        theta = dir_ + 90 
        set_wall(theta, 0)
    if (sensor_val[7] < set_acc and sensor_val[8]< set_acc):
        theta = dir_ - 90 
        set_wall(theta, 1)
    elif (sensor_val[7] > reset_acc or sensor_val[8]< reset_acc):
        theta = dir_ - 90 
        set_wall(theta, 0)
    if (sensor_val[11] < set_acc and sensor_val[12]< set_acc):
        theta = dir_ + 180 
        set_wall(theta, 1)
    elif (sensor_val[11] > reset_acc or sensor_val[12]> reset_acc):
        theta = dir_ + 180 
        set_wall(theta, 0)
    if (sensor_val[3] < set_acc and sensor_val[4]< set_acc):
        theta = dir_
        set_wall(theta, 1)
    elif (sensor_val[3] > reset_acc or sensor_val[4]> reset_acc):
        theta = dir_
        set_wall(theta, 0)
    return

def set_wall(theta, value):
    if (theta > 180):
        theta = theta - 360
    if (theta < -180):
        theta = theta + 360
    if (theta > -20 and theta < 20):
        n[block_] = value
    if (theta > -110 and theta < -70):
        e[block_] = value
    if (theta < -160 or theta > 160):
        s[block_] = value
    if (theta < 110 and theta > 70):
        w[block_] = value
    print ("Wall has been set for theta = "+str(theta)+"  for block = "+str(block_)+" with "+str(value))
    return

def find_gap(sensor_val, sensor_loc):
    max_i = np.argmax(sensor_val)
    if (sensor_val[max_i] < .4):
        return False, 0, 1
    return True, sensor_loc[max_i]*180/PI, sensor_val[max_i]


def right_turn(turn_time, clientID, left_motor_handle, right_motor_handle):
    erroCode = vrep.simxSetJointTargetVelocity(
        clientID, left_motor_handle, 0.2, vrep.simx_opmode_streaming)
    erroCode = vrep.simxSetJointTargetVelocity(
        clientID, right_motor_handle, -0.2, vrep.simx_opmode_streaming)
    print("Turn --------------------------> right "+str(turn_time))
    time.sleep(turn_time)
    erroCode = vrep.simxSetJointTargetVelocity(
        clientID, left_motor_handle, 0, vrep.simx_opmode_streaming)
    erroCode = vrep.simxSetJointTargetVelocity(
        clientID, right_motor_handle, 0, vrep.simx_opmode_streaming)
    return


def left_turn(turn_time, clientID, left_motor_handle, right_motor_handle):
    erroCode = vrep.simxSetJointTargetVelocity(
        clientID, left_motor_handle, -0.2, vrep.simx_opmode_streaming)
    erroCode = vrep.simxSetJointTargetVelocity(
        clientID, right_motor_handle, 0.2, vrep.simx_opmode_streaming)
    print("Turn --------------------------> left "+str(turn_time))
    time.sleep(turn_time)
    erroCode = vrep.simxSetJointTargetVelocity(
        clientID, left_motor_handle, 0, vrep.simx_opmode_streaming)
    erroCode = vrep.simxSetJointTargetVelocity(
        clientID, right_motor_handle, 0, vrep.simx_opmode_streaming)
    return

# align the robot according to two sensors
def align_robot(sens1, sens2, clientID, left_motor_handle, right_motor_handle):
    t = 0.05
    errorCode, sensor_handle1 = vrep.simxGetObjectHandle(
        clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(sens1), vrep.simx_opmode_oneshot_wait)
    errorCode, sensor_handle16 = vrep.simxGetObjectHandle(
        clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(sens2), vrep.simx_opmode_oneshot_wait)
    errorCode, detectionState, detectedPoint1, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
        clientID, sensor_handle1, vrep.simx_opmode_streaming)
    errorCode, detectionState, detectedPoint16, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
        clientID, sensor_handle16, vrep.simx_opmode_streaming)
    while (abs(detectedPoint1[0]-detectedPoint16[0]) > 0.01):
        t = 0.05
        if (detectedPoint1[0] > detectedPoint16[0]):
            left_turn(t, clientID, left_motor_handle, right_motor_handle)
            errorCode, detectionState, detectedPoint1, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                clientID, sensor_handle1, vrep.simx_opmode_streaming)
            errorCode, detectionState, detectedPoint16, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                clientID, sensor_handle16, vrep.simx_opmode_streaming)
        elif (detectedPoint1[0] < detectedPoint16[0]):
            right_turn(t, clientID, left_motor_handle, right_motor_handle)
            errorCode, detectionState, detectedPoint1, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                clientID, sensor_handle1, vrep.simx_opmode_streaming)
            errorCode, detectionState, detectedPoint16, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                clientID, sensor_handle16, vrep.simx_opmode_streaming)
        errorCode, detectionState, detectedPoint1, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            clientID, sensor_handle1, vrep.simx_opmode_streaming)
        errorCode, detectionState, detectedPoint16, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            clientID, sensor_handle16, vrep.simx_opmode_streaming)
        sensor_val = read_sensors(clientID, sensor_h)
    return
    
def get_dir(clientID, sensor8, target):
    position = vrep.simxGetObjectOrientation(clientID, sensor8, target, vrep.simx_opmode_buffer)
    print("position = "+str(position))
    if (position[1][2] > 0):
        dir = round(position[1][1]*180/PI,1)
    else:
        dir = round(-position[1][1]*180/PI+position[1][2]*180/PI,1)
    print("direction = "+str(dir))
    while (dir > 180):
        dir = dir - 360
    while (dir < -180):
        dir = dir + 360
    return dir

# read sensors
def read_sensors(clientID, sensor_h):
    sensor_val = np.array([])
    for i in range(1, 16+1):
        while True:
            errorCode, detectionState, detectedPoint1, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                clientID, sensor_h[i-1], vrep.simx_opmode_buffer)
            errorCode, detectionState, detectedPoint2, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                clientID, sensor_h[i-1], vrep.simx_opmode_buffer)
            if (abs(np.linalg.norm(detectedPoint2) - np.linalg.norm(detectedPoint1)) < 0.1):
                break
        sensor_val = np.append(sensor_val, round(np.linalg.norm(detectedPoint2), 2))
    print(" values = "+str(sensor_val))
    return sensor_val

# find block for current possition if existed, otherwise return -1
def find_block():
    for i in range(0,block_max):
        dist_xx = abs(x_ - x[i]) 
        dist_yy = abs(y_ - y[i])
        if (dist_xx < 1 and dist_yy < 1):
            return i
    return -1

# find new forward
def find_forward(sensor_val, sensor_loc, clientID, left_motor_handle, right_motor_handle, sensor8,target):
    global state_
    turn_side_time = 4.6
    turn_back_ration = 2.1
    if (dir_ > -20 and dir_ < 20):
        if (e[block_] == 0):
            right_turn(turn_side_time,clientID,left_motor_handle,right_motor_handle)
        elif (w[block_] == 0 ):
            left_turn(turn_side_time,clientID,left_motor_handle,right_motor_handle)
        if (w[block_] ==1 and e[block_] == 1 and n[block_] == 1):
            right_turn(turn_back_ration*turn_side_time,clientID,left_motor_handle,right_motor_handle)
            state_ = 3
            return True
    if (dir_ > -110 and dir_ < -70):
        if (n[block_] == 0 ):
            right_turn(turn_side_time,clientID,left_motor_handle,right_motor_handle)
        elif (s[block_] == 0 ):
            right_turn(turn_side_time,clientID,left_motor_handle,right_motor_handle)
        if (n[block_] ==1 and s[block_] == 1 and e[block_] == 1):
            right_turn(turn_back_ration*turn_side_time,clientID,left_motor_handle,right_motor_handle)
            state_ = 3
            return True
    if (dir_ > 70 and dir_ < 110):
        if (n[block_] == 0 ):
            right_turn(turn_side_time,clientID,left_motor_handle,right_motor_handle)
        elif (s[block_] == 0 ):
            left_turn(turn_side_time,clientID,left_motor_handle,right_motor_handle)
        if (n[block_] ==1 and s[block_] == 1 and e[block_] == 1):
            right_turn(turn_back_ration*turn_side_time,clientID,left_motor_handle,right_motor_handle)
            state_ = 3
            return True
    if (dir_ > 160 or dir_ < -160):
        if (w[block_] == 0 ):
            left_turn(turn_side_time,clientID,left_motor_handle,right_motor_handle)
        if (e[block_] == 0 ):
            left_turn(turn_side_time,clientID,left_motor_handle,right_motor_handle)
        if (w[block_] ==1 and e[block_] == 1 and s[block_] == 1):
            right_turn(turn_back_ration*turn_side_time,clientID,left_motor_handle,right_motor_handle)
            state_ = 3
            return True
    state_ = 2
    return True

# take step back when a wall is hit
def step_back(clientID, left_motor_handle, right_motor_handle):
    global x_, y_
    erroCode = vrep.simxSetJointTargetVelocity(
        clientID, left_motor_handle, -0.2, vrep.simx_opmode_streaming)
    erroCode = vrep.simxSetJointTargetVelocity(
        clientID, right_motor_handle, 0.2, vrep.simx_opmode_streaming)
    time.sleep(.1)
    erroCode = vrep.simxSetJointTargetVelocity(
        clientID, left_motor_handle, 0, vrep.simx_opmode_streaming)
    erroCode = vrep.simxSetJointTargetVelocity(
        clientID, right_motor_handle, 0, vrep.simx_opmode_streaming)
    r = (0.2 * .1)
    x_ = x_ - r*math.sin(dir_*PI/180)
    y_ = y_ - r*math.cos(dir_*PI/180)
    print(" (x, y) = " + str(x_) + " , "+ str(y_))
    return


# Close all opened connections, just in case
vrep.simxFinish(-1)
clientID = vrep.simxStart('127.0.0.1', 19999, True,
                          True, 5000, 5)  # Connect to V-REP
if clientID != -1:
    print("Connected to remote API server")
else:
    print("Connection not successful")

errorCode, left_motor_handle = vrep.simxGetObjectHandle(
    clientID, "Pioneer_p3dx_leftMotor", vrep.simx_opmode_blocking)

errorCode, right_motor_handle = vrep.simxGetObjectHandle(
    clientID, "Pioneer_p3dx_rightMotor", vrep.simx_opmode_blocking)

# Sensoring
errorCode, sensor8 = vrep.simxGetObjectHandle(
    clientID, "Pioneer_p3dx_ultrasonicSensor8", vrep.simx_opmode_blocking)

# True/False detecting object by sensor
returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
    clientID, sensor8, vrep.simx_opmode_streaming)

returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
    clientID, sensor8, vrep.simx_opmode_buffer)

# get handle for the Target and Orientation
errorCode, target = vrep.simxGetObjectHandle(
    clientID, "Target", vrep.simx_opmode_blocking)
position = vrep.simxGetObjectOrientation( clientID, sensor8, target, vrep.simx_opmode_streaming )

# prepare sensors handles
for i in range(1, 16+1):
    print('Pioneer_p3dx_ultrasonicSensor'+str(i))
    errorCode, sensor_handle = vrep.simxGetObjectHandle(
        clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i), vrep.simx_opmode_oneshot_wait)
    sensor_h.append(sensor_handle)
    errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
        clientID, sensor_handle, vrep.simx_opmode_streaming)
    sensor_val = np.append(sensor_val, np.linalg.norm(detectedPoint))

# PropKB
kb = PropKB()
print("1: kb.clauses = ", kb.clauses)
# graph for depth first search

# Initialize global parameters
t = time.time()
block_ = 0
block_max = block_
e = np.append(e, 0)
n = np.append(n, 0)
w = np.append(w, 0)
s = np.append(s, 0)
e_c = 0
w_c = 0
n_c = 0
s_c = 0
x = np.append(x, 0)
y = np.append(y, 0)   # define new block with center coordinates
x_ = 0
y_ = 0
info = ""

while (time.time()-t) < 800:
    dir_ = get_dir(clientID, sensor8, target)
    while ((dir_ >= 20 and dir_ <= 70) or (dir_ >= 110 and dir_ <= 160) 
        or (dir_ >= -160 and dir_ <= -110) or (dir_ >= -70 and dir_ <= -20)):
        left_turn(.9,clientID,left_motor_handle,right_motor_handle)
        dir_ = get_dir(clientID, sensor8, target)

    sensor_val = read_sensors(clientID, sensor_h)
    print("Robot direction = "+str(dir_))
    # Read the sensors
    find_wall(sensor_val, sensor_loc, clientID, left_motor_handle, right_motor_handle)
    print("E =" + str( e[block_]))
    print("N =" + str( n[block_]))
    print("W =" + str( w[block_]))
    print("S =" + str( s[block_]))
    e_c = e_c + e[block_]
    w_c = w_c + w[block_]
    n_c = n_c + n[block_]
    s_c = s_c + s[block_]
    
      # check for front wall
    if (sensor_val[3]< .1 and sensor_val[4]<.1):
        step_back(clientID, left_motor_handle, right_motor_handle)
        find_forward(sensor_val, sensor_loc, clientID, left_motor_handle, right_motor_handle,sensor_h[7], target)
        dir_ = get_dir(clientID, sensor8, target)
        sensor_val = read_sensors(clientID, sensor_h)
        print("Robot direction = "+str(dir_)+"  State_ =" + str(state_))
    if (state_ == 3):
        print("state == 3!!!")

    # Fine align Theta to fit direction
    if (state_ == 2 or state_ == 3):
        theta = get_dir(clientID, sensor8, target)
        if (theta > -20 and theta < 20):
            while(theta < 0):
                left_turn(.1,clientID,left_motor_handle,right_motor_handle )
                theta = get_dir(clientID,sensor8,target)
            while(theta > 0):
                right_turn(.1,clientID,left_motor_handle,right_motor_handle )
                theta = get_dir(clientID,sensor8,target)
        if (theta > -110 and theta < -70):
            while(theta < -90):
                left_turn(.1,clientID,left_motor_handle,right_motor_handle )
                theta = get_dir(clientID,sensor8,target)
            while(theta > -90):
                right_turn(.1,clientID,left_motor_handle,right_motor_handle )
                theta = get_dir(clientID,sensor8,target)
        if (theta > 160):
            while(theta < 178 ):
                left_turn(.1,clientID,left_motor_handle,right_motor_handle )
                theta = get_dir(clientID,sensor8,target)
        if (theta < -160 ):
            while(theta > -178):
                right_turn(.1,clientID,left_motor_handle,right_motor_handle )
                theta = get_dir(clientID,sensor8,target)        
        if (theta < 110 and theta > 70):
            while(theta < 90):
                left_turn(.1,clientID,left_motor_handle,right_motor_handle )
                theta = get_dir(clientID,sensor8,target)
            while(theta > 90):
                right_turn(.1,clientID,left_motor_handle,right_motor_handle )
                theta = get_dir(clientID,sensor8,target)
        print ("Adjusting theta = "+str(theta)+"  for block = "+str(block_))
        
    # main state of navigation
    if (state_ == 1 or state_ == 3):
        erroCode = vrep.simxSetJointTargetVelocity(
            clientID, left_motor_handle, 0.5, vrep.simx_opmode_streaming)
        erroCode = vrep.simxSetJointTargetVelocity(
            clientID, right_motor_handle, 0.5, vrep.simx_opmode_streaming)
        time.sleep(.3)
        erroCode = vrep.simxSetJointTargetVelocity(
            clientID, left_motor_handle, 0, vrep.simx_opmode_streaming)
        erroCode = vrep.simxSetJointTargetVelocity(
            clientID, right_motor_handle, 0, vrep.simx_opmode_streaming)
        r = (0.5 * .3)
        x_ = x_ + r*math.sin(dir_*PI/180)
        y_ = y_ + r*math.cos(dir_*PI/180)
        print(" (x, y) = " + str(x_) + " , "+ str(y_))
        dist_x = abs(x_ - x[block_]) 
        dist_y = abs(y_ - y[block_])
        step_length = 8/6
        if (state_ == 3):
            walls_c = 0
            if (e_c >= 0):
                walls_c = walls_c + 1
            if (w_c >= 0):
                walls_c = walls_c + 1
            if (n_c >= 0):
                walls_c = walls_c + 1
            if (s_c >= 0):
                walls_c = walls_c + 1
            old_info = info
            if (walls_c < 2):
                info = "S"+str(block_)
            if (walls_c == 2):
                info = "P"+str(block_)
            if (walls_c >= 3 and state_ == 3):
                info = "B"+str(block_)
            if (walls_c >= 3 and state_ == 1):
                info = "P"+str(block_)
            kb.tell(info)
            if (old_info > "" and old_info[0] == "P" and info[0] == "P"):
                kb.tell(info+" >> " + old_info)
            if (old_info > "" and old_info[0] == "P" and info[0] == "B"):
                kb.tell(info+" >> " + old_info)
                print ("kb.clauses = ", kb.clauses)    
                e = np.append(e, 0)
                n = np.append(n, 0)
                w = np.append(w, 0)
                s = np.append(s, 0)
                x = np.append(x, x_)
                y = np.append(y, y_)   # define new block with center coordinates
                e_c = 0
                w_c = 0
                n_c = 0
                s_c = 0

        if ((dist_x > step_length or dist_y > step_length) and find_block() > -1):
            block_ = find_block()
            walls_c = e[block_] + w[block_] + n[block_] + s[block_] 
            if (walls_c < 2):
                info = "S"+str(block_)
            if (walls_c == 2):
                info = "P"+str(block_)
            if (walls_c >= 3 and state_ == 3):
                info = "B"+str(block_)

        elif (dist_x > step_length or dist_y > step_length):
            block_ = block_max + 1
            block_max = block_
            
            walls_c = 0
            max_c = max(e_c, w_c, s_c, n_c)
            if (state_ == 3 and min(e_c, w_c, s_c, n_c)==1):
                walls_c = walls_c + 1       # increase count to get B 
            if (e_c >= max_c/2):
                walls_c = walls_c + 1
            if (w_c >= max_c/2):
                walls_c = walls_c + 1
            if (n_c >= max_c/2):
                walls_c = walls_c + 1
            if (s_c >= max_c/2):
                walls_c = walls_c + 1
            old_info = info
            if (walls_c < 2):
                info = "S"+str(block_)
            if (walls_c == 2):
                info = "P"+str(block_)
            if (walls_c >= 3 and state_ == 3):
                info = "B"+str(block_)
                state_ = 1
            if (walls_c >= 3 and state_ == 1):
                info = "P"+str(block_)

                state_ = 1
            kb.tell(info)
            if (old_info > "" and old_info[0] == "P" and info[0] == "P"):
                kb.tell(info+" >> " + old_info)
                kb.tell("B"+str(info[1:])+">> B"+str(old_info[1:]))
            if (old_info > "" and old_info[0] == "P" and info[0] == "B"):
                kb.tell(info+" >> " + old_info)
            print ("kb.clauses = ", kb.clauses)    
            e = np.append(e, 0)
            n = np.append(n, 0)
            w = np.append(w, 0)
            s = np.append(s, 0)
            x = np.append(x, x_)
            y = np.append(y, y_)   # define new block with center coordinates
            e_c = 0
            w_c = 0
            n_c = 0
            s_c = 0

    state_ = 1
    # center the robot by sensors values
    if (e[block_] == 1 and w[block_] == 1 and dir_ < 20 and dir_ > -20 and 
            sensor_val[0] < sensor_val[7]):
        right_turn(0.09, clientID, left_motor_handle, right_motor_handle)
        sensor_val = read_sensors(clientID, sensor_h)
    if (e[block_] == 1 and w[block_] == 1 and dir_ < 20 and dir_ > -20 and 
            sensor_val[0] > sensor_val[7]):
        left_turn(0.09, clientID, left_motor_handle, right_motor_handle)
        sensor_val = read_sensors(clientID, sensor_h)
    if (e[block_] == 1 and w[block_] == 1 and dir_ > 160 and dir_ < -160 and 
            sensor_val[0] < sensor_val[7]):
        right_turn(0.09, clientID, left_motor_handle, right_motor_handle)
        sensor_val = read_sensors(clientID, sensor_h)
    if (e[block_] == 1 and w[block_] == 1 and dir_ > 160 and dir_ < -160 and 
            sensor_val[0] > sensor_val[7]):
        left_turn(0.09, clientID, left_motor_handle, right_motor_handle)
        sensor_val = read_sensors(clientID, sensor_h)

    if (n[block_] == 1 and s[block_] == 1 and dir_ < -70 and dir_ > -110 and 
            sensor_val[0] < sensor_val[7]):
        right_turn(0.09, clientID, left_motor_handle, right_motor_handle)
        sensor_val = read_sensors(clientID, sensor_h)
    if (n[block_] == 1 and s[block_] == 1 and dir_ < -70 and dir_ > -110 and 
            sensor_val[0] > sensor_val[7]):
        left_turn(0.09, clientID, left_motor_handle, right_motor_handle)
        sensor_val = read_sensors(clientID, sensor_h)
    if (n[block_] == 1 and s[block_] == 1 and dir_ > 70 and dir_ < 110 and 
            sensor_val[0] < sensor_val[7]):
        right_turn(0.09, clientID, left_motor_handle, right_motor_handle)
        sensor_val = read_sensors(clientID, sensor_h)
    if (n[block_] == 1 and s[block_] == 1 and dir_ > 70 and dir_ < 110 and 
            sensor_val[0] > sensor_val[7]):
        left_turn(0.09, clientID, left_motor_handle, right_motor_handle)
        sensor_val = read_sensors(clientID, sensor_h)

    # quick aline when about to touch wall
    if (sensor_val[0] < .1):
        right_turn(0.07, clientID, left_motor_handle, right_motor_handle)
        sensor_val = read_sensors(clientID, sensor_h)
    if (sensor_val[7] < .1 ):
        left_turn(0.07, clientID, left_motor_handle, right_motor_handle)
        sensor_val = read_sensors(clientID, sensor_h)

    # quick aline when there is space
    if (sensor_val[0] > .4):
        right_turn(0.04, clientID, left_motor_handle, right_motor_handle)
        sensor_val = read_sensors(clientID, sensor_h)
    if (sensor_val[7] > .4 ):
        left_turn(0.04, clientID, left_motor_handle, right_motor_handle)
        sensor_val = read_sensors(clientID, sensor_h)

print("stop")