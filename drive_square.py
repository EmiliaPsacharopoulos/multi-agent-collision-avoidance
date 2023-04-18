import lcm
import numpy as np
from lcmtypes import mbot_motor_command_t, timestamp_t
import time
from lcmtypes import odometry_t
from lcmtypes import reset_odometry_t
from threading import Thread, Lock, current_thread
import client
import sys


STARTX = 0              # Start means the current coordinate
STARTY = 0
ORIENTATION = None
NEW_ORIENTATION = None
ROW = 0
DISTANCE_FACTOR = 0.123 # Larger - Longer; Smaller - Shorter (Control the 12 inches)
DRIFT_FACTOR = 0.02     # Postive - Left ; Negative - Right
# Turn Factors
R_TURN_P = 2.35         # CW #SPECIFIC TO EACH MBOT
L_TURN_P = 2.0          # CCW #SPECIFIC TO EACH MBOT

curr_odometry = None


def get_init_orientation(x, y):
    orientation = None
    if x == 0:
        orientation = "EAST"
    if y == 4:
        orientation = "SOUTH"
    if x == 4:
        orientation = "WEST"
    if y == 0:
        orientation = "NORTH"
    return orientation


def handle_lcm(lcm_obj):
    try:
        while True:
            lcm_obj.handle()
    except KeyboardInterrupt:
        print("lcm exit!")
        sys.exit()


def current_utime(): 
    return int(time.time() * 1e6)


def odometry_message_handler(channel, data):
    global curr_odometry
    curr_odometry = odometry_t.decode(data)


def turn_angle(angle):
    prev_theta = curr_odometry.theta

    if (angle >= 0):
        target = min(curr_odometry.theta + angle, np.pi*2)
        excess = max(curr_odometry.theta + angle-2*np.pi,0)

        while (curr_odometry.theta < target and curr_odometry.theta >= prev_theta):
            # print("current: ", curr_odometry.theta," target: ", target)
            prev_delta = curr_odometry.theta
            # # Rotate
            rotate = mbot_motor_command_t()
            rotate.utime = current_utime()
            rotate.trans_v = 0.0
            rotate.angular_v = 1.0

            rotate_time = timestamp_t()
            rotate_time.utime = rotate.utime
            lc.publish("MBOT_TIMESYNC", rotate_time.encode())
            lc.publish("MBOT_MOTOR_COMMAND", rotate.encode())
            time.sleep(0.1)

        if (excess > 0):
            while (curr_odometry.theta < excess):
                # # Rotate
                rotate = mbot_motor_command_t()
                rotate.utime = current_utime()
                rotate.trans_v = 0.0
                rotate.angular_v = 1.0

                rotate_time = timestamp_t()
                rotate_time.utime = rotate.utime
                lc.publish("MBOT_TIMESYNC", rotate_time.encode())
                lc.publish("MBOT_MOTOR_COMMAND", rotate.encode())
                time.sleep(0.1)
    else:
        target = max(curr_odometry.theta + angle, 0)
        excess = min(curr_odometry.theta + angle, 0)

        while (curr_odometry.theta > target and curr_odometry.theta <= prev_theta):
            # print("current: ", curr_odometry.theta," target: ", target)
            prev_delta = curr_odometry.theta
            # # Rotate
            rotate = mbot_motor_command_t()
            rotate.utime = current_utime()
            rotate.trans_v = 0.0
            rotate.angular_v = -1.0

            rotate_time = timestamp_t()
            rotate_time.utime = rotate.utime
            lc.publish("MBOT_TIMESYNC", rotate_time.encode())
            lc.publish("MBOT_MOTOR_COMMAND", rotate.encode())
            time.sleep(0.1)

        if (excess < 0):
            while (curr_odometry.theta > np.pi*2 + excess):
                # # Rotate
                rotate = mbot_motor_command_t()
                rotate.utime = current_utime()
                rotate.trans_v = 0.0
                rotate.angular_v = -1.0

                rotate_time = timestamp_t()
                rotate_time.utime = rotate.utime
                lc.publish("MBOT_TIMESYNC", rotate_time.encode())
                lc.publish("MBOT_MOTOR_COMMAND", rotate.encode())
                time.sleep(0.1)

    # Stop
    stop = mbot_motor_command_t()
    stop.utime = current_utime()
    stop.trans_v = 0.0
    stop.angular_v = 0.0

    stop_time = timestamp_t()
    stop_time.utime = stop.utime
    lc.publish("MBOT_TIMESYNC", stop_time.encode())
    lc.publish("MBOT_MOTOR_COMMAND", stop.encode())
    time.sleep(0.1)


def drive_forward(vel, duration):
    global DRIFT_FACTOR
    drive = mbot_motor_command_t()
    drive.utime = current_utime()
    drive.trans_v = vel
    drive.angular_v = DRIFT_FACTOR #SPECIFIC TO EACH MBOT

    drive_time = timestamp_t()
    drive_time.utime = drive.utime

    lc.publish("MBOT_TIMESYNC", drive_time.encode())
    lc.publish("MBOT_MOTOR_COMMAND", drive.encode())
    time.sleep(duration)

    # Stop
    stop = mbot_motor_command_t()
    stop.utime = current_utime()
    stop.trans_v = 0.0
    stop.angular_v = 0.0

    stop_time = timestamp_t()
    stop_time.utime = stop.utime
    lc.publish("MBOT_TIMESYNC", stop_time.encode())
    lc.publish("MBOT_MOTOR_COMMAND", stop.encode())
    time.sleep(0.1)


lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

def orientation(i,path):
    global NEW_ORIENTATION
    if (path[i][0] < STARTX): 
        #call another function here to see if you need to turn
        NEW_ORIENTATION = "WEST"
    elif (path[i][0] > STARTX):
        #call another function here to see if you need to turn
        NEW_ORIENTATION = "EAST"
    elif (path[i][1] < STARTY):
        #call another function here to see if you need to turn
        NEW_ORIENTATION = "SOUTH"
    elif (path[i][1] > STARTY):
        #call another function here to see if you need to turn
        NEW_ORIENTATION = "NORTH"


def turn():
    global ORIENTATION
    global NEW_ORIENTATION
    global R_TURN_P
    global L_TURN_P

    has_turned = False #TIMING CHANGE

    if (ORIENTATION == NEW_ORIENTATION):
        return
    else:
        if(ORIENTATION == "WEST"):
            if(NEW_ORIENTATION == "NORTH"):
                turn_angle(-np.pi/R_TURN_P)
            elif(NEW_ORIENTATION == "EAST"):
                turn_angle(np.pi) #turn backwards
            elif(NEW_ORIENTATION == "SOUTH"):
                turn_angle(np.pi/L_TURN_P)
            has_turned = True #TIMING CHANGE

        if(ORIENTATION == "EAST"):
            if(NEW_ORIENTATION == "NORTH"):
                turn_angle(-np.pi/L_TURN_P)
            elif(NEW_ORIENTATION == "WEST"):
                turn_angle(np.pi) #turn backwards
            elif(NEW_ORIENTATION == "SOUTH"):
                turn_angle(-np.pi/R_TURN_P)
            has_turned = True #TIMING CHANGE

        if(ORIENTATION == "SOUTH"):
            if(NEW_ORIENTATION == "EAST"):
                turn_angle(np.pi/L_TURN_P)
            elif(NEW_ORIENTATION == "NORTH"):
                turn_angle(np.pi) #turn backwards
            elif(NEW_ORIENTATION == "WEST"):
                turn_angle(-np.pi/R_TURN_P)
            has_turned = True #TIMING CHANGE

        if(ORIENTATION == "NORTH"):
            if(NEW_ORIENTATION == "WEST"):
                turn_angle(np.pi/L_TURN_P)
            elif(NEW_ORIENTATION == "SOUTH"):
                turn_angle(np.pi) #turn backwards
            elif(NEW_ORIENTATION == "EAST"):
                turn_angle(-np.pi/R_TURN_P)
            has_turned = True #TIMING CHANGE

    ORIENTATION = NEW_ORIENTATION
    if(has_turned == False):
        time.sleep(2) #TIMING CHANGE


def main():
    print ("start\n")
    path = client.my_client()
    print(path)
    print("row: ", len(path), "; col: ", len(path[0]))

    print("creating LCM ...")
    lcm_kill_thread = Thread(target = handle_lcm, args= (lc, ), daemon = True)
    lcm_kill_thread.start()
    time.sleep(0.1)

    print('resetting odometry')
    reset_odometry_msg = reset_odometry_t()
    lc.publish("RESET_ODOMETRY", reset_odometry_msg.encode())
    time.sleep(0.1)

    # Get init location and orientation
    global ORIENTATION
    ORIENTATION = get_init_orientation(path[0][0], path[0][1])
    print("Starting Position:", path[0][0], path[0][1])  
    print("Starting ORIENTATION:", ORIENTATION)

    drive_time = timestamp_t()
    drive_time.utime = current_utime()
    lc.publish("MBOT_TIMESYNC", drive_time.encode())
    time.sleep(0.1)

    subscription = lc.subscribe("ODOMETRY", odometry_message_handler)
    time.sleep(0.1)

    time.sleep(1)

    global STARTX
    global STARTY
    global DISTANCE_FACTOR
    #ROW = port_num - 6666 #indexes the robot's row of the main coordinate array
    for i in range(len(path)-1): 
        #**************CHECK WITH YUCHEN THE MAX NUMBER OF STEPS*******************************
        STARTX = path[i][0] #UPDATE CURRENT POSITION
        STARTY = path[i][1]
        if (path[i+1][0] == path[i][0] & path[i+1][1] == path[i][1]):
            #do nothing; ending condition
            dummy = 1
        else:    
            orientation(i+1, path)
            turn()
            time.sleep(0.4)                     #TIMING CHANGE

            drive_forward(DISTANCE_FACTOR, 2)   #SPECIFIC TO EACH MBOT
            time.sleep(0.4)                     #TIMING CHANGE



if __name__== "__main__":
    main()
