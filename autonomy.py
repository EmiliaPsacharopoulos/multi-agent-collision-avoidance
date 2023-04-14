import lcm
import numpy as np
from lcmtypes import mbot_motor_command_t, timestamp_t
import time
from lcmtypes import odometry_t
from lcmtypes import reset_odometry_t
from threading import Thread, Lock, current_thread
import client

DRIVE_LENGTH = 3
STOP_LENGTH = 0.5
ROTATE_LENGTH = 3
WHEEL_BASE = 0.15
WHEEL_DIAMETER = 0.084
STARTX = 0 #start means the current coordinate
STARTY = 0 #start means the current coordinate
ORIENTATION = "NORTH"
ROW = 0
NEW_ORIENTATION = "NORTH"
curr_odometry = None


def handle_lcm(lcm_obj):
    try:
        while True:
            lcm_obj.handle()
    except KeyboardInterrupt:
        print("lcm exit!")
        sys.exit()


def current_utime(): return int(time.time() * 1e6)

def odometry_message_handler(channel, data):
    global curr_odometry
    curr_odometry = odometry_t.decode(data)

def turn_angle(angle):
    angle = angle * 0.92
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
    drive = mbot_motor_command_t()
    drive.utime = current_utime()
    drive.trans_v = vel
    drive.angular_v = 0.0

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
    #3D array: ROW indexes the robot, i indexes the next coordinate, then 0=x and 1=y
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
    if (ORIENTATION == NEW_ORIENTATION):
        return
    else:
        if(ORIENTATION == "WEST"):
            if(NEW_ORIENTATION == "NORTH"):
                turn_angle(np.pi/2) #CW
            elif(NEW_ORIENTATION == "EAST"):
                turn_angle(np.pi) #turn backwards
            elif(NEW_ORIENTATION == "SOUTH"):
                turn_angle(-np.pi/2) #CCW

        if(ORIENTATION == "EAST"):
            if(NEW_ORIENTATION == "NORTH"):
                turn_angle(-np.pi/2) #CCW
            elif(NEW_ORIENTATION == "WEST"):
                turn_angle(np.pi) #turn backwards
            elif(NEW_ORIENTATION == "SOUTH"):
                turn_angle(np.pi/2) #CW

        if(ORIENTATION == "SOUTH"):
            if(NEW_ORIENTATION == "EAST"):
                turn_angle(-np.pi/2) #CCW
            elif(NEW_ORIENTATION == "NORTH"):
                turn_angle(np.pi) #turn backwards
            elif(NEW_ORIENTATION == "WEST"):
                turn_angle(np.pi/2) #CW

        if(ORIENTATION == "NORTH"):
            if(NEW_ORIENTATION == "WEST"):
                turn_angle(-np.pi/2) #CCW
            elif(NEW_ORIENTATION == "SOUTH"):
                turn_angle(np.pi) #turn backwards
            elif(NEW_ORIENTATION == "EAST"):
                turn_angle(np.pi/2) #CW
    ORIENTATION = NEW_ORIENTATION

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

    drive_time = timestamp_t()
    drive_time.utime = current_utime()
    lc.publish("MBOT_TIMESYNC", drive_time.encode())
    time.sleep(0.1)

    subscription = lc.subscribe("ODOMETRY", odometry_message_handler)
    time.sleep(0.1)

    time.sleep(1)

    global STARTX
    global STARTY

    #ROW = port_num - 6666 #indexes the robot's row of the main coordinate array
    for i in range(len(path)-1): #**************CHECK WITH YUCHEN THE MAX NUMBER OF STEPS*******************************
        STARTX = path[i][0] #UPDATE CURRENT POSITION
        STARTY = path[i][1]
        if (path[i+1][0] == path[i][0] & path[i+1][1] ==  path[i][1]):
            #do nothing; ending condition
            dummy = 1
        else:    
            orientation(i+1,path)
            turn()
            time.sleep(1)

            drive_forward(0.3048,1) #theoretically what it takes to move 1 foot per second
            time.sleep(1)    
            """    
            if i == 0:
                #starting coordinate for the selected robot
                drive_forward(0.3048,1) #theoretically what it takes to move 1 foot per second
                time.sleep(1)

            else:
                orientation(i+1,path)
                turn()
                time.sleep(1)

                drive_forward(0.3048,1) #theoretically what it takes to move 1 foot per second
                time.sleep(1)
"""
                
                
                

if __name__== "__main__":
    main()

