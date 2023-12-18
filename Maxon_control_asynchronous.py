# -*- coding: utf-8 -*-
"""
Created on Mon Dec 11 18:32:29 2023

@author: Innovacion
"""


import time
import threading

from ctypes import *
from pymavlink import mavutil


# EPOS Command Library path
path = '../../opt/EposCmdLib_6.8.1.0/lib/v7/libEposCmd.so.6.8.1.0'

# Load library
cdll.LoadLibrary(path)
epos = CDLL(path)

# Defining return variables from Library Functions
ret = 0
pErrorCode = c_uint()
pDeviceErrorCode = c_uint()

# Defining a variable NodeID and configuring connection
nodeID = 1
nodeID2 = 2
baudrate = 1000000
timeout = 500
# Configure desired motion profile
acceleration = 30000  # rpm/s, up to 1e7 would be possible
deceleration = 30000  # rpm/s
#Servo Ports
servo_port_left = 8
servo_port_right = 13

def set_servo_pwm(servo_n, microseconds):
    """ Sets AUX 'servo_n' output PWM pulse-width.

    Uses https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO

    'servo_n' is the AUX port to set (assumes port is configured as a servo).
        Valid values are 1-3 in a normal BlueROV2 setup, but can go up to 8
        depending on Pixhawk type and firmware.
    'microseconds' is the PWM pulse-width to set the output to. Commonly
        between 1100 and 1900 microseconds.

    """
    # master.set_servo(servo_n+8, microseconds) or:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,  # first transmission of this command
        servo_n,
        microseconds,  # PWM pulse-width
        0, 0, 0, 0, 0  # unused parameters
    )

# Query motor position
def get_position_is(node_n):
    pPositionIs = c_long()
    pErrorCode = c_uint()
    epos.VCS_GetPositionIs(keyHandle, node_n, byref(pPositionIs), byref(pErrorCode))
    return pPositionIs.value

# Move to position at speed
def move_to_position_speed(target_position, target_speed, node_n, servo_port):
    internal_variable = 0
    print("Moving to the reference proposed....")
    us_camera = 1100
    us_tail = 1200
    back = 0
    back_c = 0
    while True:
        if target_speed != 0:
            epos.VCS_SetPositionProfile(keyHandle, node_n, target_speed, acceleration, deceleration,
                                        byref(pErrorCode))  # set profile parameters
            epos.VCS_MoveToPosition(keyHandle, node_n, target_position, True, True, byref(pErrorCode))  # move to position
            epos.VCS_SetPositionProfile(keyHandle, 2, target_speed, acceleration, deceleration,
                                        byref(pErrorCode))  # set profile parameters
            epos.VCS_MoveToPosition(keyHandle, 2, -target_position, True, True,
                                    byref(pErrorCode))  # move to position
            internal_variable += 1

        elif target_speed == 0:
            epos.VCS_HaltPositionMovement(keyHandle, node_n, byref(pErrorCode))  # halt motor
            epos.VCS_HaltPositionMovement(keyHandle, 2, byref(pErrorCode))  # halt motor

        true_position = get_position_is(node_n)
        us = int((true_position + 10000)/14.3) + 800
        us2 = 2200 - (us - 800)
        set_servo_pwm(servo_port, us2)
        set_servo_pwm(13, us)

        # Implementar que gire la camara y el servo de atras
        if us_tail == 1400:
            back = 1
        elif us_tail == 1200:
            back = 0
        if back == 1:
            us_tail = us_tail - 20
        else:
            us_tail = us_tail + 20
        if us_camera == 1900:
            back_c = 1
        elif us_camera == 1100:
            back_c = 0
        if back_c == 1:
            us_camera = us_camera - 20
        else:
            us_camera = us_camera + 20
        print(f"US_CAMERA  {us_camera} y US_TAIL {us_tail}" )
        set_servo_pwm(15, us_camera)
        set_servo_pwm(10, us_tail)
        set_servo_pwm(10, us)
        print(f"microsegundos de pwm {us} y posicion del motor {get_position_is(node_n)}", )
        if true_position == target_position:
            break


if __name__ == "__main__":
    # Initiating connection and setting motion profile
    # Create the connection
    # Created endpoint to localhost (127.0.0.1) in the BlueOS interface, PirateMode.
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14770')
    master.wait_heartbeat()
    print('Connection success with navigator')

    #Setup Motor Left
    keyHandle = epos.VCS_OpenDevice(b'EPOS4', b'MAXON SERIAL V2', b'USB', b'USB0',
                                    byref(pErrorCode))  # specify EPOS version and interface
    epos.VCS_SetProtocolStackSettings(keyHandle, baudrate, timeout, byref(pErrorCode))  # set baudrate
    epos.VCS_ClearFault(keyHandle, nodeID, byref(pErrorCode))  # clear all faults
    epos.VCS_ActivateProfilePositionMode(keyHandle, nodeID, byref(pErrorCode))  # activate profile position mode
    epos.VCS_SetEnableState(keyHandle, nodeID, byref(pErrorCode))  # enable device

    # Setup Motor Right (Different Fins, just 2 Maxons rn)
    epos.VCS_ClearFault(keyHandle, nodeID2, byref(pErrorCode))  # clear all faults
    epos.VCS_ActivateProfilePositionMode(keyHandle, nodeID2, byref(pErrorCode))  # activate profile position mode
    epos.VCS_SetEnableState(keyHandle, nodeID2, byref(pErrorCode))  # enable device

    move_to_position_speed(0, 300, 1, 8)
    time.sleep(1)
    for s in range(2):
        move_to_position_speed(10000, 400, 1, 8)
        move_to_position_speed(-10000, 400, 1, 8)

        # move to position 0 steps at 2000 rpm/s

    move_to_position_speed(0, 200, 1, 8)  # move to position 0 steps at 2000 rpm/s
    # print('Motor position: %s' % (GetPositionIs(nodeID2)))
    set_servo_pwm(8, 1500)
    set_servo_pwm(13, 1500)
    time.sleep(1)
    move_to_position_speed(0, 300, 1, 8)
    print("Connection success with the EPOS")

    # y = threading.Thread(target=motor_right)
    # y.start()

    epos.VCS_SetDisableState(keyHandle, nodeID, byref(pErrorCode))  # disable device
    epos.VCS_SetDisableState(keyHandle, nodeID2, byref(pErrorCode))  # disable device
    epos.VCS_CloseDevice(keyHandle, byref(pErrorCode))  # close device
# board.close()
