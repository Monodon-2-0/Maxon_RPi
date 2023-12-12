# -*- coding: utf-8 -*-
"""
Created on Mon Dec 11 18:32:29 2023

@author: Innovacion
"""

import serial
import time
from ctypes import *

import pyfirmata

# board = pyfirmata.Arduino('COM3')

i = 0

# board.digital[9].mode = pyfirmata.SERVO


# def tail(valores):
#     board.digital[9].write(valores)


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
# keyHandle = 0
# Configure desired motion profile
acceleration = 30000  # rpm/s, up to 1e7 would be possible
deceleration = 30000  # rpm/s

# Query motor position
def GetPositionIs(node_n):
    pPositionIs = c_long()
    pErrorCode = c_uint()
    ret = epos.VCS_GetPositionIs(keyHandle, node_n, byref(pPositionIs), byref(pErrorCode))
    return pPositionIs.value  # motor steps

# Move to position at speed
def MoveToPositionSpeed(target_position, target_speed, node_n):
    while True:
        if target_speed != 0:
            epos.VCS_SetPositionProfile(keyHandle, 1, target_speed, acceleration, deceleration,
                                        byref(pErrorCode))  # set profile parameters
            epos.VCS_MoveToPosition(keyHandle, 1, target_position, True, True, byref(pErrorCode))  # move to position
            # time.sleep(0.8)
        elif target_speed == 0:
            epos.VCS_HaltPositionMovement(keyHandle, 1, byref(pErrorCode))  # halt motor
        true_position = GetPositionIs(node_n)
        if true_position == target_position:
            break


if __name__ == "__main__":
    # Initiating connection and setting motion profile
    keyHandle = epos.VCS_OpenDevice(b'EPOS4', b'MAXON SERIAL V2', b'USB', b'USB0',
                                    byref(pErrorCode))  # specify EPOS version and interface
    epos.VCS_SetProtocolStackSettings(keyHandle, baudrate, timeout, byref(pErrorCode))  # set baudrate
    epos.VCS_ClearFault(keyHandle, nodeID, byref(pErrorCode))  # clear all faults
    epos.VCS_ActivateProfilePositionMode(keyHandle, nodeID, byref(pErrorCode))  # activate profile position mode
    epos.VCS_SetEnableState(keyHandle, nodeID, byref(pErrorCode))  # enable device
    # epos.VCS_ClearFault(keyHandle, nodeID2, byref(pErrorCode)) # clear all faults
    # epos.VCS_ActivateProfilePositionMode(keyHandle, nodeID2, byref(pErrorCode)) # activate profile position mode
    # epos.VCS_SetEnableState(keyHandle, nodeID2, byref(pErrorCode)) # enable device
    MoveToPositionSpeed(0, 300, nodeID)
    time.sleep(1)
    # tail(160)
    for i in range(2):
        MoveToPositionSpeed(10000, 400, nodeID)  # move to position 20,000 steps at 5000 rpm/s
        # MoveToPositionSpeed(10000,200,nodeID2) # move to position 20,000 steps at 5000 rpm/s
        # print('Motor position %s: %s' % (i,GetPositionIs(nodeID)))
        # print('Motor position %s: %s' % (i,GetPositionIs(nodeID2)))
        # for j in range(160, 80, -1):
        #     tail(j)
        #     time.sleep(0.01)

        # time.sleep(1)

        MoveToPositionSpeed(-10000, 400, nodeID)  # move to position 0 steps at 2000 rpm/s
        # for j in range(80, 160, 1):
        #     tail(j)
        #     time.sleep(0.01)
    # print('Motor position: %s' % (GetPositionIs(nodeID)))
    # print('Motor position %s: %s' % (i,GetPositionIs(nodeID)))
    # print('Motor position %s: %s' % (i,GetPositionIs(nodeID2)))

    MoveToPositionSpeed(0, 200, nodeID)  # move to position 0 steps at 2000 rpm/s
    # print('Motor position: %s' % (GetPositionIs(nodeID2)))
    time.sleep(1)
    MoveToPositionSpeed(0, 300, nodeID)
    epos.VCS_SetDisableState(keyHandle, nodeID, byref(pErrorCode))  # disable device
    epos.VCS_SetDisableState(keyHandle, nodeID2, byref(pErrorCode))  # disable device
    epos.VCS_CloseDevice(keyHandle, byref(pErrorCode))  # close device
# board.close()
