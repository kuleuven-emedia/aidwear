import serial
import time
from ctypes import *

import ctypes.util
from ctypes import CDLL

name = ctypes.util.find_library("EposCmd")  # should resolve to libEposCmd.so
if not name:
  raise OSError("libEposCmd.so not found in linker path")
epos = CDLL(name)








# declare return/arg types for the few calls we use
epos.VCS_OpenDevice.restype = c_void_p
epos.VCS_OpenDevice.argtypes = [c_char_p, c_char_p, c_char_p, c_char_p, POINTER(c_uint)]
epos.VCS_GetErrorInfo.argtypes = [c_uint, c_char_p, c_ushort]

def get_err_text(code):
    buf = create_string_buffer(1024)
    epos.VCS_GetErrorInfo(code, buf, 1024)
    return buf.value.decode(errors="ignore")

def CHECK(ok, what, perr):
    if not ok:
        raise RuntimeError(f"{what} failed: 0x{perr.value:08X} – {get_err_text(perr.value)}")



# Load library
#cdll.LoadLibrary(path)
#epos = CDLL(path)

# Defining return variables from Library Functions
ret = 0
pErrorCode = c_uint()
pDeviceErrorCode = c_uint()

# Defining a variable NodeID and configuring connection
nodeID = 1
baudrate = 1000000
timeout = 500

# Configure desired motion profile
acceleration = 30000 # rpm/s, up to 1e7 would be possible
deceleration = 30000 # rpm/s

# Query motor position
def GetPositionIs():
    pPositionIs=c_long()
    pErrorCode=c_uint()
    ret=epos.VCS_GetPositionIs(keyHandle, nodeID, byref(pPositionIs), byref(pErrorCode))
    return pPositionIs.value # motor steps

# Move to position at speed
def MoveToPositionSpeed(target_position,target_speed):
    while True:
        if target_speed != 0:
            epos.VCS_SetPositionProfile(keyHandle, nodeID, target_speed, acceleration, deceleration, byref(pErrorCode)) # set profile parameters
            epos.VCS_MoveToPosition(keyHandle, nodeID, target_position, True, True, byref(pErrorCode)) # move to position
        elif target_speed == 0:
            epos.VCS_HaltPositionMovement(keyHandle, nodeID, byref(pErrorCode)) # halt motor
        true_position = GetPositionIs()
        if true_position == target_position:
            break

if __name__ == "__main__":
    Min_Value=4294778497
    Max_Value=100000

  


    # Initiating connection and setting motion profile

    keyHandle = epos.VCS_OpenDevice(b'EPOS4', b'CANopen', b'CAN_mcp251xfd 0', b'CAN0', byref(pErrorCode)) # specify EPOS version and interface
    if not keyHandle:
        raise RuntimeError(f"OpenDevice returned NULL: 0x{pErrorCode.value:08X} – {get_err_text(pErrorCode.value)}")

    epos.VCS_SetProtocolStackSettings(keyHandle, baudrate, timeout, byref(pErrorCode)) # set baudrate
    
    epos.VCS_ClearFault(keyHandle, nodeID, byref(pErrorCode)) # clear all faults
    
    epos.VCS_ActivateProfilePositionMode(keyHandle, nodeID, byref(pErrorCode)) # activate profile position mode
    
    epos.VCS_SetEnableState(keyHandle, nodeID, byref(pErrorCode)) # enable device
    print(' Initial postion Motor position: %s' % (GetPositionIs()))

    #MoveToPositionSpeed(Min_Value,5000) # move to position 20,000 steps at 5000 rpm/s
    print('Motor position: %s' % (GetPositionIs()))
    time.sleep(1)
    #MoveToPositionSpeed(Max_Value,12500) # move to position 20,000 steps at 5000 rpm/s
    print('Motor position: %s' % (GetPositionIs()))
    time.sleep(1)

    #MoveToPositionSpeed(Max_Value,16000) # move to position 20,000 steps at 5000 rpm/s
    print('Motor position: %s' % (GetPositionIs()))
    time.sleep(1)

   # MoveToPositionSpeed(0,2000) # move to position 0 steps at 2000 rpm/s

    MoveToPositionSpeed(200000,2000) # move to position 0 steps at 2000 rpm/s
    print('Motor position: %s' % (GetPositionIs()))

   #for i in range(100000, 200000, 1000): #Max is around 100000
    #    MoveToPositionSpeed(i, 2000)  # move to position i steps at 2000 rpm/s
     #   print(f"Motor position: {GetPositionIs()}")
    
    time.sleep(1)

    epos.VCS_SetDisableState(keyHandle, nodeID, byref(pErrorCode)) # disable device
    epos.VCS_CloseDevice(keyHandle, byref(pErrorCode)) # close device


