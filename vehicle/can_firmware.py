import isotp
import motor_controller as mot
import time
import random
import traceback
import sys
import math
import socket

ADDRESS=0

if len(sys.argv) == 4:
    ADDRESS = int(sys.argv[1])
    FLASH_CAN = sys.argv[2]=="1"
    FLASH_MOTOR = sys.argv[3]=="1"
else:
    print("Need 3 arguments: dest_address, flash_can (0/1), flash_motor (0/1)")


CAN_DEST_NAME = "firmware_can.bin"
MOTOR_DEST_NAME = "firmware_motor.bin"


controller = mot.MotorController(id=ADDRESS)

FIRMWARE_UPDATE=7
FIRMWARE_UPDATE_CPU2=8


s = isotp.socket()
s.set_opts(isotp.socket.flags.WAIT_TX_DONE, frame_txtime=250000,tx_stmin=250000)
s.bind("can1", isotp.Address(rxid=0x1, txid=ADDRESS))

def send_file(filename, update_type):

    fileContent = bytearray()
    with open(filename, mode='rb') as file: # b is important -> binary
        fileContent = list(file.read())

    print("start")
    for i in range(10):
        print(hex(fileContent[i]))

    fileContent.insert(0, update_type)

    try:
        s.send(bytearray(fileContent))
    except Exception as e:
        raise e
        print("Send Error")


if FLASH_MOTOR:
    send_file(MOTOR_DEST_NAME, FIRMWARE_UPDATE_CPU2)
    print()
    if FLASH_CAN:
        time.sleep(2)

if FLASH_CAN:
    send_file(CAN_DEST_NAME, FIRMWARE_UPDATE)
