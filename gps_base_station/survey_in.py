import serial
import pyubx2
from pyubx2 import UBXMessage, UBXReader, POLL
import time


PORT="/dev/ublox9"
BAUD = 9600

MODE_SURVEY_IN = 0
MODE_SET_BASE_STATION_VALUES = 1

MODE_SETTING = MODE_SURVEY_IN

TMODE_SVIN=1
TMODE_FIXED=2

LAYER_RAM = 1
LAYER_BBR = 2
LAYER_FLASH = 4

POS_ECEF = 0


serial_port = serial.Serial(PORT, BAUD, timeout=0.1)

"""
layers - 1 = Volatile RAM, 2 = Battery-Backed RAM (BBR), 4 = External Flash (may be OR'd)
transaction - 0 = None, 1 = Start, 2 = Ongoing, 3 = Commit
cfgData - an array of up to 64 (key, value) tuples. Keys can be in either keyID (int) or keyname (str) format
"""

if MODE_SETTING == MODE_SURVEY_IN:

    layers = LAYER_RAM
    transaction = 0
    cfgData = [("CFG_TMODE_MODE", TMODE_SVIN), ("CFG_TMODE_POS_TYPE", POS_ECEF)]
    msg = UBXMessage.config_set(layers, transaction, cfgData)
    # print(msg)
    # <UBX(CFG-VALSET, version=0, ram=1, bbr=0, flash=0, action=0, reserved0=0, cfgData_01=1, cfgData_02=0 ...)>
    # serial_port.write(msg.serialize())

    # <UBX(NAV-SVIN, version=0, reserved1=0, iTOW=00:14:11, dur=144001, meanX=-271498268, meanY=-428928009, meanZ=384875478, meanXHP=18, meanYHP=-30, meanZHP=8, reserved2=0, meanAcc=659, obs=144000, valid=1, active=0, reserved3=0)>


    time.sleep(2)

    layer = LAYER_RAM
    position = 0
    keys = ["CFG_MSGOUT_UBX_NAV_SVIN_USB"]
    # msgClass=UBX_CLASSES["NAV"], msgID=0x03,
    msg = UBXMessage('NAV','NAV-SVIN', POLL, rateUSB=1)

    # msg = UBXMessage.config_poll(layer, position, keys)
    serial_port.write(msg.serialize())


    stream_reader = UBXReader(serial_port)

    while True:
        if serial_port.in_waiting:
            (raw_data, data) = stream_reader.read()
            if data is not None:
                print("------------")
                # print(dir(data))
                print(data)
                # try:
                #     print("SOMETHING PRINTS BELOW")
                #     print(data.CFG_MSGOUT_UBX_NAV_SVIN_USB)
                # except:
                #     pass
                print("------------")
                serial_port.write(msg.serialize())
        time.sleep(1)
        # if data.identity == "NAV-SVIN":
        #     serial_port.write(msg.serialize())
        # print("Status: {}, Observation Time: {}, Mean 3D Stdev: {}, X: {}, Y: {}, Z: {}")

"""

    "CFG_TMODE_ECEF_X_HP": (0x20030006, I1),
    "CFG_TMODE_ECEF_X": (0x40030003, I4),
    "CFG_TMODE_ECEF_Y_HP": (0x20030007, I1),
    "CFG_TMODE_ECEF_Y": (0x40030004, I4),
    "CFG_TMODE_ECEF_Z_HP": (0x20030008, I1),
    "CFG_TMODE_ECEF_Z": (0x40030005, I4),
    "CFG_TMODE_FIXED_POS_ACC": (0x4003000F, U4),
    "CFG_TMODE_HEIGHT_HP": (0x2003000E, I1),
    "CFG_TMODE_HEIGHT": (0x4003000B, I4),
    "CFG_TMODE_LAT_HP": (0x2003000C, I1),
    "CFG_TMODE_LAT": (0x40030009, I4),
    "CFG_TMODE_LON_HP": (0x2003000D, I1),
    "CFG_TMODE_LON": (0x4003000A, I4),
    "CFG_TMODE_MODE": (0x20030001, E1),
    "CFG_TMODE_POS_TYPE": (0x20030002, E1),
    "CFG_TMODE_SVIN_ACC_LIMIT": (0x40030011, U4),
    "CFG_TMODE_SVIN_MIN_DUR": (0x40030010, U4),

"""



if MODE_SETTING == MODE_SET_BASE_STATION_VALUES:

    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    print("WARNING THIS CODE DID NOT WORK LAST TIME IT WAS RUN! ")
    print("THE SETTINGS DID NOT SEEM TO GET SAVED TO FLASH")
    print("YOU MUST FIX THE CODE TO USE THIS FEATURE")
    print("EXITING NOW")
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    import sys
    sys.exit()

    meanX=-271498268
    meanY=-428928009
    meanZ=384875478
    meanXHP=18
    meanYHP=-30
    meanZHP=8

    cfgData = [
            ("CFG_TMODE_ECEF_X", meanX),
            ("CFG_TMODE_ECEF_Y", meanY),
            ("CFG_TMODE_ECEF_Z", meanZ),
            ("CFG_TMODE_ECEF_X_HP", meanXHP),
            ("CFG_TMODE_ECEF_Y_HP", meanYHP),
            ("CFG_TMODE_ECEF_Z_HP", meanZHP),
            ("CFG_TMODE_MODE", TMODE_FIXED),
            ("CFG_TMODE_POS_TYPE", POS_ECEF)
            ]
    layers = LAYER_RAM | LAYER_BBR | LAYER_FLASH
    transaction = 0
    msg = UBXMessage.config_set(layers, transaction, cfgData)
    serial_port.write(msg.serialize())

    time.sleep(1)


    keys = [
            "CFG_TMODE_ECEF_X",
            "CFG_TMODE_ECEF_Y",
            "CFG_TMODE_ECEF_Z",
            "CFG_TMODE_ECEF_X_HP",
            "CFG_TMODE_ECEF_Y_HP",
            "CFG_TMODE_ECEF_Z_HP",
            "CFG_TMODE_MODE",
            "CFG_TMODE_POS_TYPE"
            ]
    layer = LAYER_RAM
    position = 0 # How many results to skip.
    msg = UBXMessage.config_poll(layer, position, keys)

    serial_port.write(msg.serialize())


    stream_reader = UBXReader(serial_port)

    while True:
        if serial_port.in_waiting:
            (raw_data, data) = stream_reader.read()
            if data is not None:
                print("------------")
                print(data)
                print("------------")
                # serial_port.write(msg.serialize())
        time.sleep(1)
