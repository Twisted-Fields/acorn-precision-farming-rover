
# Linux example for external capture trigger on econsystems FSCAM_CU135
# By Taylor Alexander, MIT License. Please enjoy, expand, and share.
# Run as root or add a udev rule to give user appropriate rights.

# Note: Hook the hardware trigger input up to an arduino or other device
# with a pin toggling on and off at 10hz (for example).
# Connect a webcam viewer such as "cheese" to view the camera, and then
# run this program in a terminal. You will see the webcam stream slow to 10hz.

import hid # pip3 install hidapi
import time


CAMERA_CONTROL_FSCAM_CU135 = 0x95
GET_LED_CONTROL_FSCAM_CU135 = 0x26
BUFFER_LENGTH = 65

SET_FAIL = 0x00
SET_SUCCESS = 0x01

GET_FAIL = 0x00
GET_SUCCESS = 0x01

SET_FLICKER_DETECTION_FSCAM_CU135 = 0x29
GET_FLICKER_DETECTION_FSCAM_CU135 = 0x28

ENABLE_LED_CONTROL_FSCAM_CU135 = 0x01
DISABLE_LED_CONTROL_FSCAM_CU135 = 0x00

ENABLE_POWERON_CONTROL_FSCAM_CU135 = 0x01
DISABLE_POWERON_CONTROL_FSCAM_CU135 = 0x00

ENABLE_STREAMING_CONTROL_FSCAM_CU135 = 0x01
DISABLE_STREAMING_CONTROL_FSCAM_CU135 = 0x00

ENABLE_TRIGGERACK_CONTROL_FSCAM_CU135 = 0x01
DISABLE_TRIGGERACK_CONTROL_FSCAM_CU135 = 0x00

CAMERA_CONTROL_FSCAM_CU135 = 0x95

SET_SPECIAL_EFFECT_MODE_FSCAM_CU135 = 0x04
GET_SPECIAL_EFFECT_MODE_FSCAM_CU135 = 0x03

SET_SCENE_MODE_FSCAM_CU135 = 0x02
GET_SCENE_MODE_FSCAM_CU135 = 0x01

SET_DENOISE_CONTROL_FSCAM_CU135 = 0x06
GET_DENOISE_CONTROL_FSCAM_CU135 = 0x05

GET_Q_FACTOR_FSCAM_CU135 = 0x0B
SET_Q_FACTOR_FSCAM_CU135 = 0x0C

SET_HDR_MODE_FSCAM_CU135 = 0x0A
GET_HDR_MODE_FSCAM_CU135 = 0x09

SET_STREAM_MODE_FSCAM_CU135 = 0x0E
GET_STREAM_MODE_FSCAM_CU135 = 0x0D

GET_LED_CONTROL_FSCAM_CU135 = 0x26
SET_LED_CONTROL_FSCAM_CU135 = 0x27

SET_ORIENTATION_FSCAM_CU135 = 0x11
GET_ORIENTATION_FSCAM_CU135 = 0x10

STREAM_MASTER_CONTINUOUS = 0x00
STREAM_MASTER_ONDEMAND = 0x01
STREAM_SOFTWARE_TRIGGER = 0x02
STREAM_HARDWARE_TRIGGER = 0x03

GRAB_PREVIEW_FRAME = 0x1A
GRAB_STILL_FRAME = 0x1E
STORE_FRAME = 0x1B
QUERY_NEXT_FRAME = 0x01
STORE_PREV_FRAME = 0x01
STORE_STILL_FRAME = 0x02


# Example snippett from https://www.ontrak.net/pythonhidapi.htm
VENDOR_ID = 0x2560
PRODUCT_ID = 0xC1D4

# print(hid.enumerate(VENDOR_ID, PRODUCT_ID))

# enumerate USB devices

paths = []

for d in hid.enumerate(VENDOR_ID, PRODUCT_ID):
    if int(d['interface_number']) == 2:
        paths.append(d['path'])
        print(d['path'])
    # keys = list(d.keys())
    # keys.sort()
    # print(d['path'])
    # for key in keys:
    #     print("%s : %s" % (key, d[key]))
    # print()
#

# device = hid.device()
# # device.open(VENDOR_ID, PRODUCT_ID)
#
#
# print(dir(device))
# print(device)
#
# device.close()
#
# import sys
# sys.exit()

devices = [hid.device(), hid.device()]
# devices = [hid.device()]
devices[0].open_path(paths[0])
devices[1].open_path(paths[1])

for device in devices:
    # device = hid.device()
    # device.open(VENDOR_ID, PRODUCT_ID)
    print('Connected to ecam {}\n'.format(PRODUCT_ID))


    timeout = 0

    # First just read LED Control status, as a test.

    g_out_packet_buf = [0, 0]
    g_out_packet_buf[0] = CAMERA_CONTROL_FSCAM_CU135
    g_out_packet_buf[1] = GET_LED_CONTROL_FSCAM_CU135

    device.write(g_out_packet_buf)
    time.sleep(0.5)

    data = device.read(BUFFER_LENGTH, timeout)
    print(data)

    # if data[6]==GET_SUCCESS:
    if data[0] == CAMERA_CONTROL_FSCAM_CU135 and data[1]==GET_LED_CONTROL_FSCAM_CU135 and data[6]==GET_SUCCESS:
        ledstatus=data[2]
        powerctl=data[3]
        stream=data[4]
        trigger=data[5]
        print("ledstatus {}, powerctl {}, stream {}, trigger {}".format(ledstatus, powerctl, stream, trigger))
    else:
        print("GET_FAILED")

    # Now set LED control to indicate when hardware trigger has activated.

    g_out_packet_buf = [0, 0, 0, 0, 0, 0]
    g_out_packet_buf[0] = CAMERA_CONTROL_FSCAM_CU135 # /* set camera control code */
    g_out_packet_buf[1] = SET_LED_CONTROL_FSCAM_CU135 # /* set led control code */
    g_out_packet_buf[2] = ENABLE_LED_CONTROL_FSCAM_CU135
    g_out_packet_buf[3] = DISABLE_STREAMING_CONTROL_FSCAM_CU135
    g_out_packet_buf[4] = ENABLE_TRIGGERACK_CONTROL_FSCAM_CU135
    g_out_packet_buf[5] = DISABLE_POWERON_CONTROL_FSCAM_CU135
    device.write(g_out_packet_buf)
    time.sleep(0.5)

    data = device.read(BUFFER_LENGTH, timeout)
    #print(data)

    # Finally set trigger control.

    g_out_packet_buf = [0, 0, 0, 0]
    g_out_packet_buf[1] = CAMERA_CONTROL_FSCAM_CU135  # /* set camera control code */
    g_out_packet_buf[2] = SET_STREAM_MODE_FSCAM_CU135  # /* set stream mode code */
    g_out_packet_buf[3] = STREAM_HARDWARE_TRIGGER  # /* actual stream mode */
    # g_out_packet_buf[3] = STREAM_MASTER_CONTINUOUS  # NOTE: Uncomment this to select auto trigger.

    device.write(g_out_packet_buf)

    time.sleep(2)

    data = device.read(BUFFER_LENGTH, timeout)
    if data[0] == CAMERA_CONTROL_FSCAM_CU135 and data[1]==SET_STREAM_MODE_FSCAM_CU135 and data[6]==SET_SUCCESS:
        print("SUCCESS")
    else:
        print("FAILED")

    time.sleep(2)

while True:
    for device in devices:
        # In hardware trigger mode we must continually request the next frame.
        g_out_packet_buf[1] = CAMERA_CONTROL_FSCAM_CU135 # // camera control id
        g_out_packet_buf[2] = GRAB_PREVIEW_FRAME # // query frame
        g_out_packet_buf[3] = QUERY_NEXT_FRAME # // query next frame - 0x01 , query prev frame - 0x02
        device.write(g_out_packet_buf)
        data = device.read(BUFFER_LENGTH, timeout)
        print(time.time())
    time.sleep(0.001)
