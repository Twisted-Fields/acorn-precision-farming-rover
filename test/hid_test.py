
import hid
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

STREAM_MASTER_CONTINUOUS = 0x00,
STREAM_MASTER_ONDEMAND = 0x01,
STREAM_SOFTWARE_TRIGGER = 0x02,
STREAM_HARDWARE_TRIGGER = 0x03

VENDOR_ID = 0x2560
PRODUCT_ID = 0xC1D4
device = hid.device()
device.open(VENDOR_ID, PRODUCT_ID)
print('Connected to ecam {}\n'.format(PRODUCT_ID))

ledcontrol = True

timeout = 0
while True:
    g_out_packet_buf = [0, 0]

    g_out_packet_buf[0] = CAMERA_CONTROL_FSCAM_CU135
    g_out_packet_buf[1] = GET_LED_CONTROL_FSCAM_CU135

    device.write(g_out_packet_buf)
    time.sleep(0.5)

    data = device.read(BUFFER_LENGTH, timeout)
    print(data)

    if data[6]==GET_SUCCESS:
        if data[0] == CAMERA_CONTROL_FSCAM_CU135 and data[1]==GET_LED_CONTROL_FSCAM_CU135 and data[6]==GET_SUCCESS:
            ledstatus=data[2]
            powerctl=data[3]
            stream=data[4]
            trigger=data[5]



    print("ledstatus {}, powerctl {}, stream {}, trigger {}".format(ledstatus, powerctl, stream, trigger))
    import sys
    sys.exit()
    g_out_packet_buf = [0, 0, 0, 0, 0 , 0]
    g_out_packet_buf[0] = CAMERA_CONTROL_FSCAM_CU135 # /* set camera control code */
    g_out_packet_buf[1] = SET_LED_CONTROL_FSCAM_CU135 # /* set led control code */
    if ledcontrol:
        g_out_packet_buf[2] = ENABLE_LED_CONTROL_FSCAM_CU135
    else:
        g_out_packet_buf[2] = DISABLE_LED_CONTROL_FSCAM_CU135
    g_out_packet_buf[5] = ENABLE_POWERON_CONTROL_FSCAM_CU135
    g_out_packet_buf[3] = ENABLE_STREAMING_CONTROL_FSCAM_CU135
    g_out_packet_buf[4] = ENABLE_TRIGGERACK_CONTROL_FSCAM_CU135
    g_out_packet_buf[5] = 2

    device.write(g_out_packet_buf)
    time.sleep(0.5)

    data = device.read(BUFFER_LENGTH, timeout)
    #print(data)

    ledcontrol = ledcontrol==False




  g_out_packet_buf[1] = CAMERA_CONTROL_FSCAM_CU135; /* set camera control code */
    g_out_packet_buf[2] = SET_STREAM_MODE_FSCAM_CU135; /* set stream mode code */
    g_out_packet_buf[3] = streamMode; /* actual stream mode */
