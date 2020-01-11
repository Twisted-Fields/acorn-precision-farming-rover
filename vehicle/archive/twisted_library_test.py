
import corner_actuator
import serial
import time
import sys
import math
from odrive.utils import dump_errors
from evdev import InputDevice, list_devices, categorize, ecodes, KeyEvent
from steering import calculate_steering

COUNTS_PER_REVOLUTION = corner_actuator.COUNTS_PER_REVOLUTION

ACCELERATION_COUNTS_SEC = 0.5

_USE_JOYSTICK = True

odrive_devices = {
"front_right":"335E31483536",
"front_left":"335B314C3536",
"rear_right":"3352316E3536",
"rear_left":"205332784D4B"
}

if _USE_JOYSTICK:
    joy = None
    devices = [InputDevice(fn) for fn in list_devices()]
    for dev in devices:
        if "Microsoft" in dev.name:
            joy = dev
            break
        if "Logitech" in dev.name:
            joy = dev
            break
    if not joy:
        print("CONTROLLER NOT FOUND")
        sys.exit()
    last = {
        "ABS_RZ": 128,
        "ABS_Z": 128
    }



def get_joystick(joy, st_old, th_old):
    steer = None
    throttle = None
    count = 0
    #print("Enter_joy")
    while True:
        event = joy.read_one()
        #print(event)
        if event and event.type == ecodes.EV_ABS:
            absevent = categorize(event)
            #print(ecodes.bytype[absevent.event.type][absevent.event.code])
            if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_RX':
                steer = absevent.event.value / 32768.0
                # print("ABS_X: {}".format(absevent.event.value))
            #
            if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_Y':
                throttle = -absevent.event.value / 32768.0
        count += 1
        if steer and throttle:
            break
        if count > 12:
            break
    if not steer:
        steer = st_old
    if not throttle:
        throttle = th_old
    #print("Exit_joy")
    return steer, throttle

joy_steer = 0
joy_throttle = 0
vel_cmd = 0
last_vel_cmd = 0


def get_profiled_velocity(last_vel, unfiltered_vel, period_s):
    if math.fabs(unfiltered_vel-last_vel) < ACCELERATION_COUNTS_SEC * period_s:
        increment = unfiltered_vel-last_vel
    else:
        increment = math.copysign(ACCELERATION_COUNTS_SEC, unfiltered_vel-last_vel) * period_s
    return last_vel + increment

# last_vel = 0
#tick_time = time.time()
# while True:
#     joy_steer, joy_throttle = get_joystick(joy, joy_steer, joy_throttle)
#     #print(joy_steer)
#     vel_cmd = joy_throttle
#     period = time.time() - tick_time
#     vel_cmd = get_profiled_velocity(last_vel_cmd, vel_cmd, period)
#     last_vel_cmd = vel_cmd
#     #print(vel_cmd)
#     tick_time = time.time()
#     calc = calculate_steering(joy_steer, vel_cmd)
#     val = calc['front_left'][1]
#     print("steering: {0:0.2f}, vel_cmd: {1:0.2f}, calculated vel cmd {2:0.2f}".format(joy_steer, vel_cmd, val))
#     time.sleep(0.05)
# while True:
#     steer, throttle = get_joystick(joy, steer, throttle)
#     print("Steer: {}, Throttle: {}".format(steer,throttle))
#             #     print(last["ABS_Z"])


# ports = serial.tools.list_ports.comports()
# for port in ports:
#     for name, value in odrive_devices.items():
#         if isinstance(value, str) and port.serial_number == value:
#             odrive_devices[name] = port

print(odrive_devices)

front_right = corner_actuator.CornerActuator(serial_number=odrive_devices['front_right'], name='front_right')
front_left = corner_actuator.CornerActuator(serial_number=odrive_devices['front_left'], name='front_left')
rear_right = corner_actuator.CornerActuator(serial_number=odrive_devices['rear_right'], name='rear_right')
rear_left = corner_actuator.CornerActuator(serial_number=odrive_devices['rear_left'], name='rear_left')

odrives = [front_left, front_right, rear_right, rear_left]
#odrives = [rear_left]

for drive in odrives:
    drive.print_errors(clear_errors=True)
for drive in odrives:
    steering = 0.0
    if "rear" in drive.name:
        drive.initialize_steering(steering_flipped=True)
    else:
        drive.initialize_steering(steering_flipped=False)
    drive.update_actuator(steering, 0.0)
for drive in odrives:
    drive.initialize_traction()


# odrives[0].odrv0.axis1.controller.vel_integrator_current = 0
# odrives[0].odrv0.axis1.controller.vel_setpoint = 50
#
# time.sleep(100)

# A sine wave to test
tick_time = time.time()
t0 = time.monotonic()
motion_tick_time = t0
tick_index = 0
positions = [0,10, 20, 30, 45]#,90,180,-45, -90, -180]#,-360,-90,0,45,90,135,180,270,360]
try:
    while True:
        # Get joystick value
        joy_steer, joy_throttle = get_joystick(joy, joy_steer, joy_throttle)
        vel_cmd = joy_throttle
        period = time.time() - tick_time
        # Perform acceleration on joystick value
        vel_cmd = get_profiled_velocity(last_vel_cmd, vel_cmd, period)
        last_vel_cmd = vel_cmd
        print(vel_cmd)
        tick_time = time.time()
        calc = calculate_steering(joy_steer, vel_cmd)
        for drive in odrives:
            this_pos, this_vel_cmd = calc[drive.name]
            this_pos = math.degrees(this_pos)
            drive.update_actuator(this_pos, this_vel_cmd * 1000)
            time.sleep(0.02)

        error_found = False
        for drive in odrives:
            if drive.odrv0.axis0.error or drive.odrv0.axis1.error:
                error_found = True
        if error_found:
            for drive in odrives:
                print("Drive: {}:".format(drive.name))
                print(dump_errors(drive.odrv0))
                drive.stop_actuator()
            print("Exiting due to errors.")
            sys.exit()

        time.sleep(0.05)
except KeyboardInterrupt:
    for drive in odrives:
        drive.stop_actuator()
    pass




#   250.092559] usb 1-1.1.3.2: new full-speed USB device number 7 using dwc_otg
# [  250.238525] usb 1-1.1.3.2: New USB device found, idVendor=1209, idProduct=0d32, bcdDevice= 3.00
# [  250.238541] usb 1-1.1.3.2: New USB device strings: Mfr=1, Product=2, SerialNumber=3
# [  250.238551] usb 1-1.1.3.2: Product: ODrive 3.6 CDC Interface
# [  250.238560] usb 1-1.1.3.2: Manufacturer: ODrive Robotics
# [  250.238569] usb 1-1.1.3.2: SerialNumber: 335B314C3536
# [  250.342593] usb 1-1.1.3.3: new full-speed USB device number 8 using dwc_otg
# [  250.488526] usb 1-1.1.3.3: New USB device found, idVendor=1209, idProduct=0d32, bcdDevice= 3.00
# [  250.488542] usb 1-1.1.3.3: New USB device strings: Mfr=1, Product=2, SerialNumber=3
# [  250.488552] usb 1-1.1.3.3: Product: ODrive 3.6 CDC Interface
# [  250.488561] usb 1-1.1.3.3: Manufacturer: ODrive Robotics
# [  250.488571] usb 1-1.1.3.3: SerialNumber: 3352316E3536
# [  250.602567] usb 1-1.1.3.4: new full-speed USB device number 9 using dwc_otg
# [  250.748525] usb 1-1.1.3.4: New USB device found, idVendor=1209, idProduct=0d32, bcdDevice= 3.00
# [  250.748541] usb 1-1.1.3.4: New USB device strings: Mfr=1, Product=2, SerialNumber=3
# [  250.748551] usb 1-1.1.3.4: Product: ODrive 3.6 CDC Interface
# [  250.748561] usb 1-1.1.3.4: Manufacturer: ODrive Robotics
# [  250.748570] usb 1-1.1.3.4: SerialNumber: 335E31483536
