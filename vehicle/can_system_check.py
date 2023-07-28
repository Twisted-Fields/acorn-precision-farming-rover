import isotp
import motor_controller as mot
import time
import random
import traceback
import sys
import math
import RPi.GPIO as GPIO
import corner_actuator_can as corner_actuator
from multiprocessing import Process
import subprocess

random.seed()


loopval = 0.0

# s = isotp.socket(isotp.socket.flags.WAIT_TX_DONE)


GPIO.setmode(GPIO.BCM)
GPIO.setup(corner_actuator.ESTOP_PIN, GPIO.OUT, initial=GPIO.LOW)

# def e_stop_square_wave(GPIO):
#     delay = 0.01
#     while True:
#         # ESTOP approx 100Hz square wave.
#         time.sleep(delay)
#         GPIO.output(corner_actuator.ESTOP_PIN, GPIO.LOW)
#         time.sleep(delay)
#         GPIO.output(corner_actuator.ESTOP_PIN, GPIO.HIGH)
#
# square_wave = Process(target=e_stop_square_wave, args=(GPIO,))
# square_wave.start()


def request_reply(socket, request_packet, address, error_limit=10):
    send_ok = False
    errors = 0
    while not send_ok:
        try:
            # hex_string = "".join(" 0x%02x" % b for b in request_packet)
            # print(f"Send: {address} {hex_string}")
            socket.send(request_packet)
            send_ok = True
            # print("Send okay.")
        except KeyboardInterrupt as e:
            raise e
        except Exception as e:
            # raise e
            print(f"Send Error, controller ID {controller.id}")
            time.sleep(0.004)
            errors += 1
            if errors > error_limit:
                return False
    errors = 0
    data = None
    while not data:
        try:
            data=socket.recv()
            # data2=socket.recv()
            hex_string = "".join(" 0x%02x" % b for b in data)
            # print(f"Recieve returned {hex_string}")
            if not data:
                print("Recieve error.")
                errors += 1
                if errors > error_limit:
                    return False
            if data[0] != address:
                # print("Skipping wrong packet")
                data = None
        except KeyboardInterrupt as e:
            raise e
        except Exception:
            print("Recieve error.")
            return False
    return data

def ping_request(socket, controller):
    reply = request_reply(socket, controller.simple_ping(), controller.id)
    if reply:
        return controller.decode_ping_reply(reply, print_result=True)
    else:
        return (False, False)


def sensor_request(socket, controller):
    reply = request_reply(socket, controller.sensor_request(), controller.id)
    if reply:
        return controller.decode_sensor_reply(reply)
        # return True
    else:
        return False


def reset_can_port(interface):
    subprocess.run(["ifconfig", interface, "down"])
    time.sleep(0.05)
    subprocess.run(["ifconfig", interface, "up"])
    time.sleep(0.05)


def init_controller(interface, address):
    reset_can_port(interface)
    s = isotp.socket(timeout=0.1)
    s.set_opts(isotp.socket.flags.WAIT_TX_DONE)
    s.bind(interface, isotp.Address(rxid=0x1, txid=address))
    print(s.address)
    controller = mot.MotorController(id=address)
    return s, controller

interface = "can1"

# import socket

addresses_found = []

for address in range(5, 13):
    s, controller = init_controller(interface, address)
    result = ping_request(s, controller)
    if all(result):
        print(f"ADDRESS {address} found")
        addresses_found.append(address)

    else:
        print(f"ADDRESS {address} NOT found")
    s.close()
    reset_can_port(interface)
    time.sleep(0.05)

if not len(addresses_found) > 0:
    print("No devices found!")
    sys.exit()


sockets = []
controllers = []
for address in addresses_found:
    s, controller = init_controller(interface, address)
    sockets.append(s)
    controllers.append(controller)

for s, controller in zip(sockets, controllers):
    if sensor_request(s, controller):
        controller.print_sensors()

# for s in sockets:
#     print(s.address)
#     s.close()
# import sys
# sys.exit()

count = 0
ticktime = time.time()

try:
    while True:
        count += 1
        for s, controller in zip(sockets, controllers):
            time.sleep(0.0002)
            if sensor_request(s, controller):
                controller.read_error = False
                if controller.thermal_warning:
                    print("THERMAL WARNING!!!")
                if controller.thermal_shutdown:
                    print("THERMAL SHUTDOWN!!!")
            else:
                controller.read_error = True
                print("READ_ERROR")
        if time.time() - ticktime > 0.1:
            print(f"Rate: {count*10} hz ", end='')
            for controller in controllers:
                if controller.read_error:
                    print(f"|  ID:{controller.id} --------------- |", end='')
                else:
                    print(f"| ID:{controller.id}, {controller.voltage:.2f}, {controller.therm_bridge1}, {controller.therm_bridge2} |", end='')
            print()
            ticktime = time.time()
            count = 0

except Exception as e:
    for s in sockets:
        s.close()
    raise e
