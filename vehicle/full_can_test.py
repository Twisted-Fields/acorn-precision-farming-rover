import isotp
import motor_controller as mot
import time
import random
import traceback
import sys
import math

random.seed()

controller1 = mot.MotorController(id=5)
controller2 = mot.MotorController(id=6)
controller3 = mot.MotorController(id=7)
controller4 = mot.MotorController(id=8)

loopval = 0.0

s1 = isotp.socket()
s1.set_opts(isotp.socket.flags.WAIT_TX_DONE)
s1.bind("can1", isotp.Address(rxid=0x1, txid=0x5))


s2 = isotp.socket()
s2.set_opts(isotp.socket.flags.WAIT_TX_DONE)
s2.bind("can1", isotp.Address(rxid=0x1, txid=0x6))



s3 = isotp.socket()
s3.set_opts(isotp.socket.flags.WAIT_TX_DONE)
s3.bind("can1", isotp.Address(rxid=0x1, txid=0x7))



s4 = isotp.socket()
s4.set_opts(isotp.socket.flags.WAIT_TX_DONE)
s4.bind("can1", isotp.Address(rxid=0x1, txid=0x8))


ticktime = time.time()
tickcount = 0


while True:
    # time.sleep(0.001)
    tickcount+=1

    send_ok = False
    while not send_ok:
        try:
            s1.send(controller1.sensor_request())
            send_ok = True
        except Exception as e:
            raise e
            print("Send Error")
            time.sleep(0.004)
    data = s1.recv()
    while data is None:
        data=s1.recv()
    controller1.decode_sensor_reply(data)



    send_ok = False
    while not send_ok:
        try:
            s2.send(controller2.sensor_request())
            send_ok = True
        except Exception as e:
            raise e
            print("Send Error")
            time.sleep(0.004)
    data = s1.recv()
    while data is None:
        data=s1.recv()
    controller2.decode_sensor_reply(data)

    send_ok = False
    while not send_ok:
        try:
            s3.send(controller3.sensor_request())
            send_ok = True
        except Exception as e:
            raise e
            print("Send Error")
            time.sleep(0.004)
    data = s1.recv()
    while data is None:
        data=s1.recv()
    controller3.decode_sensor_reply(data)

    send_ok = False
    while not send_ok:
        try:
            s4.send(controller4.sensor_request())
            send_ok = True
        except Exception as e:
            raise e
            print("Send Error")
            time.sleep(0.004)
    data = s1.recv()
    while data is None:
        data=s1.recv()
    controller4.decode_sensor_reply(data)

    print_duration = 0.05
    if time.time()-ticktime > print_duration:
        controller1.print_sensors()
        controller2.print_sensors()
        controller3.print_sensors()
        controller4.print_sensors()
        print(f"{tickcount*(1.0/print_duration)} ticks per second")
        ticktime = time.time()
        tickcount = 0
