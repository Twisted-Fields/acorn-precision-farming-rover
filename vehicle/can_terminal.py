import isotp
import motor_controller as mot
import time
import random
import traceback

random.seed()

controller = mot.MotorController(id=5)

loopval = 0.0

# s = isotp.socket(isotp.socket.flags.WAIT_TX_DONE)
s = isotp.socket()
s.set_opts(isotp.socket.flags.WAIT_TX_DONE)
s.bind("can1", isotp.Address(rxid=0x123, txid=0x456))

ticktime = time.time()
tickcount = 0


while True:
    data = input('--> ')
    if 'Exit' == data:
        break
    print(f"Got data {data}")
    s.send(controller.simple_FOC_pass_through(data))
    reply = s.recv()
    while reply is None:
        reply=s.recv()
    print(reply.decode())

    # print(f'Processing Message from input() *****{data}*****')
