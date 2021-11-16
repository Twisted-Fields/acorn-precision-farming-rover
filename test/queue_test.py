import multiprocessing as mp
import time
import random

random.seed()


def AppendFIFO(list, value, max_values):
    list.append(value)
    while len(list) > max_values:
        list.pop(0)
    return list


def send_slow(queue1, queue2):
    somelist = []
    maxlen = 500
    sequence = 0
    while True:
        queue1.put((time.time(), sequence))
        sequence += 1
        sleeptime = random.random() * 0.01
        time.sleep(sleeptime)
        while queue2.empty() == False:
            val, val2 = queue2.get()
            somelist = AppendFIFO(somelist, val, maxlen)
            del(val2)
            if sequence % 200 == 0:
                print("Queue2: {}".format(val))


def send_fast(queue1, queue2):
    sequence = 0
    while True:
        queue2.put((time.time(), sequence))
        sequence += 1
        sleeptime = random.random() * 0.0001
        time.sleep(sleeptime)
        while queue1.empty() == False:
            val = queue1.get()
            del(val)
            #print("Queue1: {}".format(queue1.get()))


send_queue = mp.Queue()
receive_queue = mp.Queue()
proc = mp.Process(target=send_fast, args=(send_queue, receive_queue,))
proc.start()

send_slow(send_queue, receive_queue)
