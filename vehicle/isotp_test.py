import isotp
import time
import traceback

#s = isotp.socket(isotp.socket.flags.WAIT_TX_DONE)
s = isotp.socket(timeout=2.0)
s.set_opts(isotp.socket.flags.WAIT_TX_DONE)
# s.bind("can1", isotp.Address(rxid=0x123, txid=0x456))

s.bind("can1", isotp.Address(rxid=0x1, txid=0x8))

datachunk = bytearray(99999)

ticktime=time.time()
while True:
    try:
        s.send(datachunk)
    except Exception as e:
        traceback.print_exc()
        print(time.time()-ticktime)
        ticktime=time.time()
    # If this sleep is 0.003 or less, I get errors.
    time.sleep(2)
