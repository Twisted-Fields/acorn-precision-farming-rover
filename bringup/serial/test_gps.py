import time
import datetime


from serial import Serial
import pyubx2
from pyubx2 import UBXReader, itow2utc
MM_IN_METER = 1000

def flush_serial(stream_center, stream_rear):
    stream_center.reset_input_buffer()
    stream_center.reset_output_buffer()
    stream_rear.reset_input_buffer()
    stream_rear.reset_output_buffer()
stream_center = Serial('/dev/ttySC0', 1000000, timeout=0.10) # Center Receiver
stream_rear = Serial('/dev/ttySC1', 1000000, timeout=0.10) # Rear Receiver
msg = pyubx2.UBXMessage.config_set(layers=1, transaction=0, cfgData=[("CFG_RATE_MEAS", 40)])
stream_center.write(msg.serialize())
stream_rear.write(msg.serialize())
ubr_center = UBXReader(stream_center)
ubr_rear = UBXReader(stream_rear)
flush_serial(stream_center, stream_rear)


while True:
    data_center = None
    data_rear = None
    print("READ GPS")
    try:
        (raw_data_center, data_center) = ubr_center.read()
        (raw_data_rear, data_rear) = ubr_rear.read()
        print(data_center)
        print(data_rear)
        if data_center==None or data_rear==None:
            print(f"ERROR: {data_center} {data_rear}")
        if data_center.identity != "NAV-PVT" or data_rear.identity != "NAV-PVT":
            print(f"ERROR {data_center} {data_rear}")
    except:
        print("ERROR")
        time.sleep(0.1)
