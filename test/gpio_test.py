from pca9536 import PCA9536
from smbus2 import SMBus
from pca9536 import PinMode
import time

with SMBus(6) as bus:
    device = PCA9536(bus=bus)
    pin = device[0]
    pin.mode = PinMode.output  # or use "output"
    while True:
        pin.write(True)
        time.sleep(1)
        pin.write(False)
        time.sleep(1)
