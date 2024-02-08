
import os
import time

if os.uname().machine in ['armv7l','aarch64']:
    import RPi.GPIO as GPIO
    import board
    from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
    from adafruit_extended_bus import ExtendedI2C as I2C
    import adafruit_lis3mdl

clk = 23
data = 22


charmap = {
    '0': 0x3f,
    '1': 0x06,
    '2': 0x5b,
    '3': 0x4f,
    '4': 0x66,
    '5': 0x6d,
    '6': 0x7d,
    '7': 0x07,
    '8': 0x7f,
    '9': 0x6f,
    'A': 0x77,
    'B': 0x7f,
    'b': 0x7C,
    'C': 0x39,
    'c': 0x58,
    'D': 0x3f,
    'd': 0x5E,
    'E': 0x79,
    'F': 0x71,
    'G': 0x7d,
    'H': 0x76,
    'h': 0x74,
    'I': 0x06,
    'J': 0x1f,
    'K': 0x76,
    'L': 0x38,
    'l': 0x06,
    'n': 0x54,
    'O': 0x3f,
    'o': 0x5c,
    'P': 0x73,
    'r': 0x50,
    'S': 0x6d,
    'U': 0x3e,
    'V': 0x3e,
    'Y': 0x66,
    'Z': 0x5b,
    '-': 0x40,
    '_': 0x08,
    ' ': 0x00
}


class DisplaySimulated:
    def __init__(self):
        pass
    def display_voltage(self, _):
        pass


class Display:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(clk, GPIO.OUT)
        GPIO.setup(data, GPIO.OUT)

        GPIO.output(clk, GPIO.HIGH)
        GPIO.output(data, GPIO.HIGH)

        # while True:
        #     self.send_data(0x48, 0x01)
        #     time.sleep(0.1)

        self.send_data(0x48, 0x09)
        time.sleep(0.001)
        self.send_data(0x48, 0x11)
        time.sleep(0.001)

    def set_gpio_enable(self, value):
        if value:
            GPIO.setup(clk, GPIO.OUT)
            GPIO.setup(data, GPIO.OUT)
            GPIO.output(clk, GPIO.HIGH)
            GPIO.output(data, GPIO.HIGH)
        else:
            GPIO.setup(clk, GPIO.I2C)
            GPIO.setup(data, GPIO.I2C)

    def clock_byte(self, value):
        for bitnum in range(7,-1,-1):
            dat_val = value >> bitnum & 0x1
            GPIO.output(clk, GPIO.LOW)
            time.sleep(0.00001)
            GPIO.output(data, dat_val)
            time.sleep(0.00001)
            GPIO.output(clk, GPIO.HIGH)
            time.sleep(0.00001)
        GPIO.output(clk, GPIO.LOW)
        GPIO.output(data, GPIO.LOW)
        GPIO.output(clk, GPIO.HIGH)

    def send_data(self, upper, lower):
        GPIO.output(clk, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(data, GPIO.LOW)
        time.sleep(0.00001)
        self.clock_byte(upper)
        self.clock_byte(lower)
        GPIO.output(data, GPIO.LOW)
        time.sleep(0.00001)
        GPIO.output(data, GPIO.HIGH)

    def display_voltage(self, voltage):
        self.display_string("{:05.2f}".format(voltage))

    def display_time(self):
        # Display the dot every other second so it is more clock-like
        if int(time.time())%2 == 0:
            format_string = "%I.%M"
        else:
            format_string = "%I%M"
        self.display_string(time.strftime(format_string, time.localtime(time.time())))

    def display_string(self, display_data):
        values = [0,0,0,0]
        offset = 0
        if type(display_data) is str:
            for i, c in enumerate(display_data):
                if c in charmap:
                    values[i+offset] = charmap[c]
                else:
                    values[i+offset] = 0
                if c == '.':
                    values[i-1] |= 0x80
                    offset = -1
                if i+offset == 3:
                    break

        self.send_data(0x68, values[0])
        self.send_data(0x6A, values[1])
        self.send_data(0x6C, values[2])
        self.send_data(0x6E, values[3])

if __name__ == '__main__':
    display = Display()
    display.set_gpio_enable(False)
    i2c = I2C(6)
    sox = LSM6DSOX(i2c)
    sensor = adafruit_lis3mdl.LIS3MDL(i2c)
    while True:
        display.set_gpio_enable(True)
        display.display_time()
        display.set_gpio_enable(False)
        mag_x, mag_y, mag_z = sensor.magnetic
        print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2"%(sox.acceleration))
        print("Gyro X:%.2f, Y: %.2f, Z: %.2f radians/s"%(sox.gyro))
        print('X:{0:10.2f}, Y:{1:10.2f}, Z:{2:10.2f} uT'.format(mag_x, mag_y, mag_z))
        print('')
        # time.sleep(1.0)
        time.sleep(0.5)
