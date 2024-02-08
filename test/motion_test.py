import time
import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_lis3mdl
# import busio

# SDA = 22
# SCL = 23
#
# print(board.I2C)
# print(board.SDA)
i2c = I2C(6)
# i2c = board.I2C(6)  # uses board.SCL and board.SDA
sox = LSM6DSOX(i2c)
sensor = adafruit_lis3mdl.LIS3MDL(i2c)

while True:
    mag_x, mag_y, mag_z = sensor.magnetic
    print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2"%(sox.acceleration))
    print("Gyro X:%.2f, Y: %.2f, Z: %.2f radians/s"%(sox.gyro))
    print('X:{0:10.2f}, Y:{1:10.2f}, Z:{2:10.2f} uT'.format(mag_x, mag_y, mag_z))
    print('')
    time.sleep(1.0)
