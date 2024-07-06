"""
*********************************************************************
                     This file is part of:
                       The Acorn Project
             https://wwww.twistedfields.com/research
*********************************************************************
Copyright (c) 2019-2021 Taylor Alexander, Twisted Fields LLC
Copyright (c) 2021 The Acorn Project contributors (cf. AUTHORS.md).

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*********************************************************************
"""

import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import multiprocessing as mp
import time
import math

R_HIGH = 100
R_LOW = 5.1
RESISTOR_DIVIDER_GAIN = R_LOW/(R_HIGH+R_LOW)


class VoltageSampler():

    def __init__(self, master_conn=None, simulated_hardware=False):
        self.adc = None
        self.master_conn = master_conn
        self.simulated_hardware = simulated_hardware
        if not self.simulated_hardware:
            self.i2c = busio.I2C(board.D1, board.D0)
            self.ads = ADS.ADS1115(self.i2c)
            self.chan = AnalogIn(self.ads, ADS.P0)

    def return_one_sample(self):
        return self.chan.voltage / RESISTOR_DIVIDER_GAIN

    def read_loop(self, stop_signal):
        if self.simulated_hardware:
            while not stop_signal.is_set():
                voltage = 45.12
                if self.master_conn is not None:
                    self.master_conn.send(voltage,)
                time.sleep(1)
        else:
            while not stop_signal.is_set():
                voltage = return_one_sample()
                if self.master_conn is not None:
                    self.master_conn.send(voltage,)
                else:
                    print("reading: {}, voltage: {}".format(chan.value, voltage))
                time.sleep(1)


def sampler_loop(stop_signal, master_conn=None, simulated_hardware=False):
    sampler = VoltageSampler(master_conn, simulated_hardware)
    sampler.read_loop(stop_signal)


if __name__ == "__main__":
    stop_signal = mp.Event()
    sampler_loop(stop_signal)
