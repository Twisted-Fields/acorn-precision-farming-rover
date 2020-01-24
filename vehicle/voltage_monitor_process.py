# Simple demo of reading each analog input from the ADS1x15 and printing it to
# the screen.
# Author: Tony DiCola
# License: Public Domain
import time

# Import the ADS1x15 module.
import Adafruit_ADS1x15


# Create an ADS1115 ADC (16-bit) instance.


GAIN = 1

factor1 = 47.89/24954.0
factor2 = 12.20/10047.0
factor3 = 12.2/18763.0




class VoltageSampler():

    def __init__(self, master_conn):
        self.adc = None
        self.master_conn = master_conn

    def read_loop(self):
        self.adc = Adafruit_ADS1x15.ADS1115()
        while True:
            # Read all the ADC channel values in a list.
            values = [0]*4
            for i in range(4):
                # Read the specified ADC channel using the previously set gain value.
                values[i] = self.adc.read_adc(i, gain=GAIN)
                # Note you can also pass in an optional data_rate parameter that controls
                # the ADC conversion time (in samples/second). Each chip has a different
                # set of allowed data rate values, see datasheet Table 9 config register
                # DR bit values.
                #values[i] = adc.read_adc(i, gain=GAIN, data_rate=128)
                # Each value will be a 12 or 16 bit signed integer value depending on the
                # ADC (ADS1015 = 12-bit, ADS1115 = 16-bit).
            low_voltage = values[1] * factor3
            mid_voltage = values[2] * factor2
            high_voltage = values[3] * factor1
            cell1 = low_voltage
            cell2 = mid_voltage - low_voltage
            cell3 = high_voltage - mid_voltage
            total = high_voltage

            self.master_conn.send((cell1, cell2, cell3, total),)

            #print("{:0.2f} V | {:0.2f} V | {:0.2f} V | total: {:0.2f} V".format(cell1, cell2, cell3, total))
            #print('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*values))
            # Pause for half a second.
            time.sleep(0.5)

def sampler_loop(master_conn):
    sampler = VoltageSampler(master_conn)
    sampler.read_loop()

if __name__=="__main__":
    sampler_loop()
