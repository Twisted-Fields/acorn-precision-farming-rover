import serial
ser = serial.Serial('/dev/ttyACM1', 230400, timeout=1)  # open serial port
print(ser.name)         # check which port was really used
ser.reset_input_buffer()
ser.reset_output_buffer()
while True:
    print(ser.readline())     # write a string
ser.close()             # close port
