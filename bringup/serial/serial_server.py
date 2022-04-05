import socket
import serial
import time
serial_port = "/dev/ttyACM0"

HOST = "0.0.0.0"  # Standard loopback interface address (localhost)
PORT = 65433  # Port to listen on (non-privileged ports are > 1023)


ser = serial.Serial(serial_port, timeout=1)


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    conn.setblocking(0)
    with conn:
        print("Connected by {}".format(addr))
        while True:
            bytelist = []
            while ser.in_waiting:
                toread = ser.in_waiting
                bytelist.append(ser.read(toread))
            for item in bytelist:
                conn.sendall(item)
            try:
                data = conn.recv(1024)
                if data:
                    ser.write(data)
            except BlockingIOError:
                pass
            time.sleep(0.2)
            print("%%%%")
