import socket
import serial
import time
serial_port = "/dev/ttyACM0"
# serial_port = "/dev/ttySC5"
# serial_port = "/dev/ttyACM2"

HOST = "0.0.0.0"  # Standard loopback interface address (localhost)
PORT = 65436  # Port to listen on (non-privileged ports are > 1023)

BAUD = 115200

ser = serial.Serial(serial_port, BAUD, timeout=1)


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    conn.setblocking(0)
    with conn:
        print("Connected by {}".format(addr))
        ticktime = time.time()
        while True:
            # time.sleep(0.01)
            bytelist = []
            try:
                while ser.in_waiting:
                    toread = ser.in_waiting
                    bytelist.append(ser.read(toread))
                for item in bytelist:
                    if len(item)>0:
                        conn.sendall(item)
            except OSError:
                print("OSERROR")
                # pass
            try:
                data = conn.recv(1024)
                if data:
                    ser.write(data)
            except BlockingIOError:
                pass
            if time.time() - 4 > ticktime:
                ticktime = time.time()
                print("%%%%")
