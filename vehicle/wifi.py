import netifaces
import subprocess
import time


def wifi_process(master_conn):
    interfaces = netifaces.interfaces()
    if "wlan0" in interfaces and "wlan1" in interfaces:
        wlan0_ip = None
        wlan1_ip = None
        try:
            wlan0_ip = netifaces.ifaddresses('wlan0')[2][0]['addr']  # Raises if wlan0 has no ipv4 ip.
        except KeyError:
            pass
        try:
            wlan1_ip = netifaces.ifaddresses('wlan1')[2][0]['addr']  # Raises if wlan1 has no ipv4 ip.
        except KeyError:
            pass
        if wlan0_ip and wlan1_ip:
            print("Found IP addresses for wlan0 and wlan1 so turning off wlan0.")
            print("wlan1 IP is {}".format(wlan1_ip))
            subprocess.check_call("sudo ifconfig wlan0 down", shell=True)
            print("Turned off wlan0.")
        else:
            print("Two connected wifi adapters present but both not connected " +
                  "so did not turn off wlan0.")
            print("wlan0 IP: {}  | wlan1 IP: {}".format(wlan0_ip, wlan1_ip))
        while True:
            linkdata = subprocess.check_output("iw dev wlan1 link", shell=True)
            linkdata = linkdata.splitlines()
            try:
                signal = str(linkdata[5]).split(':')[1].split(' ')[1]
                if master_conn:
                    master_conn.send(signal)
                else:
                    print("Wifi RSSI: {} dBm".format(signal))
            except Exception as e:
                raise e
            time.sleep(0.5)
    else:
        print("Did not find two wifi adapters so wlan0 will not be disabled.")

if __name__=="__main__":
    wifi_process(None)
