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

import netifaces
import subprocess
import time
import utils

_LOG_SKIP_COUNT = 20

access_points = {
    '78:8a:20:2d:9d:f9': 'Barn Interior',
    '78:8a:20:2e:9d:f9': 'Barn Interior',
    '74:83:c2:c4:cd:62': 'Front Gate HP',
    '76:83:c2:c5:cd:62': 'Front Gate HP',
    '74:83:c2:c4:ce:cb': 'Barn Media HP',
    '74:83:c2:c5:ce:cb': 'Barn Media HP',
    'e0:63:da:e7:50:f6': 'CSA Gate HP',
    'e2:63:da:e8:50:f6': 'CSA Gate HP',
    'b4:fb:e4:14:90:75': 'Farmhouse Exterior HP',
    '1a:e8:29:a5:ee:7d': 'Farmhouse Exterior HP',
    '78:8a:20:2d:9d:0d': 'Farmhouse Kitchen',
    '78:8a:20:2e:9d:0d': 'Farmhouse Kitchen',
    '74:ac:b9:34:fd:27': 'Goat Barn Interior',
    '74:ac:b9:35:fd:27': 'Goat Barn Interior',
    '18:e8:29:a4:ed:5f': 'Taylor Office New',
    '1a:e8:29:a5:ed:5f': 'Taylor Office New',
    '78:8a:20:71:cd:e5': 'Top of Hill',
    '78:8a:20:72:cd:e5': 'Top of Hill',
    '18:e8:29:a4:ee:7d': 'Upstairs Bath',
    '1a:e8:29:a5:ee:7d': 'Upstairs Bath',
    'e0:63:da:15:1e:74': 'Big Field HP',
    'e2:63:da:16:1e:74': 'Big Field HP',
    '74:ac:b9:64:36:a1': 'Back of Farmhouse',
    '74:ac:b9:63:36:a1': 'Back of Farmhouse',
    '74:ac:b9:34:fd:08': 'Farm Store',
    '76:ac:b9:35:fd:08': 'Farm Store',
    '74:83:c2:c3:d0:21': 'Mobile Tower Middle Ag',
    '74:83:c2:c4:d0:21': 'Mobile Tower Middle Ag',
    '78:8a:20:23:59:2d': 'Goat Barn Exterior HP',
    '78:8a:20:24:59:2d': 'Goat Barn Exterior HP',
    '78:8a:20:23:58:b0': 'Middle Upper Field HP',
    '78:8a:20:24:58:b0': 'Middle Upper Field HP'
}


def wifi_process(stop_signal, master_conn, logging, debug):
    logger = logging.getLogger('main.wifi')
    utils.config_logging(logger, debug)
    interfaces = netifaces.interfaces()
    if "wlan0" in interfaces and "wlan1" in interfaces:
        wlan0_ip = None
        wlan1_ip = None
        try:
            # Raises if wlan0 has no ipv4 ip.
            wlan0_ip = netifaces.ifaddresses('wlan0')[2][0]['addr']
        except KeyError:
            pass
        try:
            # Raises if wlan1 has no ipv4 ip.
            wlan1_ip = netifaces.ifaddresses('wlan1')[2][0]['addr']
        except KeyError:
            pass
        # if wlan0_ip and wlan1_ip:
        #     logger.info(
        #         "Found IP addresses for wlan0 and wlan1 so turning off wlan0.")
        #     logger.info("wlan1 IP is {}".format(wlan1_ip))
        #     subprocess.check_call("ip link set dev wlan0 down", shell=True)
        #     logger.info("Turned off wlan0.")
        # else:
        #     logger.info("Two connected wifi adapters present but both not connected " +
        #                 "so did not turn off wlan0.")
        #     logger.info("wlan0 IP: {}  | wlan1 IP: {}".format(
        #         wlan0_ip, wlan1_ip))
        log_counter = 0
        while not stop_signal.is_set():
            linkdata = subprocess.check_output("iw dev wlan1 link", shell=True)
            linkdata = linkdata.splitlines()
            tempdata = subprocess.check_output(
                "vcgencmd measure_temp", shell=True)
            try:
                # Note: My deepest apologies for not using regex here. - TLA
                signal = str(linkdata[5]).split(':')[1].split(' ')[1]
                station_mac = str(linkdata[0]).split()[2]
                if station_mac in access_points.keys():
                    station_name = access_points[station_mac]
                else:
                    station_name = station_mac

                temp = float(str(tempdata).split('=')[1].split('\'')[0])
                # print("Temp = {} C".format(temp))

                if master_conn:
                    master_conn.send((signal, station_name, temp))
                    log_counter += 1
                    if log_counter >= _LOG_SKIP_COUNT:
                        logger.info("Wifi RSSI: {} dBm, Station: {}, CPU Temp {}".format(
                            signal, station_name, temp))
                        log_counter = 0
                else:
                    logger.info(station_name)
                    logger.info("Wifi RSSI: {} dBm".format(signal))
            except Exception as e:
                # print("wifi.py prints:")
                # print(e)
                # raise e
                pass
            time.sleep(0.5)
    else:
        logger.info(
            "Did not find two wifi adapters so wlan0 will not be disabled.")
        # while True:
        #     linkdata = subprocess.check_output("iw dev wlp2s0 link", shell=True)
        #     linkdata = linkdata.splitlines()
        #     #stationdata = subprocess.check_output("iw dev wlan1 station dump", shell=True)
        #     #stationdata = stationdata.splitlines()
        #     try:
        #         # Note: My deepest apologies for not using regex here. - TLA
        #         station_mac = str(linkdata[0]).split()[2]
        #         if station_mac in access_points.keys():
        #             station_name = access_points[station_mac]
        #             print(station_name)
        #         else:
        #             #print("Mac {} not found in {}".format(station_mac, access_points.keys()))
        #             station_name = station_mac
        #         #print(str(stationdata[0]).split()[1])
        #         signal = str(linkdata[5]).split(':')[1].split(' ')[1]
        #         #signal = str(stationdata[11]).split(':')[1].split()[0][2:]
        #         if master_conn:
        #             master_conn.send(signal)
        #         else:
        #             #pass
        #             print("Wifi RSSI: {} dBm".format(signal))
        #     except Exception as e:
        #         print(e)
        #         #raise e
        #     time.sleep(0.5)


if __name__ == "__main__":
    wifi_process(None)
