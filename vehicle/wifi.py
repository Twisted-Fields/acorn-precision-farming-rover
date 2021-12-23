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
import logging
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
    # '1a:e8:29:a5:ee:7d': 'Farmhouse Exterior HP', -- duplicated with below
    '78:8a:20:2d:9d:0d': 'Farmhouse Kitchen',
    '78:8a:20:2e:9d:0d': 'Farmhouse Kitchen',
    'e0:63:da:15:1e:0f': 'Goat Barn Exterior HP',
    'e2:63:da:16:1e:0f': 'Goat Barn Exterior HP',
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
    '78:8a:20:23:58:9b': 'Mobile Tower Middle Ag',
    '78:8a:20:24:58:9b': 'Mobile Tower Middle Ag'
}


class Wifi:
    def __init__(self, debug):
        self.log_counter = 0
        self.logger = logging.getLogger('main.wifi')
        utils.config_logging(self.logger, debug)
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
            if wlan0_ip and wlan1_ip:
                self.logger.info(
                    "Found IP addresses for wlan0 and wlan1 so turning off wlan0.")
                self.logger.info("wlan1 IP is {}".format(wlan1_ip))
                subprocess.check_call("ifconfig wlan0 down", shell=True)
                self.logger.info("Turned off wlan0.")
            else:
                self.logger.info("Two connected wifi adapters present but both not connected " + "so did not turn off wlan0.")
                self.logger.info("wlan0 IP: {}  | wlan1 IP: {}".format(
                    wlan0_ip, wlan1_ip))
            self.simulate = False
        else:
            self.logger.info("Did not find two wifi adapters so wlan0 will not be disabled.")
            self.simulate = True

    def collect(self):
        if self.simulate:
            return (0, 'simulated-station', 0)

        linkdata = subprocess.check_output("iw dev wlan1 link", shell=True).splitlines()
        # Note: My deepest apologies for not using regex here. - TLA
        signal = str(linkdata[5]).split(':')[1].split(' ')[1]
        station_mac = str(linkdata[0]).split()[2]
        if station_mac in access_points.keys():
            station_name = access_points[station_mac]
        else:
            station_name = station_mac

        tempdata = subprocess.check_output("/opt/vc/bin/vcgencmd measure_temp", shell=True)
        temp = float(str(tempdata).split('=')[1].split('\'')[0])

        self.log_counter += 1
        if self.log_counter >= _LOG_SKIP_COUNT:
            self.logger.info("Wifi RSSI: {} dBm, Station: {}, CPU Temp {}".format(
                signal, station_name, temp))
            self.log_counter = 0
        return (signal, station_name, temp)
