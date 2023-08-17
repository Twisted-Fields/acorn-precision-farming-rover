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

from motor_controller import MessageType
import sys
import struct
from crccheck.crc import Crc16

class SimulatedSocket:

    def __init__(self):
        pass

    def set_opts(self, _):
        pass

    def bind(self, interface, address):
        self.interface = interface
        self.address = address.txid

    def send(self, packet):
        self.last_packet = packet

    def recv(self):
        reply = None
        if self.last_packet[0] == MessageType.REQUEST_SENSORS.value:
            reply = [0]*18
            reply[0] = self.address
            reply[1] = self.last_packet[0] & 0x7F
            reply[2] = 0 # adc1 = 512
            reply[3] = 2 # adc1 = 512
            reply[4] = 0 # adc2 = 512
            reply[5] = 2 # adc2 = 512
            reply[6] = 0b0100 # motion allowed = true, aux1 and aux2 = 0
            reply[7] = 25 # therm_bridge1
            reply[8] = 25 # therm_bridge2
            reply[9] = 25 # therm_motor1
            reply[10] = 25 # therm_motor2
            voltage_bytes = struct.pack("<f",40.00)
            reply[11] = voltage_bytes[0]
            reply[12] = voltage_bytes[1]
            reply[13] = voltage_bytes[2]
            reply[14] = voltage_bytes[3]
            reply[15] = 0
            crc = Crc16.calc(reply[:16])
            reply[16] = crc & 0xFF
            reply[17] = (crc>>8) & 0xFF
            reply = bytearray(reply)
            return reply

        if self.last_packet[0] == MessageType.LOG_REQUEST.value:
            reply = [self.address]
            reply.append(self.last_packet[0] & 0x7F)
            reply = bytearray(reply)

        return reply
