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
        # print(MessageType._value2member_map_[self.last_packet[0]])
        # print(self.last_packet[0])
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
            # print(reply)
            reply = bytearray(reply)
            return reply

        if self.last_packet[0] == MessageType.LOG_REQUEST.value:
            reply = [self.address]
            reply.append(self.last_packet[0] & 0x7F)
            # reply.append(0)
            reply = bytearray(reply)
            # print(reply[2:].decode)

        return reply




            # self.voltage = struct.unpack_from("<f", packet, offset=11)[0]


        #
        # if packet[0] != self.id:
        #     print(f"PACKET {packet[0]} NOT FOR THIS CONTROLLER ID! {self.id}:")
        #     hex_string = "".join(" 0x%02x" % b for b in packet)
        #     print(hex_string)
        #     return False
        # if packet[1] & 0x7F != MessageType.REQUEST_SENSORS.value:
        #     print("DECODE SENSOR PACKET RECEIVED WRONG PACKET TYPE")
        #     return False
        # crc = Crc16.calc(packet[:16])
        # if crc != packet[17]<<8 | packet[16]:
        #     return False
