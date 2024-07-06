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


"""
transmit to motor:
motor 1 setpoint mode, desired setpoint, maximum velocity and torque, motor 2 setpoint mode, desired setpoint, maximum velocity and torque

recieve from motor:
motor 1 position, velocity, current, motor 2 position, velocity, current, sensor values, voltage

can also send:
settings sync packet. request all settings from device

sensor value request packet. request only sensor values.
"""

"""
can bus ID
serial number
motor 1:
    setpoint mode
    setpoint
    max velocity
    max torque
    acceleration
motor 2:
    setpoint mode
    setpoint
    max velocity
    max torque
    acceleration
aux 1 value
aux 2 value
analog 1 value
analog 2 value
thermistor 1 value
thermistor 2 value
input voltage

"""
import struct
from enum import Enum
from crccheck.crc import Crc16


THERMAL_WARNING_LIMIT = 60 # low for testing
THERMAL_SHUTDOWN_LIMIT = 100 # TODO: test
LOGGING_DIVIDER = '#'

ERROR_CODE_MOTOR1_OVERSPEED = 0x01
ERROR_CODE_INVALID_SPEED_COMMAND = 0x02
ERROR_CODE_INCONSISTENT_COMMS = 0x04
ERROR_CODE_INDUCTION_ENCODER_OFFLINE = 0x08

class MessageType(Enum):
    SEND_COMPLETE_SETTINGS = 1
    REQUEST_COMPLETE_SETTINGS = 2
    SEND_BASIC_UPDATE = 3
    REQUEST_SENSORS = 4
    REQUEST_HOMING = 5
    SIMPLEFOC_PASS_THROUGH = 6
    FIRMWARE_UPDATE = 7
    FIRMWARE_UPDATE_CPU2 = 8
    SIMPLE_PING = 9
    LOG_REQUEST = 10
    RAW_BRIDGE_COMMAND = 11
    FIRMWARE_STATUS = 12
    SET_STEERING_HOME = 13
    CLEAR_ERRORS = 14


class Motor:
    def __init__(self):
        self.setpoint_mode = 0
        self.setpoint = 0.0
        self.max_velocity = 0.0
        self.max_torque = 0.0
        self.acceleration_command = 0.0
        self.encoder_counts = 0.0
        self.encoder_velocity = 0.0


class MotorController:

        def __init__(self, id=None):
            self.id = id
            self.motor1 = Motor()
            self.motor2 = Motor()
            self.aux1 = 0
            self.aux2 = 0
            self.adc1 = 0
            self.adc2 = 0
            self.therm_bridge1 = 0
            self.therm_bridge2 = 0
            self.therm_motor1 = 0
            self.therm_motor2 = 0
            self.voltage = 0.0
            self.current = 0.0
            self.motor_voltage = 0.0
            self.wattage = 0.0
            self.steering_homed = False
            self.home_position = 0.0
            self.motion_allowed = False
            self.thermal_warning = False
            self.thermal_shutdown = False
            self.read_error = False
            self.last_packet = None
            self.log_messages = []


        def check_thermals(self):
            for value in (self.therm_motor1, self.therm_motor2, self.therm_bridge1, self.therm_bridge2):
                # Warning will be set to true or false automatically.
                # But thermal shutdown must be manually cleared.
                self.thermal_warning = value > THERMAL_WARNING_LIMIT
                if value > THERMAL_SHUTDOWN_LIMIT:
                    self.thermal_shutdown = True

        def clear_thermal_shutdown(clear_thermal_shutdown=False):
            self.thermal_shutdown = True

        def serialize_motors(self):
            values = bytearray()
            values.extend(MessageType.SEND_COMPLETE_SETTINGS.value.to_bytes(1, byteorder='big'))
            values.extend(self.motor1.setpoint_mode.to_bytes(1, byteorder='big'))
            values.extend(struct.pack("<f", self.motor1.setpoint))
            values.extend(struct.pack("<f", self.motor1.max_velocity))
            values.extend(struct.pack("<f", self.motor1.max_torque))
            values.extend(struct.pack("<f", self.motor1.acceleration_command))

            values.extend(self.motor2.setpoint_mode.to_bytes(1, byteorder='big'))
            values.extend(struct.pack("<f", self.motor2.setpoint))
            values.extend(struct.pack("<f", self.motor2.max_velocity))
            values.extend(struct.pack("<f", self.motor2.max_torque))
            values.extend(struct.pack("<f", self.motor2.acceleration_command))
            return values

        def serialize_basic(self, reply_requested=False):
            """
            Send motor setpoint values. Optionally request back sensor values.
            """
            values = bytearray()
            values.extend(MessageType.SEND_BASIC_UPDATE.value.to_bytes(1, byteorder='big'))
            # Pack the bool in to the MSB to save space.
            if reply_requested:
                values[0] |= 0x40
            values.extend(struct.pack("<f", self.motor1.setpoint))
            values.extend(struct.pack("<f", self.motor2.setpoint))
            return values

        def sensor_request(self):
            """
            Request sensor values.
            """
            values = bytearray()
            values.extend(MessageType.REQUEST_SENSORS.value.to_bytes(1, byteorder='big'))
            return values

        def simple_ping(self):
            """
            Send simple ping request.
            """
            values = bytearray()
            values.extend(MessageType.SIMPLE_PING.value.to_bytes(1, byteorder='big'))
            return values

        def log_request(self):
            """
            Send log request.
            """
            values = bytearray()
            values.extend(MessageType.LOG_REQUEST.value.to_bytes(1, byteorder='big'))
            return values

        def firmware_status_request(self):
            """
            Request firmware status.
            """
            values = bytearray()
            values.extend(MessageType.FIRMWARE_STATUS.value.to_bytes(1, byteorder='big'))
            return values

        def set_steering_home(self, home_position):
            """
            Set steering home.
            """
            values = bytearray()
            values.extend(MessageType.SET_STEERING_HOME.value.to_bytes(1, byteorder='big'))
            values.extend(struct.pack("<f", home_position))
            return values

        def clear_error_codes(self):
            """
            Clear error codes.
            """
            values = bytearray()
            values.extend(MessageType.CLEAR_ERRORS.value.to_bytes(1, byteorder='big'))
            return values

        def simple_FOC_pass_through(self, data):
            values = bytearray()
            values.extend(MessageType.SIMPLEFOC_PASS_THROUGH.value.to_bytes(1, byteorder='big'))
            values.extend(data.encode())
            values.extend(int(10).to_bytes(1, 'big')) # Line Feed
            return values

        def raw_bridge_command(self, bridge_values):
            values = bytearray()
            values.extend(MessageType.RAW_BRIDGE_COMMAND.value.to_bytes(1, byteorder='big'))
            if len(bridge_values) != 6:
                raise ValueError("Must send six bridge values between 0-255")
            for bridge_value in bridge_values:
                if not 0 <= bridge_value <= 255:
                    raise ValueError("Bridge value out of bounds. Must be 0-255")
                values.extend(bridge_value.to_bytes(1,'big'))
            return values

        def decode_sensor_reply(self, packet):
            # TODO these two make up the CRC. Check it!
            # print("==============")
            if packet[0] != self.id:
                print(f"PACKET {packet[0]} NOT FOR THIS CONTROLLER ID! {self.id}:")
                hex_string = "".join(" 0x%02x" % b for b in packet)
                print(hex_string)
                return False
            if packet[1] & 0x7F != MessageType.REQUEST_SENSORS.value:
                print("DECODE SENSOR PACKET RECEIVED WRONG PACKET TYPE")
                return False
            crc = Crc16.calc(packet[:36])
            if crc != packet[37]<<8 | packet[36]:
                print("DECODE SENSOR BAD CRC!")
                return False
            self.adc1 = packet[3]<<8 | packet[2]
            self.adc2 = packet[5]<<8 | packet[4]
            self.aux1 = (packet[6] & (0x1)) > 0
            self.aux2 = (packet[6] & (0x1<<1)) > 0
            self.motion_allowed = (packet[6] & (0x1<<2)) > 0
            self.therm_bridge1 = packet[7]
            self.therm_bridge2 = packet[8]
            self.therm_motor1 = packet[9]
            self.therm_motor2 = packet[10]
            self.voltage = struct.unpack_from("<f", packet, offset=11)[0]
            self.current = struct.unpack_from("<f", packet, offset=15)[0]
            self.motor_voltage = struct.unpack_from("<f", packet, offset=19)[0]
            self.wattage = self.motor_voltage * self.current

            self.motor2.encoder_counts = struct.unpack_from("<i", packet, offset=23)[0]
            self.motor1.steering_angle_radians = struct.unpack_from("<f", packet, offset=27)[0]
            self.motor2.encoder_velocity = struct.unpack_from("<f", packet, offset=31)[0]
            self.error_codes = packet[35]

            self.check_thermals()
            self.last_packet = packet
            return True

        def decode_ping_reply(self, packet, print_result=False, logger=None):
            if packet[0] != self.id:
                if logger==None:
                    print(f"PACKET {packet[0]} NOT FOR THIS CONTROLLER ID! {self.id}")
                else:
                    logger.error(f"PACKET {packet[0]} NOT FOR THIS CONTROLLER ID! {self.id}")
                return (False, )
            if packet[1] & 0x7F != MessageType.SIMPLE_PING.value:
                if logger==None:
                    print(f"DECODE PING REPLY {MessageType.SIMPLE_PING.value} RECEIVED WRONG PACKET TYPE {packet[1]& 0x7F}")
                else:
                    logger.error(f"DECODE PING REPLY {MessageType.SIMPLE_PING.value} RECEIVED WRONG PACKET TYPE {packet[1]& 0x7F}")
                return (False, )
            can_cpu_ack = (packet[2] & 0xF) > 0
            can_motor_ack = (packet[2] & 0xF0) > 0
            if print_result:
                print(f"CAN CPU ack: {can_cpu_ack} | Motor CPU ack: {can_motor_ack}")
            if logger:
                logger.info(f"CAN CPU ack: {can_cpu_ack} | Motor CPU ack: {can_motor_ack}")
            return (can_cpu_ack, can_motor_ack)

        def decode_log_reply(self, packet):
            if packet[0] != self.id:
                print(f"PACKET {packet[0]} NOT FOR THIS CONTROLLER ID! {self.id}")
                return False
            if packet[1] & 0x7F != MessageType.LOG_REQUEST.value:
                print(f"DECODE LOG REPLY {MessageType.LOG_REQUEST.value} RECEIVED WRONG PACKET TYPE {packet[1]& 0x7F}")
                return False
            message = packet[2:].decode("utf-8")
            if len(message) > 0:
                self.log_messages.append(message)
            return message

        def decode_basic_reply(self, packet):
            self.motor1.encoder_counts = struct.unpack_from("<f", packet, offset=0)
            self.motor1.encoder_velocity = struct.unpack_from("<f", packet, offset=4)
            self.motor2.encoder_counts = struct.unpack_from("<f", packet, offset=8)
            self.motor2.encoder_velocity = struct.unpack_from("<f", packet, offset=12)

        def print_sensors(self):
            print(f"ID:{self.id}|ADC1:{self.adc1}|ADC2:{self.adc2}|AUX1:{self.aux1}|AUX2:{self.aux2}|Motion Allowed:{self.motion_allowed}|BTH1:{self.therm_bridge1}|BTH2:{self.therm_bridge2}|MTH1:{self.therm_motor1}|MTH2:{self.therm_motor2}|Voltage:{self.voltage}")


            # values.extend(byte(self.aux1))
            # values.extend(byte(self.aux2))
            # values.extend(struct.pack("<f", self.therm1))
            # values.extend(struct.pack("<f", self.therm1))


if __name__ == "__main__":
    motor = MotorController(id=5)
    print(len(motor.serialize()))
