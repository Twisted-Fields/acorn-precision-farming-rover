"""
MIT License

Copyright (c) 2016-2018 ODrive Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

This file created from modifications to ODrive Robotics' fibre library by
Taylor Alexander for Twisted Fields.
"""


import sys
import struct
import threading
import traceback
from fibre.utils import Event, wait_any, TimeoutError
from fibre.protocol import *

"""
Read from odrive asynchronously.

If you want to read odrv0.axis0.motor.current_control.Ibus:

attribute_object, ack_event, seq_no = request(odrv0.axis0.motor.current_control, 'Ibus')
if ack_event:
    ibus = retrieve_result(attribute_object, ack_event, seq_no)
    print(ibus)
"""


def remote_endpoint_request_async(self, endpoint_id, input, expect_ack, output_length):
    if input is None:
        input = bytearray(0)
    if (len(input) >= 128):
        raise Exception("packet larger than 127 currently not supported")

    if (expect_ack):
        endpoint_id |= 0x8000

    self._my_lock.acquire()
    try:
        self._outbound_seq_no = ((self._outbound_seq_no + 1) & 0x7fff)
        seq_no = self._outbound_seq_no
    finally:
        self._my_lock.release()
    seq_no |= 0x80  # FIXME: we hardwire one bit of the seq-no to 1 to avoid conflicts with the ascii protocol
    packet = struct.pack('<HHH', seq_no, endpoint_id, output_length)
    packet = packet + input

    crc16 = calc_crc16(CRC16_INIT, packet)
    if (endpoint_id & 0x7fff == 0):
        trailer = PROTOCOL_VERSION
    else:
        trailer = self._interface_definition_crc
    #print("append trailer " + trailer)
    packet = packet + struct.pack('<H', trailer)

    if (expect_ack):
        ack_event = Event()
        self._expected_acks[seq_no] = ack_event
        try:
            attempt = 0
            while (attempt < self._send_attempts):
                self._my_lock.acquire()
                try:
                    self._output.process_packet(packet)
                except ChannelDamagedException:
                    attempt += 1
                    continue  # resend
                except TimeoutError:
                    attempt += 1
                    continue  # resend
                finally:
                    self._my_lock.release()
                return ack_event, seq_no

            raise ChannelBrokenException()  # Too many resend attempts
        except:
            self._expected_acks.pop(seq_no)
            self._responses.pop(seq_no, None)


def remote_endpoint_read_async(self, ack_event, seq_no):
    try:
        attempt = 0
        while (attempt < self._send_attempts):
            # Wait for ACK until the resend timeout is exceeded
            try:
                if wait_any(self._resend_timeout, ack_event, self._channel_broken) != 0:
                    raise ChannelBrokenException()
            except TimeoutError:
                attempt += 1
                continue  # resend
            return self._responses.pop(seq_no)
            # TODO: record channel statistics
        raise ChannelBrokenException()  # Too many resend attempts
    finally:
        self._expected_acks.pop(seq_no)
        self._responses.pop(seq_no, None)


def set_object_value(attribute_object, value):
    buffer = attribute_object._codec.serialize(value)
    # Not needed to use our custom function below because we can set ACK to False.
    attribute_object._parent.__channel__.remote_endpoint_operation(attribute_object._id, buffer, False, 0)


def set_value(sub_property, name, value):
    """ Set a value without waiting for an ACK. """
    attribute_object = sub_property.__getattribute__("_remote_attributes").get(name, None)
    if attribute_object:
        set_object_value(attribute_object, value)
    else:
        raise ValueError(f"Item {name} did not return a valid object.")


def request(sub_property, name):
    attribute_object = sub_property.__getattribute__("_remote_attributes").get(name, None)
    if attribute_object:
        return request_from_object(attribute_object)
    else:
        return None, None, None


def request_from_object(attribute_object):
    ack_event, seq_no = remote_endpoint_request_async(
                        attribute_object._parent.__channel__,
                        attribute_object._id,
                        None, True,
                        attribute_object._codec.get_length())
    return attribute_object, ack_event, seq_no


def call_remote_function(sub_property, name, *args):
    attribute_object = sub_property.__getattribute__(
                                    "_remote_attributes").get(name, None)
    if (len(attribute_object._inputs) != len(args)):
        raise TypeError("expected {} arguments but have {}".format(
                            len(attribute_object._inputs), len(args)))
    for i in range(len(args)):
        set_object_value(attribute_object._inputs[i], args[i])
    # Not needed to use our custom function below because we can set ACK to False.
    attribute_object._parent.__channel__.remote_endpoint_operation(attribute_object._trigger_id, None, False, 0)
    # For functions that return a result, you must take this return value
    # and then pass it to retrieve_result.
    if len(attribute_object._outputs) > 0:
        return request_from_object(attribute_object._outputs[0])


def retrieve_result(values):
    attribute_object = values[0]
    ack_event = values[1]
    seq_no = values[2]
    if not ack_event:
        return None
    buffer = remote_endpoint_read_async(attribute_object._parent.__channel__, ack_event, seq_no)
    return attribute_object._codec.deserialize(buffer)
