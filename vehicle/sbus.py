#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
The MIT License (MIT)
Copyright (c) 2016 Fabrizio Scimia, fabrizio.scimia@gmail.com
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

Via:
https://github.com/fifteenhex/python-sbus/blob/master/sbus/rx.py
No license provided in that copy, but it originates from MIT-licensed:
https://github.com/Sokrates80/sbus_driver_micropython/blob/master/receiver/sbus_receiver.py

Additional modifications by Taylor Alexander for Twisted Fields
"""

import asyncio
import serial
import serial_asyncio
import numpy as np
from multiprocessing import shared_memory, resource_tracker
import logging
from utils import AppendFIFO, config_logging

CHAN_NBYTES = 144
CHAN_SHAPE = (18,)
CHAN_DTYPE = "int64"

class SBUSReceiver:
    class SBUSFramer(asyncio.Protocol):

        START_BYTE = 0x0f
        END_BYTE = 0x00
        SBUS_FRAME_LEN = 25

        def __init__(self):
            super().__init__()
            self._in_frame = False
            self.transport = None
            self._frame = bytearray()
            self.frames = asyncio.Queue()

        def connection_made(self, transport):
            self.transport = transport

        def data_received(self, data):
            for b in data:
                if self._in_frame:
                    self._frame.append(b)
                    if len(self._frame) == SBUSReceiver.SBUSFramer.SBUS_FRAME_LEN:
                        decoded_frame = SBUSReceiver.SBUSFrame(self._frame)
                        # print(decoded_frame)
                        asyncio.run_coroutine_threadsafe(self.frames.put(decoded_frame), asyncio.get_running_loop())
                        self._in_frame = False
                else:
                    if b == SBUSReceiver.SBUSFramer.START_BYTE:
                        self._in_frame = True
                        self._frame.clear()
                        self._frame.append(b)

        def connection_lost(self, exc):
            asyncio.get_event_loop().stop()

    class SBUSFrame:
        OUT_OF_SYNC_THD = 10
        SBUS_NUM_CHANNELS = 18
        SBUS_SIGNAL_OK = 0
        SBUS_SIGNAL_LOST = 1
        SBUS_SIGNAL_FAILSAFE = 2

        def __init__(self, frame):
            self.sbusChannels = [None] * SBUSReceiver.SBUSFrame.SBUS_NUM_CHANNELS

            channel_sum = int.from_bytes(frame[1:23], byteorder="little")

            for ch in range(0, 16):
                self.sbusChannels[ch] = channel_sum & 0x7ff
                channel_sum = channel_sum >> 11

            # to be tested, No 17 & 18 channel on taranis X8R
            if (frame[23]) & 0x0001:
                self.sbusChannels[16] = 2047
            else:
                self.sbusChannels[16] = 0
            # to be tested, No 17 & 18 channel on taranis X8R
            if ((frame[23]) >> 1) & 0x0001:
                self.sbusChannels[17] = 2047
            else:
                self.sbusChannels[17] = 0

            # Failsafe
            self.failSafeStatus = SBUSReceiver.SBUSFrame.SBUS_SIGNAL_OK
            if (frame[SBUSReceiver.SBUSFramer.SBUS_FRAME_LEN - 2]) & (1 << 2):
                self.failSafeStatus = SBUSReceiver.SBUSFrame.SBUS_SIGNAL_LOST
            if (frame[SBUSReceiver.SBUSFramer.SBUS_FRAME_LEN - 2]) & (1 << 3):
                self.failSafeStatus = SBUSReceiver.SBUSFrame.SBUS_SIGNAL_FAILSAFE

        def get_rx_channels(self):
            """
            Used to retrieve the last SBUS channels values reading
            :return:  an array of 18 unsigned short elements containing 16 standard channel values + 2 digitals (ch 17 and 18)
            """

            return self.sbusChannels

        def get_rx_channel(self, num_ch):
            """
            Used to retrieve the last SBUS channel value reading for a specific channel
            :param: num_ch: the channel which to retrieve the value for
            :return:  a short value containing
            """

            return self.sbusChannels[num_ch]

        def get_failsafe_status(self):
            """
            Used to retrieve the last FAILSAFE status
            :return:  a short value containing
            """

            return self.failSafeStatus

        def __repr__(self):
            return ",".join(str(ch) for ch in self.sbusChannels)

    def __init__(self, logger):
        self._transport = None
        self._protocol = None
        self._shm = None
        self.logger = logger

    @staticmethod
    async def create(port, logger):
        receiver = SBUSReceiver(logger)
        receiver._transport, receiver._protocol = await serial_asyncio.create_serial_connection(
            asyncio.get_running_loop(),
            SBUSReceiver.SBUSFramer,
            port,
            baudrate=100000,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_TWO,
            bytesize=serial.EIGHTBITS)
        return receiver

    async def get_frame(self):
        return await self._protocol.frames.get()

    def setup_sbus_shared_memory(self):
        name = 'sbus_sharedmem'
        try:
            self._shm = shared_memory.SharedMemory(name=name)
            self.logger.info("Connected to existing shared memory {}".format(name))
        except FileNotFoundError:
            self._shm = shared_memory.SharedMemory(
                name=name, create=True, size=CHAN_NBYTES)
            self.logger.info("Created shared memory {}".format(name))
        self.sbus_values = np.ndarray(CHAN_SHAPE,
                                         dtype=CHAN_DTYPE,
                                         buffer=self._shm.buf)
        # self.sbus_values[:] = channels[:]


async def main():
    logger = logging.getLogger('sbus_joystick')
    config_logging(logger, debug=False)
    sbus = await SBUSReceiver.create(port="/dev/ttyAMA1", logger=logger)
    min = 193
    max = 1791
    middle = 992
    range = max - min
    sbus.setup_sbus_shared_memory()
    # channels = np.array(frame.sbusChannels)
    # sbus.sbus_values[:] = channels[:]
    while True:
        frame = await sbus.get_frame()
        # print(frame.sbusChannels)
        channels = np.array(frame.sbusChannels)
        sbus.sbus_values[:] = channels[:]


if __name__ == '__main__':
    asyncio.run(main())
