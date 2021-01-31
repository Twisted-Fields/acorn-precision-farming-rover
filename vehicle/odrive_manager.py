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

import json
import logging
import threading
import fibre
from fibre.utils import Logger

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
fibre_logger = Logger(verbose=logger.getEffectiveLevel() == logging.DEBUG)


class OdriveManager:
    _CONFIG_KEY = 'config'
    _CRC_KEY = 'crc16'

    def __init__(self, path, serial_number, cache_file=None):
        self.channel = None
        self.path = path
        self.serial_number = serial_number
        self.cache_file = cache_file or 'odrive-cache-{serial_number}.json' \
            .format(serial_number=serial_number)

    def setup_odrive(self, config, crc):
        """
        Set up fibre remote object using given configuration.
        """

        json_data = {"name": "fibre_node", "members": config}
        self.channel._interface_definition_crc = crc

        # Just some black magic
        # Nothing to see here
        logger.debug('Making fibre remote object connection.')
        obj = fibre.remote_object.RemoteObject(
            json_data,
            None,
            self.channel,
            fibre_logger,
        )
        obj.__dict__['_json_data'] = json_data['members']
        obj.__dict__['_json_crc'] = crc

        return obj

    def read_config_from_device(self):
        """
        Read fibre connection configuration from odrive device directly, then
        cache it in a JSON file.
        """

        json_bytes = self.channel.remote_endpoint_read_buffer(0)

        try:
            json_string = json_bytes.decode('ascii')
        except UnicodeDecodeError as e:
            logger.warning('Loaded JSON bytes not valid ascii: %s.', str(e))
            return None

        logger.debug('Calculating crc for JSON string.')
        crc = fibre.protocol.calc_crc16(
            fibre.protocol.PROTOCOL_VERSION,
            json_bytes,
        )

        try:
            config = json.loads(json_string)
        except json.decoder.JSONDecodeError as e:
            logger.warning(
                'Loaded JSON string from odrive invalid: %s.',
                str(e),
            )
            return None

        od = self.setup_odrive(config=config, crc=crc)

        logger.info('Caching JSON data in file.')
        with open(self.cache_file, 'w') as f:
            json.dump({self._CONFIG_KEY: config, self._CRC_KEY: crc}, f)

        return od

    def find_odrive(self):
        """
        Find odrive device and set up fibre connection.
        """

        cancellation_token = threading.Event()

        def callback(channel):
            self.channel = channel
            cancellation_token.set()

        fibre.serial_transport.discover_channels(
            path=self.path,
            callback=callback,
            logger=fibre_logger,
            serial_number=self.serial_number,
            cancellation_token=cancellation_token,
            channel_termination_token=None,
        )

        try:
            with open(self.cache_file, 'r') as f:
                data = json.load(f)
                config = data[self._CONFIG_KEY]
                crc = data[self._CRC_KEY]

                # TODO: Handle bad checksum
                od = self.setup_odrive(config=config, crc=crc)
                return od

        except (FileNotFoundError, KeyError, json.decoder.JSONDecodeError) as e:
            logger.warning('Could not read cached odrive configuration (%s). '
                           'Reading from device.', str(e))

            od = self.read_config_from_device()
            return od


if __name__ == '__main__':
    od = OdriveManager(path='/dev/ttySC3', serial_number='336B31643536').find_odrive()
    while True:
        print(od.vbus_voltage)
