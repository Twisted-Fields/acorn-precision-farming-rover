import json
import logging
import threading
import fibre
from fibre.utils import Logger
import serial
import time
import sys
import trace
from async_odrive import request, retrieve_result, set_value, call_remote_function


logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
fibre_logger = Logger(verbose=logger.getEffectiveLevel() == logging.DEBUG)

BAUD = 921600


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
    od0 = OdriveManager(path='/dev/ttySC0',
                       serial_number='336B31643536').find_odrive()

    od1 = OdriveManager(path='/dev/ttySC1',
                       serial_number='335B314C3536').find_odrive()

    od2 = OdriveManager(path='/dev/ttySC2',
                       serial_number='3352316E3536').find_odrive()

    od3 = OdriveManager(path='/dev/ttySC3',
                       serial_number='205F3882304E').find_odrive()
    odrives = [od0,od1,od2,od3]

    for idx, od in enumerate(odrives):
        od.vbus_voltage


    # tracer = trace.Trace(
    # ignoredirs=[sys.prefix, sys.exec_prefix],
    # trace=1,
    # count=1,
    # countfuncs=1,
    # countcallers=1,
    # outfile='odrive_test.out',
    # timing=True)
    #
    #
    # def myfunc(odrive):
    #     odrive.axis0.controller.pos_setpoint = 0
    #
    # tracer.runfunc(myfunc, od0)
    #
    # r = tracer.results()
    # r.write_results(show_missing=True, coverdir=".")
    # sys.exit()
    _ADC_PORT_STEERING_POT = 5
    tick_time = time.time()
    count = 0
    maxcount = 0
    while True:
        temp_vals = [[], [], [], []]
        for idx, od in enumerate(odrives):
            call_remote_function(od.axis0.controller, 'move_to_pos', 0)
            time.sleep(0.0005)
            # temp_vals[idx].append(call_remote_function(od, 'get_adc_voltage', _ADC_PORT_STEERING_POT))
            # set_value(od.axis0.controller, 'pos_setpoint', 0.0)
            time.sleep(0.0005)
            set_value(od.axis1.controller, 'vel_setpoint', 0.0)
            # temp_vals[idx].append(request(od, 'vbus_voltage'))
            # time.sleep(0.001)
            # temp_vals[idx].append(request(od.axis0.motor.current_control, 'Ibus'))
            # time.sleep(0.001)
            # temp_vals[idx].append(request(od.axis1.motor.current_control, 'Ibus'))
            # time.sleep(0.001)
            time.sleep(0.0005)
            temp_vals[idx].append(request(od.axis0.encoder, 'pos_estimate'))
            time.sleep(0.0005)
            temp_vals[idx].append(request(od.axis0.encoder, 'vel_estimate'))
            time.sleep(0.0005)
            temp_vals[idx].append(request(od.axis1.encoder, 'pos_estimate'))
            time.sleep(0.0005)
            temp_vals[idx].append(request(od.axis1.encoder, 'vel_estimate'))
            # time.sleep(0.001)
            # temp_vals[idx].append(request(od.axis1, 'error'))
            # time.sleep(0.001)
            # temp_vals[idx].append(request(od.axis1, 'error'))

        count += 1
        for tmp in temp_vals:
            vbus_voltage = retrieve_result(tmp[0])
            # print(vbus_voltage)
            ibus1 = retrieve_result(tmp[1])
            # print(ibus1)
            ibus2 = retrieve_result(tmp[2])
            # print(ibus2)
            pos1 = retrieve_result(tmp[3])
            # pos1 = retrieve_result(tmp[4])

            # # print(pos1)
            # vel1 = retrieve_result(tmp[4])
            # # print(vel1)
            # pos2 = retrieve_result(tmp[5])
            # # print(pos2)
            # vel2 = retrieve_result(tmp[6])
            # # print(vel1)
            # err1 = retrieve_result(tmp[7])
            # # print(pos2)
            # err2 = retrieve_result(tmp[8])
            # print(vel2)
        if time.time() - tick_time > 1.0:
            print(count)
            # print(data)
            tick_time = time.time()
            maxcount = count
            count = 0
        # time.sleep(0.1)
        # print(od._remote_attributes)
