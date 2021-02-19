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
This script was originally shared on the ODrive community discussion board by
user marcelrobitaille.

https://discourse.odriverobotics.com/t/fw-0-4-1-why-does-odrive-instance-takes-30s-to-be-created/791/14
"""

import json
import logging
import threading
import random


class FakeOdriveManager:
    _CONFIG_KEY = 'config'
    _CRC_KEY = 'crc16'

    def __init__(self, path, serial_number, cache_file=None):
        self.channel = None
        self.path = path
        self.serial_number = serial_number


    def find_odrive(self):
        """
        Find odrive device and set up fibre connection.
        """
        return FakeOdrive()


class FakeOdrive:
    def __init__(self):
        self.vbus_voltage = 17.20576286315918
        self.serial_number = 0x20643596524B
        self.hw_version_major = 3
        self.hw_version_minor = 6
        self.hw_version_variant = 56
        self.fw_version_major = 0
        self.fw_version_minor = 0
        self.fw_version_revision = 0
        self.fw_version_unreleased = 1
        self.user_config_loaded = True
        self.brake_resistor_armed = True
        class Stats:
            def __init__(self):
                self.uptime = 216876
                self.min_heap_space = 4216
                self.min_stack_space_axis0 = 7820
                self.min_stack_space_axis1 = 7820
                self.min_stack_space_comms = 12956
                self.min_stack_space_usb = 3276
                self.min_stack_space_uart = 3932
                self.min_stack_space_usb_irq = 1796
                self.min_stack_space_startup = 612
        self.system_stats = Stats()
        class Config:
            def __init__(self):
                self.brake_resistance = 2.0
                self.enable_uart = True
                self.enable_i2c_instead_of_can = False
                self.enable_ascii_protocol_on_usb = False
                self.dc_bus_undervoltage_trip_level = 8.0
                self.dc_bus_overvoltage_trip_level = 59.92000198364258
        self.config = Config()
        print(self.config.dc_bus_overvoltage_trip_level)

        class Axis:
            def __init__(self):
                self.error = 0x0000
                self.step_dir_active = False
                self.current_state = 1
                self.requested_state = 0
                self.loop_counter = 1722222
                self.lockin_state = 0
                class AxisConfig():
                    def __init__(self):
                        self.startup_motor_calibration = False
                        self.startup_encoder_index_search = False
                        self.startup_encoder_offset_calibration = False
                        self.startup_closed_loop_control = False
                        self.startup_sensorless_control = False
                        self.enable_step_dir = False
                        self.counts_per_step = 2.0
                        self.watchdog_timeout = 0.0
                        self.step_gpio_pin = 1
                        self.dir_gpio_pin = 2
                self.config = AxisConfig()
                class Motor():
                    def __init__(self):
                        self.error = 0x0000
                        self.armed_state = 0
                        self.is_calibrated = True
                        self.current_meas_phB = 0.23636668920516968
                        self.current_meas_phC = 0.033308207988739014
                        self.DC_calib_phB = -0.49849551916122437
                        self.DC_calib_phC = -0.758490800857544
                        self.phase_current_rev_gain = 0.012500000186264515
                        self.thermal_current_lim = 10.0
                        class CurrentControl():
                            def __init__(self):
                                self.p_gain = 9999900.0
                                self.i_gain = 0.7417920827865601
                                self.v_current_control_integral_d = 0.0
                                self.v_current_control_integral_q = 0.0
                                self.Ibus = 0.0
                                self.final_v_alpha = 0.0
                                self.final_v_beta = 0.0
                                self.Iq_setpoint = 0.0
                                self.Iq_measured = 0.0
                                self.Id_measured = 0.0
                                self.I_measured_report_filter_k = 1.0
                                self.max_allowed_current = 30.375
                                self.overcurrent_trip_level = 33.75
                        self.current_control = CurrentControl()

                        class GateDriver():
                            def __init__(self):
                                self.drv_fault = 0
                        self.gate_driver = GateDriver()

                        class TimingLog():
                            def __init__(self):
                                self.TIMING_LOG_GENERAL = 0
                                self.TIMING_LOG_ADC_CB_I = 2706
                                self.TIMING_LOG_ADC_CB_DC = 12850
                                self.TIMING_LOG_MEAS_R = 0
                                self.TIMING_LOG_MEAS_L = 0
                                self.TIMING_LOG_ENC_CALIB = 0
                                self.TIMING_LOG_IDX_SEARCH = 0
                                self.TIMING_LOG_FOC_VOLTAGE = 0
                                self.TIMING_LOG_FOC_CURRENT = 0

                        self.timing_log = TimingLog()

                        class MotorConfig():
                            def __init__(self):
                                self.pre_calibrated = True
                                self.pole_pairs = 7
                                self.calibration_current = 10.0
                                self.resistance_calib_max_voltage = 4.0
                                self.phase_inductance = 99999.0
                                self.phase_resistance = 0.0074179209768772125
                                self.direction = 1
                                self.motor_type = 4
                                self.current_lim = 35.0
                                self.inverter_temp_limit_lower = 100.0
                                self.inverter_temp_limit_upper = 120.0
                                self.requested_current_range = 25.0
                                self.current_control_bandwidth = 100.0

                        self.config = MotorConfig()
                self.motor = Motor()

                class Controller():
                    def __init__(self):
                        self.error = 0x0000
                        self.pos_setpoint = 0.0
                        self.vel_setpoint = 0.0
                        self.vel_integrator_current = 0.0
                        self.current_setpoint = 0.0
                        self.vel_ramp_target = 0.0
                        self.vel_ramp_enable = False

                        class ControllerConfig():
                            def __init__(self):
                                self.control_mode = 3
                                self.pos_gain = -10.0
                                self.vel_gain = 0.004999999888241291
                                self.vel_integrator_gain = 0.0010000000474974513
                                self.vel_limit = 20000.0
                                self.vel_limit_tolerance = 1.2000000476837158
                                self.vel_ramp_rate = 20.0
                                self.setpoints_in_cpr = False
                        self.config = ControllerConfig()
                    def move_to_pos(self, _):
                        return

                self.controller = Controller()

                class Encoder():
                    def __init__(self):
                        self.error = 0x0000
                        self.is_ready = False
                        self.index_found = False
                        self.shadow_count = 0
                        self.count_in_cpr = 0
                        self.interpolation = 0.5
                        self.phase = 0.010995574295520782
                        self.pos_estimate = 0.0
                        self.pos_cpr = 0.0
                        self.hall_state = 7
                        self.vel_estimate = 0.0
                        self.calib_scan_response = 0.0

                        class EncoderConfig():
                            def __init__(self):
                                self.mode = 0
                                self.use_index = False
                                self.find_idx_on_lockin_only = False
                                self.pre_calibrated = False
                                self.zero_count_on_find_idx = True
                                self.cpr = 2000
                                self.offset = 0
                                self.offset_float = 0.0
                                self.enable_phase_interpolation = True
                                self.bandwidth = 1000.0
                                self.calib_range = 0.019999999552965164
                                self.calib_scan_distance = 50.26548385620117
                                self.calib_scan_omega = 12.566370964050293
                                self.idx_search_unidirectional = False
                                self.ignore_illegal_hall_state = False
                        self.config = EncoderConfig()
                self.encoder = Encoder()

                class SensorlessEstimator():
                    def __init__(self):
                        self.error = 0x0000
                        self.phase = None
                        self.pll_pos = None
                        self.vel_estimate = None

                        class SensorlessConfig():
                            def __init__(self):
                                self.observer_gain = 1000.0
                                self.pll_bandwidth = 1000.0
                                self.pm_flux_linkage = 0.0015800000401213765
                        self.config = SensorlessConfig()

                self.sensorless_estimator = SensorlessEstimator()

                class TrapTraj():
                    def __init__(self):
                        class TrapTrajConfig():
                            def __init__(self):
                                self.vel_limit = 12000.0
                                self.accel_limit = 3000.0
                                self.decel_limit = 3000.0
                                self.A_per_css = 0.0
                        self.config = TrapTrajConfig()
                self.trap_traj = TrapTraj()

        self.axis0 = Axis()
        self.axis1 = Axis()

        class Can():
            def __init__(self):
                self.node_id = 0
                self.TxMailboxCompleteCallbackCnt = 0
                self.TxMailboxAbortCallbackCnt = 0
                self.received_msg_cnt = 0
                self.received_ack = 0
                self.unexpected_errors = 0
                self.unhandled_messages = 0
        self.can = Can()
        self.test_property = 0
        self._remote_attributes = {}
    def get_adc_voltage(self, _):
        return 3.3/2.0 + 0.5 * random.choice((-1.0, 1.0))

        # self.test_function(delta: int)
        # self.get_oscilloscope_val(index: int)
        # self.get_adc_voltage(gpio: int)
        # self.save_configuration()
        # self.erase_configuration()
        # self.reboot()
        # self.enter_dfu_mode()


if __name__ == '__main__':
    od = FakeOdriveManager(path='/dev/ttySC3', serial_number='336B31643536').find_odrive()
    while True:
        print(od.vbus_voltage)
        time.sleep()
