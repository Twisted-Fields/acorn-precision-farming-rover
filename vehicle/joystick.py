from evdev import InputDevice, list_devices, categorize, ecodes
import sbus
import asyncio
import multiprocessing as mp
import time
from multiprocessing import shared_memory
import os
import numpy as np

_JOYSTICK_MIN = 0.02
_SBUS_MID_VAL = 1024
_SBUS_MIN = 193
_SBUS_MAX = 1791
_SBUS_RANGE = _SBUS_MAX - _SBUS_MIN

_SBUS_STEER_CH = 3
_SBUS_THROTTLE_CH = 1
_SBUS_STRAFE_CH = 0
_SBUS_SWITCH_R_CH = 7
_SBUS_SWITCH_L_CH = 6
_SBUS_BUTTON = 5
_NO_CHANGE_LIMIT = 20  # If Joystick input hasn't change after a while, zero values.

_USE_JOYSTICK_IN_SIMULATION = False

STICK_ACTIVE_MIN = 0.1



def sbus_proc():
    asyncio.run(sbus.main())

class JoystickSBUS():
    def __init__(self, logger, simulated=False):
        self.logger=logger
        self.simulated = simulated
        self.steer, self.throttle, self.strafe = 0.0, 0.0, 0.0
        self.no_change_counter = 0
        self.max_no_change_counter = 0
        self.autonomy_allowed = True
        self.autonomy_requested = False
        self.strafe_allowed = False
        self.initial_values = []
        self.initial_value_has_changed = False
        self.safety_check_passed_switches_down = False
        self.safety_check_passed_switches_up = False

    def __str__(self):
        return "Steer : {:.2f}, Throttle: {:.2f}, Strafe: {:.2f}".format(self.steer, self.throttle, self.strafe)

    def clear_steer(self):
        self.steer = 0.0

    def update_value(self):
        ch = self.sbus_channels[:]

        # Switch down = True.
        switch_left_down = ch[_SBUS_SWITCH_L_CH] > _SBUS_MID_VAL
        switch_right_down = ch[_SBUS_SWITCH_R_CH] > _SBUS_MID_VAL

        steer = (ch[_SBUS_STEER_CH] - _SBUS_MID_VAL)/(_SBUS_RANGE/2)
        throttle = (ch[_SBUS_THROTTLE_CH] - _SBUS_MID_VAL)/(_SBUS_RANGE/2)
        strafe_stick = (ch[_SBUS_STRAFE_CH] - _SBUS_MID_VAL)/(_SBUS_RANGE/2)

        if len(self.initial_values) == 0:
            self.initial_values =list(ch)
        if not self.initial_value_has_changed:
            for cmp1, cmp2 in zip(ch, self.initial_values):
                if cmp1 != cmp2:
                    # TODO: this check is potentially over restrictive.
                    if not self.check_active(steer, throttle, strafe_stick):
                        self.initial_value_has_changed = True
        if not self.initial_value_has_changed:
            return

        """
        At controller boot, controller requires switches to be up. Once acorn
        program initializes joystick control, the user must flip both switches
        down while joysticks are neutral, then flip them back up. Only then
        will joystick control be active. These safety checks reduce the chances
        that a malfunctioning joystick will be given control of the robot.
        """
        if not self.safety_check_passed_switches_down:
            if switch_left_down and switch_right_down:
                if not self.check_active(steer, throttle, strafe_stick):
                    self.safety_check_passed_switches_down = True
                else:
                    return
            else:
                return

        if not self.safety_check_passed_switches_up:
            if not switch_left_down and not switch_right_down:
                if not self.check_active(steer, throttle, strafe_stick):
                    self.safety_check_passed_switches_up = True
                else:
                    return
            else:
                return

        self.strafe_allowed = switch_right_down
        if self.strafe_allowed:
            strafe = strafe_stick
        else:
            strafe = 0.0

        self.autonomy_allowed = switch_left_down
        if ch[_SBUS_BUTTON] > _SBUS_MID_VAL:
            self.autonomy_requested = True

        if self.activated() and steer == self.steer and throttle == self.throttle and strafe == self.strafe:
            self.no_change_counter += 1
            # print("NO CHANGE {} ###############".format(self.no_change_counter))
        else:
            self.steer = steer
            self.throttle = throttle
            self.strafe = strafe
            self.no_change_counter = 0
        if self.no_change_counter > _NO_CHANGE_LIMIT:
            # print("NO CHANGE LIMIT")
            self.steer, self.throttle, self.strafe = 0.0, 0.0, 0.0
        if self.no_change_counter > self.max_no_change_counter:
            self.max_no_change_counter = self.no_change_counter
        # if self.activated():
            # print("MAX NO CHANGE {}".format(self.max_no_change_counter))

    def connect(self):
        joy_proc = mp.Process(target=sbus_proc, daemon=True)
        joy_proc.start()
        self.setup_sbus_shared_memory()
        self.initial_connect_time = time.time()

    def activated(self):
        return self.check_active(steer=self.steer, throttle=self.throttle, strafe=self.strafe)

    def check_active(self, steer, throttle, strafe):
        return abs(steer) > STICK_ACTIVE_MIN or abs(throttle) > STICK_ACTIVE_MIN or abs(strafe) > STICK_ACTIVE_MIN

    def setup_sbus_shared_memory(self):
        # set up shared memory for GUI four wheel steeing debugger (sim only).
        name = 'sbus_sharedmem'
        while not os.path.exists('/dev/shm/' + name):
            time.sleep(0.5)
        self._shm = shared_memory.SharedMemory(name=name)
        self.logger.info("Connected to existing shared memory {}".format(name))
        self.sbus_channels = np.ndarray(sbus.CHAN_SHAPE, dtype=sbus.CHAN_DTYPE, buffer=self._shm.buf)
        # print("ALJHDFKASJHDGLFJDHG")

class Joystick():

    def __init__(self, simulated=False):
        self.joy = None
        self.simulated = simulated
        self.steer, self.throttle, self.strafe = 0.0, 0.0, 0.0
        self.autonomy_allowed = True
        self.autonomy_requested = False
        self.strafe_allowed = False
        self.initial_values = []
        self.initial_value_has_changed = False

    def __expr__(self):
        s = "Joystick"
        if self.simulated:
            s = "Simulated " + s
        return f"<{s}: steer {self.steer}, throttle {self.throttle}, strafe {self.strafe}>"

    def connect(self):
        devices = [InputDevice(fn) for fn in list_devices()]
        for dev in devices:
            if "Microsoft" in dev.name:
                self.joy = dev
                return
            if "Logitech" in dev.name:
                self.joy = dev
                return

    def get_joystick_values(self, st_old, th_old, stf_old):
        """
        get the latest joystick events and map them to steer, throttle, or strafe, until all pending events are drained.
        when there's no event mapping to a value, the old value is returned.
        """
        steer = st_old
        throttle = th_old
        strafe = stf_old
        while self.joy:
            event = None
            try:
                event = self.joy.read_one()
            except Exception as e:
                self.logger.error("Joystick read exception: {}".format(e))
            if event is None:
                break
            if event.type != ecodes.EV_ABS:
                continue
            absevent = categorize(event)
            code = ecodes.bytype[absevent.event.type][absevent.event.code]
            if code == 'ABS_RX':
                steer = absevent.event.value / 32768.0
            elif code == 'ABS_Y':
                throttle = -absevent.event.value / 32768.0
            elif code == 'ABS_X':
                strafe = absevent.event.value / 32768.0

        return steer, throttle, strafe

    def update_value(self):
        if self.simulated and not _USE_JOYSTICK_IN_SIMULATION:
            self.steer, self.throttle, self.strafe = 0.0, 0.0, 0.0
        else:
            self.steer, self.throttle, self.strafe = self.get_joystick_values(
                self.steer, self.throttle, self.strafe)
            if abs(self.throttle) < _JOYSTICK_MIN:
                self.throttle = 0.0
            if abs(self.steer) < _JOYSTICK_MIN:
                self.steer = 0.0

    def activated(self):
        return self.check_active(steer=self.steer, throttle=self.throttle, strafe=self.strafe)

    def check_active(self, steer, throttle, strafe):
        return abs(steer) > STICK_ACTIVE_MIN or abs(throttle) > STICK_ACTIVE_MIN or abs(strafe) > STICK_ACTIVE_MIN

    def clear_steer(self):
        self.steer = 0.0
