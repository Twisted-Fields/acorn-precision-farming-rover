from evdev import InputDevice, list_devices, categorize, ecodes

_JOYSTICK_MIN = 0.02


class Joystick():

    def __init__(self, simulated=False):
        self.joy = None
        self.simulated = simulated
        self.steer, self.throttle, self.strafe = 0.0, 0.0, 0.0

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
        if self.simulated and False:  # ???
            self.steer, self.throttle, self.strafe = 0.0, 0.0, 0.0
        else:
            self.steer, self.throttle, self.strafe = self.get_joystick_values(
                self.steer, self.throttle, self.strafe)
            if abs(self.throttle) < _JOYSTICK_MIN:
                self.throttle = 0.0
            if abs(self.steer) < _JOYSTICK_MIN:
                self.steer = 0.0

    def activated(self):
        return abs(self.steer) > 0.1 or abs(self.throttle) > 0.1 or abs(self.strafe) > 0.1

    def clear_steer(self):
        self.steer = 0.0
