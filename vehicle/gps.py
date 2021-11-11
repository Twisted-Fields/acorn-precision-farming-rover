import time
import gps_tools
import rtk_process

class GPS:
    def __init__(self, logger, simulate_hardware=False):
        self.logger = logger
        self.simulate_hardware = simulate_hardware
        self.simulated_sample = gps_tools.GpsSample(37.353039233,
                                                    -122.333725682, 100,
                                                    ("fix", "fix"), 20, 0,
                                                    time.time(), 0.5)
        self.last_gps_sample = None
        self.last_good_gps_sample = None
        self.gps_buffers = ["", ""]
        if self.simulate_hardware:
            self.rtk_socket1 = None
            self.rtk_socket2 = None
        else:
            self.rtk_process.launch_rtk_sub_procs(self.logger)
            self.rtk_socket1, rtk_socket2 = rtk_process.connect_rtk_procs(self.logger)

    def last_sample(self):
        """returns the latest GPS sample, either from the GPS device or simulated"""
        return self.simulated_sample if self.simulate_hardware else self.last_gps_sample

    def last_good_sample(self):
        """returns the latest good GPS sample (non-nil), either from the GPS device or simulated"""
        return self.simulated_sample if self.simulate_hardware else self.last_good_gps_sample

    def update_from_device(self, print_gps, retries):
        """calls the rtk process to get real GPS sample"""
        self.gps_buffers, self.last_gps_sample = (
            rtk_process.rtk_loop_once(
                self.rtk_socket1,
                self.rtk_socket2,
                buffers=self.gps_buffers,
                print_gps=print_gps,
                last_sample=self.last_gps_sample,
                retries=retries,
                logger=self.logger))
        if self.last_gps_sample is not None:
            self.last_good_gps_sample = self.last_gps_sample
        else:
            # Occasional bad samples are fine. A very old sample will
            # get flagged in the final checks.
            self.last_gps_sample = self.last_good_gps_sample

    def update_simulated_sample(self, lat, lon, azimuth_degrees):
        """update simulated sample, only useful if the device is simulated"""
        self.simulated_sample = gps_tools.GpsSample(
            lat, lon, self.simulated_sample.height_m, ("fix", "fix"), 20,
            azimuth_degrees, time.time(), 0.5)

    def is_dual_fix(self):
        """check if the last sample is dual fix."""
        return gps_tools.is_dual_fix(self.last_sample())

    def dump(self):
        """dump the sample (only if simulated)"""
        if self.simulate_hardware:
            self.logger.info(
                f"Lat: {self.simulated_sample.lat:.10f}, "
                f"Lon: {self.simulated_sample.lon:.10f}, "
                f"Azimuth: {self.simulated_sample.azimuth_degrees:.2f}, "
                f"Distance: {2.8:.4f}, Fixes: ({True}, {True}), Period: {0.100:.2f}")
