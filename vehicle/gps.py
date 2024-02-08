import time
import gps_tools
import datetime


_SUPPORT_NEW_GPS = True
_SUPPORT_OLD_GPS = True

if _SUPPORT_NEW_GPS:
    from serial import Serial
    import pyubx2
    from pyubx2 import UBXReader, itow2utc
    MM_IN_METER = 1000
if _SUPPORT_OLD_GPS:
    import rtk_process

BAUD = 921600
HISTORY_SAMPLE_PERIOD_S = 1
TOTAL_HISTORY_SAMPLE_DURATION_S = 120
# MAX_ALLOWED_HACC_MM = 50
MAX_ALLOWED_HACC_MM = 550
MAXIMUM_SERIAL_BYTES_IN_WAITING = 200

class GPS:
    def __init__(self, logger, simulate_hardware=False, single_gps=False, single_gps_port=None):
        self.logger = logger
        self.simulate_hardware = simulate_hardware
        self.simulated_sample = gps_tools.GpsSample(37.353039233,
                                                    -122.333725682, 100,
                                                    ("fix", "fix"), 20, 0,
                                                    time.time(), 0.5)
        self.single_gps = single_gps
        self.single_gps_port = single_gps_port
        self.last_gps_sample = None
        self.last_good_gps_sample = None
        self.gps_buffers = ["", ""]
        self.sample_history = []
        if self.simulate_hardware:
            self.rtk_socket1 = None
            self.rtk_socket2 = None
        else:
            # This is set at runtime, as saving it to flash would mean that
            # the USB connection is flooded with messages when we need to
            # examine the module config.
            msg = pyubx2.UBXMessage.config_set(layers=1, transaction=0, cfgData=[("CFG_RATE_MEAS", 40)])
            msg2 = pyubx2.UBXMessage.config_set(layers=1, transaction=0, cfgData=[("CFG_RATE_NAV", 2)])

            if self.single_gps:
                self.stream_gps = Serial(self.single_gps_port, BAUD, timeout=0.10)
                self.stream_gps.write(msg.serialize())
                # Make sure that every measurement becomes a nav solution.
                self.stream_gps.write(msg2.serialize())
                self.ubr_gps = UBXReader(self.stream_gps)
                self.flush_single()
            else:
                # self.stream_front = Serial('/dev/ttySC4', 921600, timeout=0.10) # Center Receiver
                # self.stream_rear = Serial('/dev/ttySC5', 921600, timeout=0.10) # Rear Receiver
                self.stream_front = Serial('/dev/ttySC0', BAUD, timeout=0.10) # Center Receiver
                self.stream_rear = Serial('/dev/ttySC1', BAUD, timeout=0.10) # Rear Receiver
                self.cno_front = 0
                self.cno_rear = 0
                # Set high measurement rate.
                self.stream_front.write(msg.serialize())
                self.stream_rear.write(msg.serialize())
                self.stream_front.write(msg2.serialize())
                self.stream_rear.write(msg2.serialize())
                self.ubr_front = UBXReader(self.stream_front)
                self.ubr_rear = UBXReader(self.stream_rear)
                self.flush_serial()

    def flush_single(self):
        if self.simulate_hardware:
            return
        self.stream_gps.reset_input_buffer()
        self.stream_gps.reset_output_buffer()

    def flush_serial(self):
        if self.simulate_hardware:
            return
        self.stream_front.reset_input_buffer()
        self.stream_front.reset_output_buffer()
        self.stream_rear.reset_input_buffer()
        self.stream_rear.reset_output_buffer()

    def last_sample(self):
        """returns the latest GPS sample, either from the GPS device or simulated"""
        return self.simulated_sample if self.simulate_hardware else self.last_gps_sample

    def last_good_sample(self):
        """returns the latest good GPS sample (non-nil), either from the GPS device or simulated"""
        return self.simulated_sample if self.simulate_hardware else self.last_good_gps_sample

    def update_sample_history(self, latest_sample):
        """
        Save samples in to a buffer periodically, for GPS drift analysis.
        """
        sample_ok = False
        if latest_sample.status == True:
            sample_ok = True
        try:
            if all(latest_sample.status):
                sample_ok = True
        except:
            pass
        if not sample_ok:
            return
        if len(self.sample_history) == 0:
            self.sample_history.append((time.time(),latest_sample))
        if time.time() - self.sample_history[-1][0] > HISTORY_SAMPLE_PERIOD_S:
            self.sample_history.append((time.time(),latest_sample))
        while time.time() - self.sample_history[0][0] > TOTAL_HISTORY_SAMPLE_DURATION_S:
            self.sample_history.pop(0)

    def print_sample_history_stats(self):
        if len(self.sample_history) < 2:
            return
        oldest_sample = self.sample_history[0][1]
        newest_sample = self.sample_history[-1][1]
        data_duration = newest_sample.time_stamp - oldest_sample.time_stamp
        distance_oldest_newest = gps_tools.get_distance(newest_sample, oldest_sample)
        max_distance = 0
        for sample in self.sample_history:
            distance = gps_tools.get_distance(newest_sample, sample[1])
            if distance > max_distance:
                max_distance = distance
        self.logger.info(f"GPS history - oldest-newest distance: {distance_oldest_newest:.3f}, max distance in samples: {max_distance:.3f}, total time: {data_duration}, {len(self.sample_history)} samples")

    def new_gps_sample(self, print_gps):
        """
        <UBX(NAV-PVAT, iTOW=00:51:08, version=0, validDate=1, validTime=1, fullyResolved=1,
        validMag=1, year=2022, month=4, day=16, hour=0, min=51, sec=8, reserved0=0,
        reserved1=342, tAcc=21, nano=180, fixType=3, gnssFixOK=1, diffSoln=1, vehRollValid=0,
        vehPitchValid=1, vehHeadingValid=1, carrSoln=1, confirmedAvai=1, confirmedDate=1,
        confirmedTime=1, numSV=16, lon=-122.3333403, lat=37.3543208, height=83449,
        hMSL=113754, hAcc=30, vAcc=32, velN=-1, velE=4, velD=0, gSpeed=4, sAcc=134,
        vehRoll=0.0, vehPitch=2.12137, vehHeading=288.11954, motHeading=288.11954,
        accRoll=0.0, accPitch=180.0, accHeading=180.0, magDec=12.86, magAcc=0.7,
        errEllipseOrient=116.64, errEllipseMajor=67, errEllipseMinor=32,
        reserved2=1011181288, reserved3=0)>

        <UBX(NAV-PVT, iTOW=23:23:00.200000, year=2022, month=5, day=25, hour=23,
        min=23, second=0, validDate=1, validTime=1, fullyResolved=1, validMag=0,
        tAcc=21, nano=199625482, fixType=3, gnssFixOk=1, difSoln=1, psmState=0,
        headVehValid=0, carrSoln=2, confirmedAvai=1, confirmedDate=1,
        confirmedTime=1, numSV=18, lon=-122.3333634, lat=37.354267,
        height=83784, hMSL=114090, hAcc=14, vAcc=10, velN=2, velE=-1, velD=-9,
        gSpeed=3, headMot=351.58296, sAcc=16, headAcc=93.05322, pDOP=1.33,
        invalidLlh=0, lastCorrectionAge=2, reserved0=558842204, headVeh=0.0,
        magDec=0.0, magAcc=0.0)>

        0 = no ﬁx
        1 = dead reckoning only
        2 = 2D-ﬁx
        3 = 3D-ﬁx
        4 = GNSS + dead reckoning combined
        5 = time only ﬁx
        """
        if self.single_gps:
            return self.new_gps_sample_single(print_gps)
        data_front = None
        data_rear = None
        try:
            (raw_data_front, data_front) = self.ubr_front.read()
            (raw_data_rear, data_rear) = self.ubr_rear.read()

            # Usually we read data fast enough but if the processor gets bogged
            # down a bit we will fall begind and read data will be too old.
            # Old data presents a problem for safe navigation, and will trip
            # navigation safety checks. So here, play catch up as needed.
            while self.stream_front.in_waiting >= MAXIMUM_SERIAL_BYTES_IN_WAITING:
                (raw_data_front, data_front) = self.ubr_front.read()
            while self.stream_rear.in_waiting >= MAXIMUM_SERIAL_BYTES_IN_WAITING:
                (raw_data_rear, data_rear) = self.ubr_rear.read()

            if data_front==None or data_rear==None:
                if data_front==None:
                    self.logger.error(f"NO DATA FROM FRONT GPS MODULE! Rear module returned: {data_rear}")
                if data_rear==None:
                    self.logger.error(f"NO DATA FROM REAR GPS MODULE! Center module returned: {data_front}")
                return None
            if data_front.identity != "NAV-PVT" or data_rear.identity != "NAV-PVT":
                self.logger.error(f"{data_front} {data_rear}")
                return None

            MAX_REREAD_ATTEMPTS = 5
            reread_attempts = 0
            # Make sure both data samples are from the same instant.
            while data_rear.iTOW != data_front.iTOW:
                self.logger.info(f"iTOW correcting: {data_rear.iTOW} {data_front.iTOW}")
                if data_front.iTOW > data_rear.iTOW:
                    data_rear = None
                    (raw_data_rear, data_rear) = self.ubr_rear.read()
                elif data_rear.iTOW > data_front.iTOW:
                    data_front = None
                    (raw_data_front, data_front) = self.ubr_front.read()
                reread_attempts += 1
                if reread_attempts > MAX_REREAD_ATTEMPTS:
                    break

            if data_rear.iTOW != data_front.iTOW:
                # Failed to get matched data. Try again later.
                self.logger.error(f"iTOW mismatch {data_front} {data_rear}")
                return None

            if data_front==None or data_rear==None:
                self.logger.error(f"{data_front} {data_rear}")
                return None

            self.raw_samples = (data_front, data_rear)
            microseconds = int(data_front.nano/1000)
            if microseconds < 0:
                microseconds = 0
            time_stamp = datetime.datetime(data_front.year, data_front.month, data_front.day, data_front.hour, data_front.min, data_front.second, microsecond=microseconds, tzinfo=datetime.timezone.utc)
            local_timezone = datetime.datetime.now().astimezone().tzinfo
            time_stamp = time_stamp.replace(tzinfo=datetime.timezone.utc).astimezone(tz=local_timezone)
            heading = gps_tools.get_heading(data_rear, data_front)
            robot_center_point = gps_tools.return_midpoint(data_front, data_rear)
            is_good_rtk_fix_front = data_front.hAcc < MAX_ALLOWED_HACC_MM and data_front.fixType>=3 and data_front.difSoln==1
            is_good_rtk_fix_rear = data_rear.hAcc < MAX_ALLOWED_HACC_MM and data_rear.fixType>=3 and data_rear.difSoln==1
            latest_sample = gps_tools.GpsSample(robot_center_point.lat, robot_center_point.lon,
                data_front.hMSL/MM_IN_METER, (is_good_rtk_fix_front, is_good_rtk_fix_rear), (
                data_front.numSV, data_rear.numSV), heading, time_stamp, 0)
            # print(f"Speed: {data.gSpeed}, Heading: {data.vehHeading}, Accuracy: {data.accHeading}")
            self.update_sample_history(latest_sample)
            if print_gps:
                if not is_good_rtk_fix_rear or not is_good_rtk_fix_front:
                    fix_string = ""
                    if not data_front.hAcc < MAX_ALLOWED_HACC_MM:
                        fix_string += f"Front hAcc fail: {data_front.hAcc}. "
                    if not data_rear.hAcc < MAX_ALLOWED_HACC_MM:
                        fix_string += f"Rear hAcc fail: {data_rear.hAcc}. "
                    if not data_front.fixType>=3:
                        fix_string += f"Front fix fail {data_front.fixType}. "
                    if not data_rear.fixType>=3:
                        fix_string += f"Rear fix fail {data_rear.fixType}. "
                    if not data_front.difSoln==1:
                        fix_string += f"Front RTK fail, difSoln {data_front.difSoln}. "
                    if not data_rear.difSoln==1:
                        fix_string += f"Rear RTK fail, difSoln {data_rear.difSoln}. "
                    self.logger.info(fix_string)


                if self.last_gps_sample:
                    period = (latest_sample.time_stamp - self.last_gps_sample.time_stamp).total_seconds()
                else:
                    period = 0.0
                self.logger.info("Lat {:.8f}, Lon {:.8f}, Height {:.2f}, Head {:.1f}, Fix:{}, Period: {:.3f}, hACC {},{} CNO {:.1f},{:.1f}".format(
                    latest_sample.lat, latest_sample.lon, latest_sample.height_m, latest_sample.azimuth_degrees, latest_sample.status, period, data_front.hAcc, data_rear.hAcc, self.cno_front, self.cno_rear))
            self.last_gps_sample = latest_sample
            if print_gps:
                msg = pyubx2.UBXMessage("NAV", "NAV-SAT", pyubx2.POLL)
                self.stream_front.write(msg.serialize())
                self.stream_rear.write(msg.serialize())
                while data_front.identity != "NAV-SAT":
                    (raw_data_front, data_front) = self.ubr_front.read()
                while data_rear.identity != "NAV-SAT":
                    (raw_data_rear, data_rear) = self.ubr_rear.read()
                # print("GOT NAV_SAT MESSAGES")
                # print(data_front)
                cno_sum = 0
                for sat_num in range(data_front.numSvs):
                    health = eval(f"data_front.health_{(sat_num+1):02d}")
                    cno = eval(f"data_front.cno_{(sat_num+1):02d}")
                    cno_sum+=cno
                    # print(f"Front Health {sat_num}: {health}, CNO: {cno}")
                self.cno_front = cno_sum/data_front.numSvs
                # print(f"Front CNO average {self.cno_front}")
                # print("GOT NAV_SAT MESSAGE")
                # print(data_rear)
                cno_sum = 0
                for sat_num in range(data_rear.numSvs):
                    health = eval(f"data_rear.health_{(sat_num+1):02d}")
                    cno = eval(f"data_rear.cno_{(sat_num+1):02d}")
                    cno_sum+=cno
                    # print(f"Rear Health {sat_num}: {health}, CNO: {cno}")
                self.cno_rear = cno_sum/data_rear.numSvs
                # print(f"Rear CNO average {self.cno_rear}")
            return latest_sample
        except Exception as e:
            self.logger.error("EXCEPTION IN GPS HANDLER 'new_gps_sample'")
            self.logger.error(e)
            # raise e
            try:
                self.logger.error(f"Center GPS: {data_front}")
                self.logger.error(f"Rear GPS: {data_rear}")
            except:
                pass
            return None


    def new_gps_sample_single(self, print_gps):
        """
        grab GPS data when we are using a single reciever.
        This is not used on acorn but is used on our GPS grid box
        at the farm and relies on acorn code.
        TODO: refactor this file to more cleanly support this?
        """
        data_front = None
        try:
            errors = 0
            ERROR_LIMIT = 5
            while True:
                (raw_data_gps, data_gps) = self.ubr_gps.read()
                # self.logger.info(raw_data_gps)
                if data_gps==None:
                    if errors > ERROR_LIMIT:
                        self.logger.error("NO DATA FROM GPS MODULE!")
                        return None
                    errors+=1
                elif data_gps.identity != "NAV-PVT":
                    if errors > ERROR_LIMIT:
                        # self.logger.error(f"{data_front}")
                        return None
                    errors+=1
                else:
                    break

            self.raw_samples = (data_gps)
            microseconds = int(data_gps.nano/1000)
            if microseconds < 0:
                microseconds = 0
            time_stamp = datetime.datetime(data_gps.year, data_gps.month, data_gps.day, data_gps.hour, data_gps.min, data_gps.second, microsecond=microseconds, tzinfo=datetime.timezone.utc)
            local_timezone = datetime.datetime.now().astimezone().tzinfo
            time_stamp = time_stamp.replace(tzinfo=datetime.timezone.utc).astimezone(tz=local_timezone)
            heading = 0
            is_good_rtk_fix = data_gps.hAcc < MAX_ALLOWED_HACC_MM and data_gps.fixType>=3 and data_gps.difSoln==1
            latest_sample = gps_tools.GpsSample(data_gps.lat, data_gps.lon,
                data_gps.hMSL/MM_IN_METER, is_good_rtk_fix, (
                data_gps.numSV), heading, time_stamp, 0)
            # print(f"Speed: {data.gSpeed}, Heading: {data.vehHeading}, Accuracy: {data.accHeading}")
            self.update_sample_history(latest_sample)
            if print_gps:
                if self.last_gps_sample:
                    period = (latest_sample.time_stamp - self.last_gps_sample.time_stamp).total_seconds()
                else:
                    period = 0.0
                self.logger.info("Lat: {:.8f}, Lon: {:.8f}, Height M: {:.2f}, Heading: {:.1f}, Fix: {}, Period: {:.3f}, Differential: {}, hAcc: {}".format(
                    latest_sample.lat, latest_sample.lon, latest_sample.height_m, latest_sample.azimuth_degrees, latest_sample.status, period, data_gps.difSoln==1, data_gps.hAcc))
            return latest_sample
        except Exception as e:
            self.logger.error("EXCEPTION IN GPS HANDLER 'new_gps_sample_single'")
            self.logger.error(e)
            try:
                self.logger.error(f"GPS: {data_gps}")
            except:
                pass
            return None


    def update_simulated_sample(self, lat, lon, azimuth_degrees):
        """update simulated sample, only useful if the device is simulated"""
        self.simulated_sample = gps_tools.GpsSample(
            lat, lon, self.simulated_sample.height_m, ("fix", "fix"), 20,
            azimuth_degrees, time.time(), 0.5)

    def is_dual_fix(self):
        """check if the last sample is dual fix."""
        try:
            return all(self.last_sample().status)
        except:
            return False

    def dump(self):
        """dump the sample (only if simulated)"""
        if self.simulate_hardware:
            self.logger.info(
                f"Lat: {self.simulated_sample.lat:.10f}, "
                f"Lon: {self.simulated_sample.lon:.10f}, "
                f"Azimuth: {self.simulated_sample.azimuth_degrees:.2f}, "
                f"Distance: {2.8:.4f}, Fixes: ({True}, {True}), Period: {0.100:.2f}")
