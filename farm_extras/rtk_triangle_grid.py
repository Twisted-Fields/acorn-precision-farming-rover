import rtk_process
import time
import gps_tools
import subprocess
import utm
import math
import logging
import sys
import numpy as np
from utils import config_logging
from rpi_ws281x import PixelStrip, Color



# LED strip configuration:
LED_COUNT = 97        # Number of LED pixels.
SKIP_LEDS = LED_COUNT - 60
LED_PIN = 10          # GPIO pin connected to the pixels (18 uses PWM!).
LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA = 10          # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255  # Set to 0 for darkest and 255 for brightest
LED_INVERT = False    # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
strip.begin()

def led_color(strip, color):
    for i in range(strip.numPixels()):
        if i < SKIP_LEDS:
            strip.setPixelColor(i, Color(0,0,0))
            continue
        strip.setPixelColor(i, color)
    strip.show()

def led_partial(strip, color, stop_index):
    for i in range(strip.numPixels()):
        if i > stop_index:
            strip.setPixelColor(i, Color(0,0,0))
        else:
            strip.setPixelColor(i, color)
    strip.show()

def led_percent(strip, percent, color):
    for i in range(strip.numPixels()):
        if i < SKIP_LEDS:
            strip.setPixelColor(i, Color(0,0,0))
            continue
        RESOLUTION=20
        idx = (i-SKIP_LEDS)%RESOLUTION
        if idx > percent/(100/RESOLUTION) and idx>0:
            strip.setPixelColor(i, Color(0,0,0))
        else:
            strip.setPixelColor(i, color)
    strip.show()

led_color(strip, Color(0,0,0))


ROAD_ALIGNED_8_METER_GRID = 1
NORTH_ALIGNED_4_METER_GRID = 2

GRID_VERSION = NORTH_ALIGNED_4_METER_GRID

USE_DATABASE = False

if USE_DATABASE:
    import redis
    import pickle
    r = redis.Redis(
        host='192.168.1.170',
        port=6379)

STATIONARY_TIME = 30

logger = logging.getLogger('rtk_triangles')

config_logging(logger)

tcp_sock1 = None

if not USE_DATABASE:
    rtk_process.launch_rtk_sub_procs(single=True, logger=logger)
    tcp_sock1, tcp_sock2 = rtk_process.connect_rtk_procs(single=True, logger=logger)
    # raise Exception()
print_gps_counter = 0
latest_sample = None
position_list = []
last_bad_status_time = 0
bad_tone_time = 0
good_tone_time = 0
save_file_okay = True

time.sleep(0.5)
led_partial(strip, Color(255,0,0), 20)
time.sleep(0.5)
led_partial(strip, Color(255,0,255), 20)
time.sleep(0.5)
led_partial(strip, Color(0,255,0), 20)
time.sleep(0.5)
led_partial(strip, Color(255,215,0), 20)
time.sleep(0.5)
led_partial(strip, Color(0,255,255), 20)


# BASE_LOCATION = (37.353766071, -122.332961493) # GPS BASE STATION

if USE_DATABASE:
    poly_path = None
    for key in r.scan_iter():
        #logger.info(key)
        if 'gpspolygon' in str(key):
            logger.info(key)
            polygon = pickle.loads(r.get(key))
            #logger.info(polygon["geometry"]["coordinates"][0])
            polygon = polygon["geometry"]["coordinates"][0]
            logger.info(polygon)
            # sys.exit()
    #BASE_LOCATION = (polygon[0][1],polygon[0][0])
    #SECOND_POINT = (polygon[1][1],polygon[1][0])
    if GRID_VERSION == NORTH_ALIGNED_4_METER_GRID:
        BASE_LOCATION = (37.3535689340, -122.3294015900)
        point = gps_tools.project_point(gps_tools.GpsPoint(BASE_LOCATION[0],BASE_LOCATION[1]), bearing_degrees=0, distance_meters=10)
        SECOND_POINT = (point.lat, point.lon)
else:

    if GRID_VERSION == ROAD_ALIGNED_8_METER_GRID:
        BASE_LOCATION = (37.3535689340, -122.3294015900)
        SECOND_POINT = (37.3534222740, -122.3291569190)
    if GRID_VERSION == NORTH_ALIGNED_4_METER_GRID:
        BASE_LOCATION = (37.3535689340, -122.3294015900)
        point = gps_tools.project_point(gps_tools.GpsPoint(BASE_LOCATION[0],BASE_LOCATION[1]), bearing_degrees=0, distance_meters=10)
        SECOND_POINT = (point.lat, point.lon)

#-122.32923657,   37.35350514

logger.info(BASE_LOCATION)
logger.info(SECOND_POINT)

grid_size_meters = 2000.0
if GRID_VERSION == ROAD_ALIGNED_8_METER_GRID:
    grid_spacing_meters = 8.0
if GRID_VERSION == NORTH_ALIGNED_4_METER_GRID:
    grid_spacing_meters = 4.0
triangle_height = grid_spacing_meters * math.sqrt(3.0) / 2.0

array_width = int(grid_size_meters/grid_spacing_meters)

base_x, base_y, zone1, zone2 = utm.from_latlon(latitude=BASE_LOCATION[0], longitude=BASE_LOCATION[1])
second_x, second_y, second_zone1, second_zone2 = utm.from_latlon(latitude=SECOND_POINT[0], longitude=SECOND_POINT[1])

grid_angle_radians = math.pi/2.0 - math.atan2(base_y - second_y, base_x - second_x)
logger.info(grid_angle_radians)

offset_index = int(array_width/2.0)


start_offset_x = math.sin(grid_angle_radians) * grid_spacing_meters * offset_index - math.cos(grid_angle_radians) * triangle_height * offset_index
start_offset_y = math.sin(grid_angle_radians) * triangle_height * offset_index + math.cos(grid_angle_radians) * grid_spacing_meters * offset_index

# start_corner_x = base_x - math.sin(grid_angle_radians) * offset_hypoteneuse
# start_corner_y = base_y - math.cos(grid_angle_radians) * offset_hypoteneuse

start_corner_x = base_x - start_offset_x
start_corner_y = base_y - start_offset_y

# start_corner_x = 0
# start_corner_y = 0


grid_array = np.zeros((array_width * (array_width + 1), 2))

logger.info("Generating grid_array")

orig_x = []
orig_y = []

point1 = None
point2 = None
point3 = None

for x_value in range(array_width):
    #logger.info(x_value)
    for y_value in range(array_width):

        x_offset = math.sin(grid_angle_radians) * grid_spacing_meters * x_value - math.cos(grid_angle_radians) * triangle_height * y_value
        y_offset = math.sin(grid_angle_radians) * triangle_height * y_value + math.cos(grid_angle_radians) * grid_spacing_meters * x_value

        if y_value % 2 == 1:
            x_offset += math.sin(grid_angle_radians) * grid_spacing_meters * 0.5
            y_offset += math.cos(grid_angle_radians) * grid_spacing_meters * 0.5

        grid_index = x_value * array_width + y_value - 1
        grid_array[grid_index][0] = start_corner_x + x_offset
        grid_array[grid_index][1] = start_corner_y + y_offset
        orig_x.append(grid_array[grid_index][0])
        orig_y.append(grid_array[grid_index][1])
        if x_value == 0:
            if y_value == 0:
                point1 = (grid_array[grid_index][0],grid_array[grid_index][1])
            if y_value == 1:
                point2 = (grid_array[grid_index][0],grid_array[grid_index][1])
        if x_value == 1:
            if y_value == 0:
                point3 = (grid_array[grid_index][0],grid_array[grid_index][1])

# orig_x.append(100001)
# orig_y.append(100001)


def calc_distance_2D(p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


logger.info(triangle_height)
logger.info(grid_spacing_meters)
logger.info(calc_distance_2D(point1, point2))
logger.info(calc_distance_2D(point1, point3))
# sys.exit()


if USE_DATABASE:
    for key in r.scan_iter():
        logger.info(key)
        if 'twistedfields:robot:acorn1:key' in str(key):
            acorn = pickle.loads(r.get(key))
            logger.info(acorn)
            acorn.gps_path_data = []
            for index in range(len(orig_x)):
                location = utm.to_latlon(orig_x[index], orig_y[index], zone1, zone2)
                sample = gps_tools.GpsSample(location[0], location[1], 0, ('fix','fix'), (), 0, 0, 0)
                acorn.gps_path_data.append(sample)
                logger.info(location)
            logger.info(acorn.gps_path_data)
            robot_pickle = pickle.dumps(acorn)
            r.set(key, robot_pickle)
            break

    sys.exit()



#logger.info(grid_array)
# sys.exit()
#
# fig = plt.figure()
# ax = fig.add_subplot(111)
# ax.scatter(orig_x, orig_y)
# plt.show()
#
# sys.exit()

# DISTANCE_LIMIT = 0.5
DISTANCE_LIMIT = 0.1
audio_thresh = 5

#sample_tick = 0
buffer = ""
while True:

    # try:
    #     strip.end()
    #     strip.begin() # Shouldn't be necessary but the code was hanging without this.
    # except Exception as e:
    #     logger.error(e)

    buffer, latest_sample = rtk_process.rtk_loop_once_single_receiver(tcp_sock1, buffer, print_gps=(print_gps_counter % 10 == 0), last_sample=latest_sample, logger=logger)
    try:
        latest_sample.lat
    except:
        logger.error("Sample Error. May just need to wait for GPS system to start.")
        led_percent(strip, 10, Color(255, 0, 0))
        continue
    # print_gps_counter += 1
    #logger.info(latest_sample)
#    sample_tick+=1
    base_x, base_y, zone1, zone2 = utm.from_latlon(latitude=latest_sample.lat, longitude=latest_sample.lon)
    sample_point = np.array([base_x,base_y])
    dist = np.linalg.norm(grid_array-sample_point,ord=2, axis=1.)
    distance = np.amin(dist)
    logger.info(distance)
    # continue
    # logger.info(latest_sample.status)

    if latest_sample.status == 'fix' and latest_sample.rtk_age < 20:
        if distance < DISTANCE_LIMIT:
            # if sample_tick > 3:
            #     sample_tick = 0
                #subprocess.Popen("aplay /home/pi/complete.wav", shell=True)
            led_color(strip,Color(0, 180, 0))
        else:
            # if sample_tick > distance:
            #     sample_tick = 0
                # subprocess.Popen("aplay /home/pi/wait.wav", shell=True)
            max_dist = (grid_spacing_meters/2.0 - DISTANCE_LIMIT)
            # distance = 1.0
            percent = int(100 * ((max_dist - (distance - DISTANCE_LIMIT))/max_dist))
            led_percent(strip, percent, Color(0, 0, 180))
            #logger.info(percent)
    else:
        # if sample_tick > 12:
        #     sample_tick = 0
            # subprocess.Popen("aplay /home/pi/error.wav", shell=True)
        led_percent(strip, 10, Color(255, 0, 0))
        time.sleep(0.1)
        # pass
