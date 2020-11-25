import redis
import time
import pickle
from scipy.interpolate import CubicSpline
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as mp_colors
import sys

from scipy.interpolate import splprep, splev
sys.path.append('../vehicle')
from remote_control_process import EnergySegment


_SMOOTH_MULTIPLIER = 0.00000000001

r = redis.Redis(
    host='acornserver.local',
    port=6379)

# r.set('foo', 'bar')

# self.sequence_num = sequence_num
# self.time_stamp = end_gps.time_stamp
# self.start_gps = start_gps
# self.end_gps = end_gps
# self.duration = end_gps.time_stamp - start_gps.time_stamp
# self.distance_sum = distance_sum
# self.meters_per_second = distance_sum / self.duration
# self.watt_seconds_per_meter = total_watt_seconds/distance_sum
# self.height_change = end_gps.height_m - start_gps.height_m
# self.avg_watts = avg_watts


for key in r.scan_iter():
    #print(key)
    if 'energy_segment' in str(key):
        orig_x = []
        orig_y = []
        orig_z = []
        colors = []
        min_colorval = 9999999
        max_colorval = -9999999
        print(key)
        list_length = r.llen(key)
        print("List Length {}".format(list_length))
        first_stamp = pickle.loads(r.lindex(key, 0)).start_gps.time_stamp
        colorby = ""
        watt_seconds = False
        now = time.time()
        for idx in range(4816, list_length):
            segment = pickle.loads(r.lindex(key, idx))
            #print("sequence, {}, watt_seconds_per_meter, {}, meters_per_second, {}".format(segment.sequence_num, segment.watt_seconds_per_meter, segment.meters_per_second))
            # orig_x.append(segment.start_gps.lat)
            # orig_y.append(segment.start_gps.lon)
            # print(segment.start_gps.lat, segment.start_gps.lon)
            if segment.height_change > -0.15 and segment.watt_seconds_per_meter < 1000 and segment.meters_per_second < 2:
                this_stamp = segment.start_gps.time_stamp
                if now - this_stamp > 3000:
                    print(idx)
                    continue

                # age = this_stamp - first_stamp
                # if age > 4000:
                #     continue
                #
                if watt_seconds:
                    orig_y.append(segment.watt_seconds_per_meter)
                else:
                    orig_y.append(segment.avg_watts)

                try:
                    print(segment.per_motor_watt_average)
                except:
                    continue
                #orig_x.append(segment.meters_per_second)
                orig_x.append(segment.height_change)
                orig_z.append(segment.meters_per_second)
                if colorby == "age":
                    age = this_stamp - first_stamp
                    colors.append(age)
                    #print(age)
                    if age < min_colorval:
                        min_colorval = age
                    if age > max_colorval:
                        max_colorval = age
                else:
                #    print("asiokuhsdflikhj")
                    colors.append(segment.meters_per_second)
                    if segment.meters_per_second < min_colorval:
                        min_colorval = segment.meters_per_second
                    if segment.meters_per_second > max_colorval:
                        max_colorval = segment.meters_per_second
            #    print(segment.start_gps.lat, segment.start_gps.lon)


        print("min_colorval {}, max_colorval {}".format(min_colorval, max_colorval))
        cNorm  = mp_colors.Normalize(vmin=min_colorval, vmax=max_colorval)
        scalarMap = cm.ScalarMappable(norm=cNorm, cmap=cm.jet)

        for idx in range(len(colors)):
            colors[idx] = scalarMap.to_rgba(colors[idx])

        fig = plt.figure()
        #ax = fig.add_subplot(111, projection = '3d')
        ax = fig.add_subplot(111)

        ax.set_title('Acorn energy usage')
        if watt_seconds:
            ax.set_ylabel('watt seconds per meter')
        else:
            ax.set_ylabel('average watts over one meter')
        ax.set_xlabel('height change (m)')

        #ax.scatter(orig_x, orig_y, orig_z, c = colors)
        ax.scatter(orig_x, orig_y, c = colors)
        plt.show()



       # #     newkey = str(key).replace('-key\'',':key')
       # #     newkey = newkey.replace('b\'','')
       # # #     print(newkey)
       # # #     # #print(bytes(newkey, encoding='ascii'))
       # # #     # # #newkey = "twistedfields:gpspath:{}-key".format(str(key))
       # #     r.delete(key)
       #  gps_coords = pickle.loads(r.get(key))
       #  spline = spline_lib.GpsSpline(gps_coords, smooth_factor=1, num_points=500)
       #  #lat_smooth, lon_smooth, orig_x, orig_y = smooth_track(gps_coords, smooth_factor=10, num_points=200)
       #  #print(list(zip(lat_smooth, lon_smooth)))
       #  #plt.plot(dat[:,0], dat[:,1],'ro')
       #  lat_smooth = []
       #  lon_smooth = []
       #  for p in spline.points:
       #      lat_smooth.append(p.lat)
       #      lon_smooth.append(p.lon)

       #
       #
       #  point_of_interest = {'lat':37.35409860533507, 'lon':-122.33325479993744}
       #  point_of_interest = {'lat':37.35398195436689, 'lon':-122.33308312401907}
       #  point_of_interest = {'lat': 37.3540842425, 'lon': -122.3333173125}
       #  point_of_interest = {'lat': 37.35402, 'lon': -122.3334}
       #
       #      #37.3540842425, -122.3333173125
       #  start = time.time()
       #  for _ in range(1000):
       #      closeu = spline.closestUOnSpline(point_of_interest)
       #  time1 = time.time()-start
       #  print("closeu {}, time {}".format(closeu, time1))
       #  start = time.time()
       #  for _ in range(1000):
       #      closeu2 = spline.closestUOnSplinePoints(point_of_interest)
       #  time2 = time.time()-start
       #  print("closeu2 {}, time {}".format(closeu2, time2))
       #  coord = spline.coordAtU(closeu)
       #  coord2 = spline.coordAtU(closeu2)
       #  mag = spline.slopeRadiansAtU(closeu)
       #  import math
       #  mag = math.degrees(mag)
       #  #mag = mag[0]/mag[1] * 90
       #  print("closeu {}, coord {}, mag {}".format(closeu, coord, mag))
       #
       #





        # plt.plot(orig_x, orig_y, 'ro')
    #    plt.plot(lat_smooth, lon_smooth, 'bo')
    #    plt.plot(point_of_interest['lat'],point_of_interest['lon'], 'go', markersize=20)
    #    plt.plot(coord.lat, coord.lon, 'mo', markersize=20)
    #    plt.plot(coord2.lat, coord2.lon, 'yo', markersize=20)
        # plt.title(str(key))
        # plt.show()
         #   print(value)
         #   point_data = []
         #   lats = []
         #   lons = []
         #   utm_x = []
         #   utm_y = []
         # #  try:
         #
         #
         #   for line in value:
         #       lats.append(line['lat'])
         #       lons.append(line['lon'])
         #       point_data.append((line['lat'], line['lon']))
         #       utm_coord = utm.from_latlon(line['lat'], line['lon'])
         #       utm_x.append(utm_coord[0])
         #       utm_x.append(utm_coord[1])
         #   x, y = np.array(lats), np.array(lons)
         #   #simple_coords = rdp(point_data, epsilon=1e-4)
         #   #print("{} points reduced to {}!".format(coords.shape[0], simple_coords.shape[0]))
         #   #plt.plot(simple_coords[:, 0], simple_coords[:, 1], 'ro')
         #   #plt.show()
         #
         #   smooth_factor = 1
         #
         #
         #
         #   dat = np.array([(x,y) for x,y in zip(lats, lons)])
         #    #dat = np.array([(x,y) for x,y in zip(coords.lon[::18], coords.lat[::18])])
         #   tck, u = splprep(dat.T, u=None, s=smooth_factor * _SMOOTH_MULTIPLIER, per=0, t=10)
         #   u_new = np.linspace(u.min(), u.max(), 200)
         #   x_new, y_new = splev(u_new, tck, der=0)
         #   #print(x_new)








           # print(point_data)
           # plt.plot(x, y, 'ro', ms=5)
           # cs = CubicSpline(x, y)
           # xs = 2 * np.pi * np.linspace(0, 1, 100)
           # ax.plot(xs, cs(xs), label="S")
           # plt.show()
           # spl = UnivariateSpline(x, y)
           # xs = np.linspace(-3, 3, 1000)
           # plt.plot(xs, spl(xs), 'g', lw=3)
          # except:
            #   print('exception unpickling key {}'.format(key))
               #r.delete(key)



# while True:
#     value = r.get('foo')
#     print(value)
#     time.sleep(0.1)
