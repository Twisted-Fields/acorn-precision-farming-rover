import redis
import time
import pickle
from scipy.interpolate import CubicSpline
import numpy as np
import matplotlib.pyplot as plt

from scipy.interpolate import splprep, splev

import spline_lib

_SMOOTH_MULTIPLIER = 0.00000000001

r = redis.Redis(
    host='localhost',
    port=6379)

# r.set('foo', 'bar')



# def smooth_track(gps_coords, smooth_factor, num_points):
#     """ Calculated a spline based on a gps track.
#     Args:
#         gps_coords: A list of dict objects with 'lat' and 'lon' keys.
#         smooth_factor: Any float, but recommend 1-10.
#         num_points: The number of points
#     Returns: Four lists. Smoothed lat and lon coords and original lat and lon
#         coords.
#     """
#     np_points = np.empty((len(gps_coords), 2))
#     orig_lats = []
#     orig_lons = []
#     for idx in range(len(gps_coords)):
#         line = gps_coords[idx]
#         np_points[idx] = ((line['lat'], line['lon']))
#         orig_lats.append(line['lat'])
#         orig_lons.append(line['lon'])
#
#     tck, u = splprep(np_points.T, u=None, s=smooth_factor * _SMOOTH_MULTIPLIER, per=0, t=10)
#     u_new = np.linspace(u.min(), u.max(), num_points)
#     lat_smooth, lon_smooth = splev(u_new, tck, der=0)
#     return lat_smooth, lon_smooth, orig_lats, orig_lons



for key in r.scan_iter():
    print(key)
    if 'gpspath' in str(key) and 'long_strawberry_parking2' in str(key):
        print(key)
       # #     newkey = str(key).replace('-key\'',':key')
       # #     newkey = newkey.replace('b\'','')
       # #     print(newkey)
       # #     # #print(bytes(newkey, encoding='ascii'))
       # #     # # #newkey = "twistedfields:gpspath:{}-key".format(str(key))
       #     r.delete(key)
        gps_coords = pickle.loads(r.get(key))
        spline = spline_lib.GpsSpline(gps_coords, smooth_factor=1, num_points=500)
        #lat_smooth, lon_smooth, orig_x, orig_y = smooth_track(gps_coords, smooth_factor=10, num_points=200)
        #print(list(zip(lat_smooth, lon_smooth)))
        #plt.plot(dat[:,0], dat[:,1],'ro')
        lat_smooth = []
        lon_smooth = []
        for p in spline.points:
            lat_smooth.append(p.lat)
            lon_smooth.append(p.lon)
        orig_x = []
        orig_y = []
        for p in gps_coords:
            orig_x.append(p['lat'])
            orig_y.append(p['lon'])
            print(p['lat'], p['lon'])


        point_of_interest = {'lat':37.35409860533507, 'lon':-122.33325479993744}
        point_of_interest = {'lat':37.35398195436689, 'lon':-122.33308312401907}
        point_of_interest = {'lat': 37.3540842425, 'lon': -122.3333173125}
        point_of_interest = {'lat': 37.35402, 'lon': -122.3334}

            #37.3540842425, -122.3333173125
        start = time.time()
        for _ in range(1000):
            closeu = spline.closestUOnSpline(point_of_interest)
        time1 = time.time()-start
        print("closeu {}, time {}".format(closeu, time1))
        start = time.time()
        for _ in range(1000):
            closeu2 = spline.closestUOnSplinePoints(point_of_interest)
        time2 = time.time()-start
        print("closeu2 {}, time {}".format(closeu2, time2))
        coord = spline.coordAtU(closeu)
        coord2 = spline.coordAtU(closeu2)
        mag = spline.slopeRadiansAtU(closeu)
        import math
        mag = math.degrees(mag)
        #mag = mag[0]/mag[1] * 90
        print("closeu {}, coord {}, mag {}".format(closeu, coord, mag))


        plt.plot(orig_x, orig_y, 'ro')
        plt.plot(lat_smooth, lon_smooth, 'bo')
        plt.plot(point_of_interest['lat'],point_of_interest['lon'], 'go', markersize=20)
        plt.plot(coord.lat, coord.lon, 'mo', markersize=20)
        plt.plot(coord2.lat, coord2.lon, 'yo', markersize=20)
        plt.title(str(key))
        plt.show()
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
