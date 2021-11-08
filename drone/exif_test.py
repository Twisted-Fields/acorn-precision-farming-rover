import shutil
import gps_tools
import sys
import os
from PIL import Image
from PIL.ExifTags import TAGS, GPSTAGS

# This file contains code samples from https://sylvaindurand.org/gps-data-from-photos-with-python/


def get_exif(filename):
    exif = Image.open(filename)._getexif()
    # print(type(exif))

    if exif is not None:
        exif_copy = exif.copy()
        for key, value in exif_copy.items():
            name = TAGS.get(key, key)
            exif[name] = exif.pop(key)

        new_exif = {}
        if 'GPSInfo' in exif:
            for key in exif['GPSInfo'].keys():
                name = GPSTAGS.get(key, key)
                new_exif[name] = exif['GPSInfo'][key]

        return new_exif


def get_coordinates(info):
    for key in ['Latitude', 'Longitude']:
        if 'GPS'+key in info and 'GPS'+key+'Ref' in info:
            e = info['GPS'+key]
            ref = info['GPS'+key+'Ref']
            info[key] = (str(e[0][0]/e[0][1]) + '°' +
                         str(e[1][0]/e[1][1]) + '′' +
                         str(e[2][0]/e[2][1]) + '″ ' +
                         ref)

    if 'Latitude' in info and 'Longitude' in info:
        return [info['Latitude'], info['Longitude']]


def get_decimal_coordinates(info):
    for key in ['Latitude', 'Longitude']:
        if 'GPS'+key in info and 'GPS'+key+'Ref' in info:
            e = info['GPS'+key]
            ref = info['GPS'+key+'Ref']
            info[key] = (e[0][0]/e[0][1] +
                         e[1][0]/e[1][1] / 60 +
                         e[2][0]/e[2][1] / 3600
                         ) * (-1 if ref in ['S', 'W'] else 1)

    if 'Latitude' in info and 'Longitude' in info:
        return [info['Latitude'], info['Longitude']]

# get_decimal_coordinates(exif['GPSInfo'])


sys.path.append('../vehicle')
#GpsPoint = namedtuple('GpsPoint', 'lat lon')

points = []
target = "DJI_0635.JPG"
path = "/home/taylor/Pictures/mapping/cap_dec_2_2020/images/"

target_path = os.path.join(path, target)
target_coords = get_decimal_coordinates(get_exif(target_path))
target_gps = gps_tools.GpsPoint(target_coords[0], target_coords[1])

for r, d, f in os.walk(path):
    for file in f:
        if file.lower().endswith(('.png', '.jpg', '.jpeg')):
            filepath = os.path.join(r, file)
            coords = get_decimal_coordinates(get_exif(filepath))
            gps_point = gps_tools.GpsPoint(coords[0], coords[1])
            distance = gps_tools.get_distance(target_gps, gps_point)
            if coords is not None and distance < 50:
                points.append(
                    (coords[0], coords[1], os.path.basename(filepath)))


index = 0
# exif = get_exif('/home/taylor/Pictures/mapping/cap_dec_2_2020/images/DJI_0173.JPG')
# coords = get_decimal_coordinates(exif)
out_path = "/home/taylor/Pictures/mapping/cap_dec_2_2020/gps_test/"
# os.mkdir(out_path)


print("name, dec, latitude, longitude, n")
for coords in points:
    name = coords[2]
    full_path = os.path.join(path, name)
    shutil.copy(full_path, out_path)
    print("{}, dec, {}, {}, {}".format(coords[2], coords[0], coords[1], index))
    index += 1
