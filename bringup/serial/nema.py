# from serial import Serial
# from pynmeagps import NMEAReader
# stream = Serial('/dev/ttySC5', 115200, timeout=3)
# nmr = NMEAReader(stream)
# while True:
#     (raw_data, parsed_data) = nmr.read()
#     print(parsed_data)
import time

from serial import Serial
from pyubx2 import UBXReader

from pyubx2.ubxhelpers import itow2utc

stream = Serial('/dev/ttySC5', 921600, timeout=0.1)
ubr = UBXReader(stream)
start = time.time()
loop_count = 0
while True:
    (raw_data, parsed_data) = ubr.read()
    # print(stream.readline())
    # loop_count += 1
    # if time.time() - start > 1.0:
    #     print(loop_count)
    #     loop_count = 0
    #     start = time.time()
    # print(parsed_data.identity)
    print(f"{parsed_data.lat} {parsed_data.lon} {itow2utc(parsed_data.iTOW)}")
    # print(parsed_data)
    # print(itow2utc(parsed_data.iTOW))
    # print(parsed_data.fixType)



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





['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__',
'__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__',
'__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__',
'__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_calc_num_repeats', '_checksum',
'_do_attributes', '_do_len_checksum', '_get_aopstatus_version', '_get_cfgnmea_version',
'_get_dict', '_get_mga_version', '_get_relposned_version', '_get_rxmpmp_version',
'_get_rxmpmreq_version', '_get_rxmrlm_version', '_get_timvcocal_version', '_immutable',
'_is_navhp', '_length', '_mode', '_parsebf', '_payload', '_scaling', '_set_attribute',
'_set_attribute_bitfield', '_set_attribute_bits', '_set_attribute_cfgval', '_set_attribute_group',
'_set_attribute_navhp', '_set_attribute_single', '_ubxClass', '_ubxID', 'accHeading', 'accPitch',
'accRoll', 'carrSoln', 'config_del', 'config_poll', 'config_set', 'confirmedAvai', 'confirmedDate',
'confirmedTime', 'day', 'diffSoln', 'errEllipseMajor', 'errEllipseMinor', 'errEllipseOrient',
'fixType', 'fullyResolved', 'gSpeed', 'gnssFixOK', 'hAcc', 'hMSL', 'height', 'hour', 'iTOW',
'identity', 'lat', 'length', 'lon', 'magAcc', 'magDec', 'min', 'month', 'motHeading', 'msg_cls',
'msg_id', 'msgmode', 'nano', 'numSV', 'payload', 'reserved0', 'reserved1', 'reserved2', 'reserved3',
'sAcc', 'sec', 'serialize', 'tAcc', 'vAcc', 'validDate', 'validMag', 'validTime', 'vehHeading',
'vehHeadingValid', 'vehPitch', 'vehPitchValid', 'vehRoll', 'vehRollValid', 'velD', 'velE', 'velN',
'version', 'year']
"""
