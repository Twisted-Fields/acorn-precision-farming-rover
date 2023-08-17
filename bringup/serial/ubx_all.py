# from serial import Serial
# from pynmeagps import NMEAReader
# stream = Serial('/dev/ttySC5', 115200, timeout=3)
# nmr = NMEAReader(stream)
# while True:
#     (raw_data, parsed_data) = nmr.read()
#     print(parsed_data)
import time
import sys
from serial import Serial
from pyubx2 import UBXReader
from pyubx2.ubxhelpers import itow2utc
from pyubx2 import UBXMessage
# layers = 1
# transaction = 0
# cfgData = [("CFG_RATE_MEAS", 40)]
# msg = UBXMessage.config_set(layers=1, transaction=0, cfgData=[("CFG_RATE_MEAS", 40)])
# print(msg)
# sys.exit()

port_names = ('/dev/ttySC0', '/dev/ttySC1', '/dev/ttySC2', '/dev/ttySC3')
# port_names = ('/dev/ttySC1','/dev/ttySC3')

BAUD = 921600
# BAUD = 999999
# BAUD = 1000000  # NOTE: Actual baud rate is 1M but must set as 921600


port_list = []

for name in port_names:
    port_list.append(Serial(name, BAUD, timeout=0.1))

reader_list = []
for port in port_list:
    port.flushInput()
    port.flushOutput()
    reader_list.append(UBXReader(port))

# ubr0 = UBXReader(stream0)
# ubr1 = UBXReader(stream1)

start = time.time()
loop_count = 0
while True:
    try:
        # ubr._stream.write(msg.serialize())
        idx = 0
        for reader in reader_list:
            (raw_data, parsed_data0) = reader.read()
            print(f"{port_names[idx]} | {parsed_data0}")
            idx+=1
            # print(dir(reader))
        # (raw_data, parsed_data1) = ubr1.read()
            print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
        # print(f"{parsed_data0.numSV} {parsed_data1.numSV}")
        # print(parsed_data.iTOW)
        # print(parsed_data.identity)
        # print(dir(parsed_data))


    except KeyboardInterrupt:
        break
    except Exception:
        pass

    # if time.time() - start > 5:
    #     if msg:
    #         ubr._stream.write(msg.serialize())
    #         msg = None
    # print(stream.readline())
    # loop_count += 1
    # if time.time() - start > 1.0:
    #     print(loop_count)
    #     loop_count = 0
    #     start = time.time()
    # print(parsed_data.identity)
    #print(f"{parsed_data.lat} {parsed_data.lon} {itow2utc(parsed_data.iTOW)} {parsed_data.vehHeading}")

    # print(itow2utc(parsed_data.iTOW))
    # print(parsed_data.fixType)



"""
<UBX(NAV-PVT, iTOW=23:23:00.200000, year=2022, month=5, day=25, hour=23,
min=23, second=0, validDate=1, validTime=1, fullyResolved=1, validMag=0,
tAcc=21, nano=199625482, fixType=3, gnssFixOk=1, difSoln=1, psmState=0,
headVehValid=0, carrSoln=2, confirmedAvai=1, confirmedDate=1,
confirmedTime=1, numSV=18, lon=-122.3333634, lat=37.354267,
height=83784, hMSL=114090, hAcc=14, vAcc=10, velN=2, velE=-1, velD=-9,
gSpeed=3, headMot=351.58296, sAcc=16, headAcc=93.05322, pDOP=1.33,
invalidLlh=0, lastCorrectionAge=2, reserved0=558842204, headVeh=0.0,
magDec=0.0, magAcc=0.0)>

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
