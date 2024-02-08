"""
*********************************************************************
                     This file is part of:
                       The Acorn Project
             https://wwww.twistedfields.com/research
*********************************************************************
Copyright (c) 2019-2021 Taylor Alexander, Twisted Fields LLC
Copyright (c) 2021 The Acorn Project contributors (cf. AUTHORS.md).

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*********************************************************************
"""

import redis
import time
import pickle
import sys
from datetime import datetime
import glob

from scipy.interpolate import splprep, splev
sys.path.append('../vehicle')

from remote_control_process import EnergySegment, PathSection

HIDE_AUTOGEN_PATHS = True

r = redis.Redis(
    # host='192.168.1.170',
    host='0.0.0.0',
    port=6379)

print("Options:")
print("1: Explore Database")
print("2: Import File")
print("3: Clean Database")
selection = int(input("Enter selection: "))

if selection == 2:
    files = glob.glob('*.pickle')
    if len(files) == 0:
        print("No files found! Exiting.")
        sys.exit()
    file_index = 0
    for file in files:
        file_index += 1
        print(f"{file_index}: {file}")
    print(f"{file_index+1}: None, exit program.")
    selection = int(input("Enter selection: "))
    if selection-1 == file_index:
        sys.exit()
    with open(files[selection-1], 'rb') as handle:
        path = pickle.load(handle)
        if isinstance(path[0], PathSection):
            point_count = 0
            for section in path:
                point_count += len(section.points)
            print(f"Path opened. Path has {len(path)} PathSection elements with a total of {point_count} points.")
        else:
            print(f"Path opened. Path has {len(path)} list elements.")
        path_name = input(f"Filename was {files[selection-1]}, please enter desired path name in database: ")
        site_list = []
        for key in r.scan_iter():
            split_key = key.decode('utf-8').split(":")
            if len(split_key[0]) > 0:
                site_list.append(split_key[0])
        site_list = list(set(site_list))
        print("Existing site names: ")
        print(site_list)
        site_name = input("Enter site name: ")
        key_tuple = (site_name, 'gpspath', path_name, 'key')
        pathkey = ":".join(key_tuple)
        selection = input(f"Will save path with key {pathkey} to database. Continue? y/(n): ")
        if selection == "y":
            r.set(pathkey,pickle.dumps(path))
            r.bgsave()
            print("Changes saved to database! Exiting.")
        else:
            print("Nothing changed. Exiting.")
        sys.exit()


key_list = []
site_list = []
path_list = []
pathsection_list = []
robot_list = []
gpspolygon_list = []
energy_segment_list = []
for key in r.scan_iter():
    # if 'acorn_test' in key.decode('utf-8') and 'robot' in key.decode('utf-8'):
    #     r.delete(key)
    key_list.append(key.decode('utf-8'))
    split_key = key.decode('utf-8').split(":")
    if(len(split_key) < 3):
        continue
    if len(split_key[0]) > 0:
        site_list.append(split_key[0])
    if split_key[1]=="robot" and split_key[3]!="energy_segment":
        robot_list.append(split_key[2])
    elif split_key[1]=="gpspolygon":
        gpspolygon_list.append(split_key[2])
    elif split_key[1]=="gpspath":
        if HIDE_AUTOGEN_PATHS and "autogen" in split_key[2]:
            continue
        path = pickle.loads(r.get(key))
        if isinstance(path[0], PathSection):
            pathsection_list.append(split_key[2])
        path_list.append(split_key[2])
    elif split_key[3]=="energy_segment":
        energy_segment_list.append(key.decode('utf-8'))

        # print(type(pickle.loads(r.get(key))))
# for key in key_list:
#     print(key)
if selection == 1:
    site_list = list(set(site_list))
    robot_list = list(set(robot_list))
    gpspolygon_list = list(set(gpspolygon_list))

    print(site_list)
    print(robot_list)
    # print(energy_segment_list)
    print()
    selector_index = 0
    selectable_paths = []
    for path in path_list:
        selector_index += 1
        selectable_paths.append(path)
        if path in pathsection_list:
            print(f"{selector_index}:(P) {path}")
        else:
            print(f"{selector_index}:    {path}")
    print()
    pathnum = int(input("Enter a path number: "))
    path_name = selectable_paths[pathnum-1]
    key_tuple = (site_list[0], 'gpspath', path_name, 'key')

    pathkey = ":".join(key_tuple)
    print(f"Loading path with key: {pathkey}")
    path = pickle.loads(r.get(pathkey))

    print()
    if isinstance(path[0], PathSection):
        point_count = 0
        for section in path:
            point_count += len(section.points)
        print(f"Path opened. Path has {len(path)} PathSection elements with a total of {point_count} points.")
    else:
        print(f"Path opened. Path has {len(path)} list elements.")
    print()

    print("What would you like to do?")
    print("1: Sample (print first item)")
    print("2: Export to disk")
    print("3: Print in full")
    print("4: Delete path")
    print("5: Fix Control Values")


    selection = int(input("Enter selection: "))

    if(selection==1):
        if isinstance(path[0], PathSection):
            for item in path:
                print("--------------------------------------------------")
                print(item)
        else:
            print(path[0])

    if(selection==2):
        # print(path[0])
        date_string = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p")
        filename = f"{path_name}_{date_string}.pickle"
        with open(filename, 'wb') as handle:
            pickle.dump(path, handle, protocol=pickle.HIGHEST_PROTOCOL)
        print(f"Saved file as {filename}")

    if(selection==3):
        print(path)

    if(selection==4):
        print(f" WARNING! This will delete path {pathkey}.")
        selection = input(f"Delete the key? y/(n): ")
        if selection == "y":
            r.delete(pathkey)
            print("Path deleted! Exiting.")
        else:
            print("Nothing changed. Exiting.")
        sys.exit()

    if(selection==5):
        if isinstance(path[0], PathSection):
            path[0].control_values.lateral_p *= -1
            path[0].control_values.lateral_d *= -1
            for idx in range(len(path)):
                path[idx].control_values = path[0].control_values
                # item.control_values.lateral_d *= -1
                print(path[idx])
            selection = input("Values shown above will be committed to database. Continue? y/(n): ")
            if selection == "y":
                r.set(pathkey,pickle.dumps(path))
                r.bgsave()
                print("Changes saved to database! Exiting.")
            else:
                print("Nothing changed. Exiting.")
            sys.exit()
        else:
            print(path[0])

if selection == 3:
    print("Clean Database Selected.")
    # print(energy_segment_list)
    # key = energy_segment_list[0]
    # r.rename(key,"energy_segment_old")
    newkey = 'twistedfields:robot:acorn1:energy_segment:key'
    # r.rename(newkey,"energy_segment_old")


    # list_length = r.llen(newkey)
    # print(f"List Length: {list_length}")
    # last_seq = 0
    # last_stamp = 0
    # new_list_length = 0
    # for idx in range(list_length-1,list_length-10, -1):
    #     segment = pickle.loads(r.lindex(newkey, idx))
    #     print(len(segment.subsampled_points))
        # if segment.sequence_num != last_seq and segment.time_stamp != last_stamp:
        #     last_seq = segment.sequence_num
        #     last_stamp = segment.time_stamp
        #     # print(last_seq)
        #     r.rpush(newkey, pickle.dumps(segment))
        #     new_list_length+=1
        #     percent_complete = idx/list_length * 100
        #     print(f"new_list_length {new_list_length}, idx {idx}, {percent_complete:.2f}")



    # r.delete(newkey)
    # sys.exit()
    oldkey = "energy_segment_old"
    r.delete(oldkey)
    sys.exit()
    list_length = r.llen(oldkey)
    print(f"List Length: {list_length}")
    last_seq = 0
    last_stamp = 0
    new_list_length = 0
    for idx in range(list_length):
        segment = pickle.loads(r.lindex(oldkey, idx))
        if segment.sequence_num != last_seq and segment.time_stamp != last_stamp:
            last_seq = segment.sequence_num
            last_stamp = segment.time_stamp
            # print(last_seq)
            segment.subsampled_points = []
            r.rpush(newkey, pickle.dumps(segment))
            new_list_length+=1
            percent_complete = idx/list_length * 100
            print(f"new_list_length {new_list_length}, idx {idx}, {percent_complete:.2f}")
            print(segment.sequence_num)
            print(segment.time_stamp)
