import os
import sys
import re
from datetime import datetime


date_list = []

name_match = "tmux_output_((\d*)_(\d*)_(\d*)_(\d*)_(\d*)_(PM|AM))"
regex = re.compile(name_match)
log_file_names = os.listdir("../logs")
for file in log_file_names:
    try:
        date_string = regex.findall(file)[0][0]
        date  = datetime.strptime(date_string, "%Y_%m_%d_%I_%M_%p")
        if date not in date_list:
            date_list.append(date)
    except:
        pass

checklist = ["Exception","Traceback","ERROR", "Error"]

date_list.sort()
for date in date_list[-10:]:
    print("##################################")
    date_string = date.strftime('%Y_%m_%d_%I_%M_%p')
    # print(date_string)
    print(date)
    for file in log_file_names:
        if date_string in file:
            print(file)
            with open("../logs/" + file, encoding = 'utf-8') as file:
                content = file.readlines()
                for line in content[-20:]:
                    if any(item in line for item in checklist):
                        for line in content[-20:]:
                            print(line,end="")
                        break

    print("##################################")
