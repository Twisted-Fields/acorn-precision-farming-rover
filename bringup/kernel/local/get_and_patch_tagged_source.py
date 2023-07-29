
import subprocess
import re
import os

folder="/home/acorn/linux"

replacement_file="https://raw.githubusercontent.com/torvalds/linux/master/net/can/isotp.c"

match_string=r"Installed: \d:(\d.\d+)"

command = ["apt policy raspberrypi-kernel"]
result = subprocess.run(command, capture_output=True, shell=True)
# print(result.stdout)

tag = re.findall(match_string, str(result.stdout))[0]
print(f"Tag is {tag}")

print(f"Downloading Sources to {folder}")

os.mkdir(folder)
command = f"git clone --depth=1 --branch {tag} https://github.com/raspberrypi/linux {folder}"
print(command)
result = subprocess.run(command, capture_output=True, shell=True)
print(result.stdout)

print("Replacing isotp.c file with corrected version")
command=f"wget -O {folder}/net/can/isotp.c {replacement_file}"
print(command)
result = subprocess.run(command, capture_output=True, shell=True)
print(result.stdout)
