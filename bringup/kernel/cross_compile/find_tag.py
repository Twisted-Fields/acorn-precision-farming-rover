
import subprocess
import re

ROBOT_HOSTNAME = "acornv2"

match_string=r"Installed: \d:(\d.\d+)"


command = ["ssh","acorn@"+ROBOT_HOSTNAME,"apt policy raspberrypi-kernel"]
result = subprocess.run(command, capture_output=True)
# print(result.stdout)

x = re.findall(match_string, str(result.stdout))
print(x[0])
