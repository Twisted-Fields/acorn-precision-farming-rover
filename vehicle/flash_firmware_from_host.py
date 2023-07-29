
import subprocess

FIRMWARE_FOLDER = "/home/taylor/Software/2023_code/rp2040_combine/"
ROBOT_HOSTNAME = "acorn3"
CAN_SUBPATH = ".pio/build/canbus/firmware.bin"
MOTOR_SUBPATH = ".pio/build/motorcontroller/firmware.bin"

CAN_DEST_NAME = "firmware_can.bin"
MOTOR_DEST_NAME = "firmware_motor.bin"

CAN_ADDRESSES = [0x7]

FLASH_CAN = True
FLASH_MOTOR = True


def copy_file(file_path, dest_name):
    command = ["rsync","-aP", file_path, "acorn@"+ROBOT_HOSTNAME+":/home/acorn/acorn/vehicle/"+dest_name]
    print(command)
    result = subprocess.run(command, capture_output=True)
    print(result.stdout)

if FLASH_CAN:
    copy_file(FIRMWARE_FOLDER+CAN_SUBPATH, CAN_DEST_NAME)
    print("Copied CAN firmware")
if FLASH_MOTOR:
    copy_file(FIRMWARE_FOLDER+MOTOR_SUBPATH, MOTOR_DEST_NAME)
    print("Copied MOTOR firmware")

for address in CAN_ADDRESSES:
    command = ["ssh","acorn@"+ROBOT_HOSTNAME,f"cd /home/acorn/acorn/vehicle ; python3 can_firmware.py {address} {int(FLASH_CAN)} {int(FLASH_MOTOR)}"]
    print(command)
    result = subprocess.run(command, capture_output=True)
    print(result.stdout)
