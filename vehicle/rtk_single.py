import rtk_process
import time
import gps_tools
import subprocess


STATIONARY_TIME = 30

rtk_process.launch_rtk_sub_procs(single=True)
tcp_sock1, tcp_sock2 = rtk_process.connect_rtk_procs(single=True)
print_gps_counter = 0
latest_sample = None
position_list = []
last_bad_status_time = 0
bad_tone_time = 0
good_tone_time = 0
save_file_okay = True

while True:
    latest_sample = rtk_process.single_loop(tcp_sock1, print_gps=(print_gps_counter % 10 == 0), last_sample=latest_sample)
    print(latest_sample.status)
    if latest_sample.status == 'fix' and latest_sample.rtk_age < 20:
        print("Append")
        position_list.append(latest_sample)
    else:
        last_bad_status_time = time.time()
    while len(position_list) > 0:
        if latest_sample.time_stamp - position_list[0].time_stamp > STATIONARY_TIME:
            position_list.pop(0)
        else:
            break
    if time.time() - last_bad_status_time < 30 and time.time() - bad_tone_time > 30:
        bad_tone_time = time.time()
        print("Bad status")
        subprocess.Popen("aplay /home/pi/error.wav", shell=True)
    if len(position_list) > 100 and time.time() - last_bad_status_time > STATIONARY_TIME:
        device_stationary = True
        max_distance = 0
        for position in position_list:
            distance = gps_tools.get_distance(latest_sample, position)
            if distance > max_distance:
                max_distance = distance
            if distance > 0.05:
                device_stationary = False
        if device_stationary:
            print("Device Stationary")
            if save_file_okay:
                # save the file
                location_string = "Lat Lon: {:.10f}, {:.10f}, Height M: {:.4f}, Fix: {}, Spread: {}, Stamp: {:.4f}".format(latest_sample.lat, latest_sample.lon, latest_sample.height_m, latest_sample.status, max_distance, latest_sample.time_stamp)
                print(location_string)
                with open("/home/pi/gps_locations.txt", "a+") as gps_file:
                    gps_file.write(location_string + "\n")
                save_file_okay = False
            if time.time() - good_tone_time > 10:
                good_tone_time = time.time()
                subprocess.Popen("aplay /home/pi/complete.wav", shell=True)
        else:
            if time.time() - good_tone_time > 3:
                good_tone_time = time.time()
                subprocess.Popen("aplay /home/pi/wait.wav", shell=True)
            print("Waiting for device to settle.")
            save_file_okay = True

    print_gps_counter += 1
