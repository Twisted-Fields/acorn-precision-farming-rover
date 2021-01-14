
filename=$(date +"%m_%d_%Y_%H:%M:%S")

gst-launch-1.0 --verbose v4l2src device="/dev/video0" !  "video/x-raw,format=UYVY,width=4208,height=3120,framerate=9/1" ! nvvidconv ! "video/x-raw(memory:NVMM), format=(string)I420, width=(int)4208, height=(int)3120" ! omxh264enc control-rate=2 preset-level=3 bitrate=20000000 ! matroskamux ! queue ! filesink location=/home/nvidia/Videos/{$filename}_1.mkv \
 v4l2src device="/dev/video1" !   "video/x-raw,format=UYVY,width=4208,height=3120,framerate=9/1" ! nvvidconv ! "video/x-raw(memory:NVMM), format=(string)I420, width=(int)4208, height=(int)3120" ! omxh264enc control-rate=2 preset-level=3 bitrate=20000000 ! matroskamux ! queue ! filesink location=/home/nvidia/Videos/{$filename}_2.mkv -e
