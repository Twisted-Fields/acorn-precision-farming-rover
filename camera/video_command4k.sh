
filename=$(date +"%m_%d_%Y_%H:%M:%S")

gst-launch-1.0 v4l2src device="/dev/video0" !   "video/x-raw, format=(string)UYVY, width=(int)4096,height=(int)2160" ! nvvidconv ! "video/x-raw(memory:NVMM), format=(string)I420, width=(int)4096, height=(int)2160" ! omxh264enc control-rate=2 preset-level=3 bitrate=20000000 ! matroskamux ! queue ! filesink location=/media/nvidia/external/Videos/{$filename}_1.mkv \
 v4l2src device="/dev/video1" !   "video/x-raw, format=(string)UYVY, width=(int)4096,height=(int)2160" ! nvvidconv ! "video/x-raw(memory:NVMM), format=(string)I420, width=(int)4096, height=(int)2160" ! omxh264enc control-rate=2 preset-level=3 bitrate=20000000 ! matroskamux ! queue ! filesink location=/media/nvidia/external/Videos/{$filename}_2.mkv -e
