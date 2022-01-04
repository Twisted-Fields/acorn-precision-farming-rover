# gst-launch-1.0 --verbose v4l2src device="/dev/video0" !  "image/jpeg,framerate=20/1,width=4208,height=3120" ! jpegparse ! nvjpegdec ! video/x-raw ! nvvidconv ! \
# "video/x-raw(memory:NVMM), format=(string)I420, width=(int)4208, height=(int)3120" ! \
#  omxh264enc control-rate=2 preset-level=3 bitrate=20000000 ! matroskamux ! queue ! filesink location=/home/nvidia/Videos/test_1.mkv

# gst-launch-1.0 v4l2src device="/dev/video0" ! "video/x-raw, format=(string)UYVY, width=(int)4208, height=(int)3120" ! nvvidconv ! \
# "video/x-raw(memory:NVMM), format=(string)I420, width=(int)4208, height=(int)3120" ! \
# omxh264enc control-rate=2 preset-level=3 bitrate=20000000 ! matroskamux ! queue ! filesink location=/home/nvidia/Videos/test_1.mkv \


gst-launch-1.0 v4l2src device="/dev/video1" ! "video/x-raw, format=(string)UYVY, width=(int)4096,height=(int)2160" ! nvvidconv ! \
"video/x-raw(memory:NVMM), format=(string)I420, width=(int)4096, height=(int)2160" ! \
omxh264enc control-rate=2 preset-level=3 bitrate=20000000 ! matroskamux ! queue ! filesink location=/home/nvidia/Videos/test_1.mkv


gst-launch-1.0 v4l2src device="/dev/video1" ! "video/x-raw, format=(string)UYVY, width=(int)4208,height=(int)3120" ! nvvidconv ! \
"video/x-raw(memory:NVMM), format=(string)I420, width=(int)4096, height=(int)3037" ! \
nvv4l2h265enc  bitrate=8000000 ! h265parse ! qtmux ! filesink location=/home/nvidia/Videos/test_h265.mp4 -e


#omxh264enc control-rate=2 preset-level=3 bitrate=20000000 ! matroskamux ! queue ! filesink location=/home/nvidia/Videos/test_1.mkv
