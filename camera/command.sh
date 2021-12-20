gst-launch-1.0 v4l2src device="/dev/video0" !   "video/x-raw, width=640, height=480, format=(string)UYVY" ! xvimagesink \
 v4l2src device="/dev/video1" !   "video/x-raw, width=640, height=480, format=(string)UYVY" ! xvimagesink -e
