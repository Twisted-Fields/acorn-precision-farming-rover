gst-launch-1.0 v4l2src device="/dev/video0" !   "video/x-raw, format=(string)UYVY, width=(int)3840,height=(int)2160" ! xvimagesink \
 v4l2src device="/dev/video1" !   "video/x-raw, format=(string)UYVY, width=(int)3840,height=(int)2160" ! xvimagesink -e
