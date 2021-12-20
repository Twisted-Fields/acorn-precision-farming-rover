NV_CAPS='video/x-raw(memory:NVMM), width=1920, height=1080, format=I420, framerate=30/1'
IN_CAPS='video/x-raw,format=UYVY,width=1920,height=1080,framerate=30/1'
OUT_CAPS='video/x-raw,format=RGBA,width=1920,height=1080,framerate=30/1'
gst-launch-1.0 nvcamerasrc fpsRange="30.0 30.0" ! capsfilter caps="$NV_CAPS" ! nvvidconv flip-method=2 ! capsfilter caps="$IN_CAPS" ! ptzr tilt-level=-1 ! capsfilter caps="$OUT_CAPS" ! videoconvert ! xvimagesink
