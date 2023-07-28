
sudo cp kernel/can-isotp.ko /lib/modules/`uname -r`/kernel/net/can/

sudo echo "
# /etc/modules: kernel modules to load at boot time.
#
# This file contains the names of kernel modules that should be loaded
# at boot time, one per line. Lines beginning with '#' are ignored.
can-isotp
" > /etc/modules

sudo echo "options can-isotp max_pdu_size=200000" > /etc/modprobe.d/can-isotp.conf
