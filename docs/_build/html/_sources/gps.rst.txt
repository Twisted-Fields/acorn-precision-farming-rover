

Acorn and GPS
=================================================

Acorn Uses RTKLIB to determine GPS position data to within a few centimeters.
http://rtkexplorer.com/
http://rtkexplorer.com/resources/

Make sure to configure new GPS recievers with u-center:
http://rtkexplorer.com/configuring-the-gps-receiver/

u-center can be run on linux with wine.
See the following page for more information on how to use Wine with serial ports.
https://www.onetransistor.eu/2015/12/wine-serial-port-linux.html

Notably, wine will create symlinks to existing serial ports in the following directory:
~/.wine/dosdevices/


RTKLIB configuration file details:
https://rtklibexplorer.wordpress.com/2018/11/27/updated-guide-to-the-rtklib-configuration-file/
