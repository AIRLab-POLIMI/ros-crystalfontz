# ros-crystalfontz
ROS node and drivers for crystalfontz displays

TODO

Installation:

* create file /etc/udev/rules.d/98-crystalfontz.rules containing

SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="fc0d", SYMLINK+="crystalfontz", MODE="0666"
