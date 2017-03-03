# ros-crystalfontz
ROS node and drivers for crystalfontz displays

TODO

Installation:

* Execute the file ros-crystalfontz/bash/install.sh:
```
roscd crystalfontz/bash/ && sudo ./install.sh
```

This creates the file /etc/udev/rules.d/99-crystalfontz.rules containing

SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="fc0d", SYMLINK+="crystalfontz", MODE="0666"
