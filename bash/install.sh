UDEV_RULE_FILE=/etc/udev/rules.d/99-crystalfontz.rules
if [ -e $UDEV_RULE_FILE ];
	then echo "The file "$UDEV_RULE_FILE" already exists; Doing nothing.";
	else
		echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="fc0d", SYMLINK+="crystalfontz", MODE="0666"' >> $UDEV_RULE_FILE;
		echo "Created file $UDEV_RULE_FILE."
fi
