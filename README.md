# ros-crystalfontz
ROS node and drivers for crystalfontz displays


Installation:
============================================================

* Execute the file ros-crystalfontz/bash/install.sh:
```
roscd crystalfontz/bash/ && sudo ./install.sh
```

This creates the file /etc/udev/rules.d/99-crystalfontz.rules containing

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="fc0d", SYMLINK+="crystalfontz", MODE="0666"
```

Generalities:
============================================================
The crystalfontz ROS package includes three ROS nodes:
  1. crystalfontz_driver (roslaunch crystalfontz driver.launch) 
  2. crystalfontz_logic (roslaunch crystalfontz logic.launch)
  3. crystalfontz_logic_test (rosrun crystalfontz test)

The crystalfontz_driver ROS node does the following:
  1. subscribes to the topics line_1, line_2, line_3, line_4 and displays their contents on lines 1...4 of the display
  2. subscribes to the topics led_1, led_2, led_3 AND led_4 and applies the required color values to the LEDs on the side of the display
  3. publishes every change of state of the display keys on the topic key_activity

The crystalfontz_logic ROS node does the following:
  1. generates and publishes messages on topic line_4
  2. generates and publishes LED status on topics led_1, ..., led_4
  3. reads button state changes from the topic key_activity
  4. manages an onscreen menu using which the user can execute commands (executing a command corresponds to running a script)

The crystalfontz_logic_test ROS node publishes test text on the line topics and random values on the led topics.

Configuration:
============================================================
The crystalfontz ROS package contains two configuration files:
The driver configuration (config/driver_default.yaml) specifies the values displayed when the crystalfontz_driver node is launched.
*  contrast: The contrast used when the node is launched (0 ... 254)
*  backlight_power: The backlight power used when the node is launched (0 ... 100)
*  init_line_x: the string displayed on line x when the node is launched
*  red_led_x, green_led_x: the RG values to which the led are set when the node is launched (0.0 ... 1.0)

The logic configuration (config/logic_default.yaml).
*  menu_folder: specifies the complete path of the folder containing the scripts.

Topic contents:
============================================================
The payload of messages published on each of the topics line_1, ..., line_4 is a string (std_msgs::String). The characters of such strings will be displayed on the display, in the corresponding position, on the line (Line 1, ..., Line 4) corresponding to the topic.
The string can be longer than 20 characters and the remaining characters are ignored.
If the string is shorter than 20 characters, the string is filled with spaces.

An important exception is control character '\t', which -if present in position X- means that the character currently displayed in position X of the Line should not be changed. In this way, multiple nodes can use different portions of the same Line, publishing their messages asynchronously, without interfering.

The payload of messages published on each of the topics led_1, ..., led_4 is std_msgs::ColorRGBA. These contain the intensity of the Red and Green components of the LED, Blue and Alpha values are ignored.

The payload of messages published on the topics key_activity is a short int (std_msgs::Int8). The integer values correspond to PRESSED and RELEASED activities as described in the CFA635-xxx-KU_Data_Sheet documentation:

| Key activity | Value |
| ------ | ------ |
| KEY_UP_PRESS | 1 |
| KEY_DOWN_PRESS | 2 |
| KEY_LEFT_PRESS | 3 |
| KEY_RIGHT_PRESS | 4 |
| KEY_ENTER_PRESS | 5 |
| KEY_EXIT_PRESS | 6 |
| KEY_UP_RELEASE | 7 |
| KEY_DOWN_RELEASE | 8 |
| KEY_LEFT_RELEASE | 9 |
| KEY_RIGHT_RELEASE | 10 |
| KEY_ENTER_RELEASE | 11 |
| KEY_EXIT_RELEASE | 12 |


Menu structure:
============================================================
The menu is used by the user to select the commands to execute. 
It is a tree whose leaves are the available scripts. The scripts file name must end in ".sh".
Each menu item can be a submenu or a command. Executing a command corresponds to running a script.

The menu is defined by a directory structure where:
  - root directory is called "menu"
  - submenus correspond to subdirectories
  - commands correspond to script files

An example of menu folder present in this package is:

"example script folder/example scripts/not propagating.sh"

"example script folder/example scripts/propagating.sh"

"example script folder/no scripts in here/stdout.txt"

NOTES: Command names are the names of the corresponding scripts, displayed without the ".sh" extension.


Keys
============================================================
The keys on the side of the display are used to interact with the menu.
Their usage is the following:

| Key | Action |
| ------ | ------ |
| ENTER | Enters menu mode and, when a command name is onscreen runs the corresponding script. |
| EXIT | When in menu mode, exits menu mode; When running a script, the script is terminated. |
| UP, DOWN | Scroll through menu items available at current level |
| RIGHT | if a submenu name is onscreen, enters it |
| LEFT | goes to upper level submenu |

In menu mode, active keys are shown on the right of line 4 of the display.


Menu behaviour when running scripts
============================================================
After a script is run using the menu, the script can be terminated by the menu system by pressing X. When the user presses the X button, SIGTERM is sent to the process running the script.
Two different kinds of bash scripts are suggested and are present in the source as examples: "example script folder/example scripts/not propagating.sh" and "example script folder/example scripts/propagating.sh".
The former simply executes some commands and, when SIGTERM is received, the script terminates leaving the child processes alive. The latter propagates the SIGTERM to the children.

While running scripts via the menu system:
1. the menu system writes "Running XXX ..." on Line 4
2. the menu system does not update Line 4 any more until the script terminates
3. the script should use a line different from Line 4 for its output
4. the menu system stops terminates the script when the user presses the X key; the menu return to the last displayed entry


Display usage
============================================================
The display subdivided into 4 Lines of 20 characters each. Each line can be addressed separately.

Lines 1, 2 and 3 of the display are left free for usage by ROS nodes. The programmer is responsible for avoiding conflicts among messages generated by different ROS nodes. As already explained (see Topic Contents), it is possible for multiple nodes to share the same Line without conflicts (provided that nodes use different subsets of the 1-20 character range).

NOTE: Line 4 of the display is available only when crystalfontz_logic is not used.

When menu mode is not active, line 4 shows "press OK for menu...".
When menu mode is active, line 4 shows the current menu item. The right area is used to show the available navigation options, corresponding to active keys (left, right, up, down, enter amd exit).
Options are only shown when available.

