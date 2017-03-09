# ros-crystalfontz
ROS driver and menu system for CFA-635 crystalfontz displays


Installation:
============================================================
* Clone the package into your catkin workspace
* Compile the package with
```
catkin_make
```
* Enter the directory of the ros-crystalfontz package and execute
```
./bash/install.sh
```
The last step creates a udev rule that assigns a predefined name (namely, "crystalfontz") to the display every time it is connected to the machine. Precisely, the install.sh script creates a file named */etc/udev/rules.d/99-crystalfontz.rules* containing
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="fc0d", SYMLINK+="crystalfontz", MODE="0666"
```


Generalities:
============================================================
The crystalfontz ROS package has been developed at the Artificial Intelligence and Robotics Lab of Politecnico di Milano university (http://airlab.ws.dei.polimi.it/index.php/AIRWiki).
The key components of the ros-crystalfontz package are the following ROS nodes:

** crystalfontz-driver **
Provides an interface between ROS-based software and a Crystalfontz display, based on the use of ROS topics. More specifically, crystalfontz-driver enables the programmer to use all the functionalities of the CFA-635 simply by publishing or reading messages on topics, thus completely removing the need to know and use the communication protocols.

** crystalfontz-logic **
Provides a menu-based system to run scripts using the keypad of the Crystalfontz display. An arbitrarily complex tree of submenus can be easily defined, simply by storing the scripts into a subdirectory tree having the same structure. Submenu names correspond to directory names in the tree.

The crystalfontz ROS package includes three ROS nodes:
  1. crystalfontz_driver (run with *roslaunch crystalfontz driver.launch*) 
  2. crystalfontz_logic (run with *roslaunch crystalfontz logic.launch*)
  3. crystalfontz_logic_test (a test node, run with *rosrun crystalfontz test*)

While crystalfontz_driver is necessary to interface the display with a ROS system, crystalfontz_logic is only needed if the programmer wants to implement a menu-based system to run scripts.

The crystalfontz_driver ROS node does the following:
  1. subscribes to ROS topics */crystalfontz/line_1*, */crystalfontz/line_2*, */crystalfontz/line_3*, */crystalfontz/line_4* and displays their contents on lines 1...4 (respectively) of the display
  2. subscribes to ROS topics */crystalfontz/led_1*, */crystalfontz/led_2*, */crystalfontz/led_3* and */crystalfontz/led_4* and applies the required color values to the LEDs on the left of the display
  3. subscribes to ROS topic */crystalfontz/contrast* and sets the contrast of the display to the specified value
  4. subscribes to ROS topic */crystalfontz/backlight_power* and sets the required backlight intensity
  5. publishes state changes of the onboard keys on ROS topic */crystalfontz/key_activity*

The crystalfontz_logic ROS node does the following:
  1. publishes menu-related messages on ROS topic */crystalfontz/line_4*
  2. reads button state changes from ROS topic */crystalfontz/key_activity*
  3. manages an onscreen menu system that the user can interact with using the onboard keypad: through this system the user can run bash scripts

Bash scripts accessible via the menu system can in turn be used to run any other command. 

After a script has been run via the keypad, it can be killed via the keypad as well (if needed). Script behavior when the user kills it via the keypad can vary. Two types of scripts, with different behaviors, have been provided as examples in this package:
* "propagating scripts" that, when they are killed via the keypad, also kill all their children processes (i.e. processes run by the script)
* "non-propagating scripts" that, when they are killed via the keypad, do not kill all their children processes (i.e. processes run by the script)
Users of crystalfontz_logic are of course free to define other types of script.

The crystalfontz_logic_test ROS node is a simple test node that can be used to check the crystalfontz package. This node shows test texts on the four lines of the display and applies random intensity values to the LEDs.


Configuration:
============================================================
The crystalfontz package contains two configuration files:
*  config/driver_default.yaml
*  config/logic_default.yaml

config/driver_default.yaml specifies the initial configuration of the CFA-635 device used when the crystalfontz_driver node is launched. Precisely, 
*  contrast: contrast value applied to the display (0 ... 254)
*  backlight_power: backlight intensity applied to the display (0 ... 100)
*  init_line_1...4: string displayed on lines 1...4
*  red_led_1...4: intensity of the red component for each of four onboard LEDS (0.0 ... 1.0)
*  green_led_1...4: intensity of the green component for each of the four onboard LEDS (0.0 ... 1.0)

config/logic_default.yaml contains only one parameter, namely
*  menu_folder: specifies the complete path of the folder defining the menu structure for node crystalfontz_logic (see section "Menu structure" below)


Topic contents:
============================================================
The payload of messages published on each of ROS topics */crystalfontz/line_1...4* is a string; messages use ROS message type *std_msgs::String*. Each character of the payload strings is displayed on the display, in the same position it appears in the string, on the display line (Line 1, ..., Line 4) corresponding to the topic name.
Display lines are 20 character long. Message strings can be longer than 20 characters: characters after the 20th are ignored. If the string is shorter than 20 characters, subsequent characters (up to the 20th) are filled with spaces.

An important exception is **character '\t'**. If a '\t' character is present in position X in the message string, it means that the character currently displayed in position X of the display line should be left unchanged. In this way, multiple nodes can use different portions of the same Line, publishing their messages asynchronously, without interfering.

Messages published on ROS topics */crystalfontz/led_1...4* use ROS message type *std_msgs::ColorRGBA*. These messages define the intensity of the Red and Green subLEDs of the four (bicolor) onboard LED featured by the CFA-635 display. Blue and Alpha values are ignored.

Messages published on ROS topic */crystalfontz/contrast* use ROS message type *std_msgs::int32*. These messages define the contrast of the LCD panel of the CFA-635 display.

Messages published on ROS topic */crystalfontz/backlight_power* use ROS message type *std_msgs::int32*. These messages define the backlight intensity of the CFA-635 display.

Messages published on ROS topic */crystalfontz/key_activity* is an 8-bit integer; messages use ROS message type *std_msgs::Int8*. Payload values correspond to PRESSED and RELEASED activities as described in the technical documentation of the Crystalfontz CFA-635 device. Possible values are listed in the following table:

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
The menu is used by the user to select the commands to execute. Submenus with an arbitrary depth can be defined.
Menu structure corresponds to the subdirectory structure of a **menu folder**: i.e., a directory tree where the scripts are stored. The location in the filesystem of such directory is specified by a parameter (see section "Configuration").

The menu folder is a tree whose leaves are the available scripts. Scripts must be executable bash scripts; their filenames must end in ".sh".
The directories and subdirectories of the menu folder define a menu that can be navigated using the keypad of the CFA-635. While navigating, at each instant one (and only one) of the items in the currently active (sub)menu is shown onscreen. Such item can be
* the name of a submenu of the current menu item, or
* the name of a script
If selected, such item causes (respectively) the following:
* the submenu becomes the currently active menu, or
* the script is run
Script names are displayed without the ".sh" extension.

This package includes an example of menu folder, called "example script folder" (note that blank spaces are part of the directory name). The menu folder includes two subdirectories (defining submenus) called "example scripts" and "no scripts". The first contains a *non-propagating script* (see section "Generalities" for script types) called "non-propagating.sh"; the second contains a *propagating script* called "propagating.sh".


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

