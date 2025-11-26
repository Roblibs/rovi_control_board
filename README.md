# rovi_control_board
Tests for the Yahboom ROS Control Board used by the Room View Bot

- using repo https://github.com/Roblibs/Rosmaster_Lib

# Testing
## quick test
list devices and add acces rights with `sudo chmod a+rw /dev/ttyUSB0`

## adding rule
run script from https://github.com/Roblibs/Rosmaster_Lib/blob/V3.3.9/add_rule.sh

```shell
sudo ./add_rule.sh my_ros_board
```

the run
```shell
uv run main.py
```

joystick discovery
```shell
sudo apt update
sudo apt install bluetooth bluez bluez-tools python3-evdev
bluetoothctl
#> power on
#> agent on
#> default-agent
#> scan on
#> [NEW] Device XX:XX:XX:XX:XX:XX Wireless Controller
#> pair XX:XX:XX:XX:XX:XX
#> trust XX:XX:XX:XX:XX:XX
#> connect XX:XX:XX:XX:XX:XX
#> quit
ls /dev/input/
sudo apt install joystick
jstest /dev/input/js0

sudo apt install python3-pygame
```

# Influx Time series db
subscribe to topics and post on influxdb

```shell
uv run tsdb/ros_to_db.py
```
