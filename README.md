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
