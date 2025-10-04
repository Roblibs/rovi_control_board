from Rosmaster_Lib import Rosmaster
import argparse
import time

def info():
    print("Hello from rovi-control-board!")
    bot = Rosmaster(com="/dev/my_ros_board")
    bot.create_receive_threading()
    version = bot.get_version()
    print(f"Version: {version}")
    voltage = bot.get_battery_voltage()
    print(f"Battery voltage: {voltage}V")
    del bot

def debug():
    print("Hello from rovi-control-board in debug mode!")
    bot = Rosmaster(com="/dev/my_ros_board", debug=True)
    bot.create_receive_threading()

    version = bot.get_version()
    print(f"Version: {version}")
    voltage = bot.get_battery_voltage()
    print(f"Battery voltage: {voltage}V")

    while True:
        time.sleep(1)
    del bot

def car_set(car_type):
    print("Hello from rovi-control-board in car_set mode!")
    bot = Rosmaster(com="/dev/my_ros_board")
    bot.create_receive_threading()

    # Read current car type
    current_type = bot.get_car_type_from_machine()
    print(f"Current car type: 0x{current_type:02X}")

    bot.set_car_type(car_type)
    time.sleep(0.1)

    # Verify change
    new_type = bot.get_car_type_from_machine()
    print(f"Requested car type: 0x{car_type:02X}")
    print(f"New car type readback: 0x{new_type:02X}")
    if new_type != car_type:
        print("Warning: car type did not update to requested value.")
    del bot

def car_get():
    print("Hello from rovi-control-board in car_get mode!")
    bot = Rosmaster(com="/dev/my_ros_board")
    bot.create_receive_threading()

    version = bot.get_version()
    print(f"Version: {version}")
    voltage = bot.get_battery_voltage()
    print(f"Battery voltage: {voltage}V")

    car_type_val = bot.get_car_type_from_machine()

    car_type_map = {
        0x00: "CAR_TYPE_NONE",
        0x01: "CAR_MECANUM (X3 small frame mecanum)",
        0x02: "CAR_MECANUM_MAX (X3 PLUS large frame large mecanum)",
        0x03: "CAR_MECANUM_MINI (large frame small mecanum)",
        0x04: "CAR_FOURWHEEL (X1 four-wheel)",
        0x05: "CAR_ACKERMAN (R2 Ackerman)",
        0x06: "CAR_SUNRISE (Sunrise Pi car)",
    }

    car_type_str = car_type_map.get(car_type_val, f"UNKNOWN (0x{car_type_val:02X})")
    print(f"Car type raw: {car_type_val} (0x{car_type_val:02X}) -> {car_type_str}")

    del bot

def cli():
    parser = argparse.ArgumentParser(description="Rovi control board CLI")
    parser.add_argument(
        "command",
        nargs="?",
        default="info",
        choices=["info", "help", "debug", "car_get", "car_set"],
        help="""
        - info: show version and battery voltage;
        - help: show Rosmaster help()
        - debug: show log of received data
        - car_get: get car status
        - car_set: set car type to 0x02 (CAR_MECANUM_MAX)
        """
    )
    args = parser.parse_args()
    if args.command == "car_set":
        # Hardcoded car type value (0x02)
        args.car_type = 0x02
    return args

def main():
    args = cli()
    if args.command == "help":
        bot = Rosmaster(com="/dev/my_ros_board")
        help(bot)
        del bot
        return
    if args.command == "info":
        info()
    if args.command == "debug":
        debug()
    if args.command == "car_get":
        car_get()
    if args.command == "car_set":
        car_set(args.car_type)

if __name__ == "__main__":
    main()

