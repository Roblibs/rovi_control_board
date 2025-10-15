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

def set_wheel_motor(speed):
    print("Hello from rovi-control-board in wheel motor test mode!")
    bot = Rosmaster(com="/dev/my_ros_board")
    bot.create_receive_threading()

    voltage = bot.get_battery_voltage()
    print(f"Battery voltage: {voltage}V")

    print(f"Setting wheel motors to {speed}% for 4 seconds...")
    bot.set_motor(speed, speed, speed, speed)
    time.sleep(1)

    print("Stopping wheel motors...")
    bot.set_motor(0, 0, 0, 0)

    del bot

def set_wheel_motor_ramp(speed, ramp_time=4, plateau_time=1):
    print("Hello from rovi-control-board in wheel motor ramp test mode!")
    bot = Rosmaster(com="/dev/my_ros_board")
    bot.create_receive_threading()

    voltage = bot.get_battery_voltage()
    print(f"Battery voltage: {voltage}V")

    # Validate ramp_time
    if ramp_time <= 0:
        print("Invalid ramp_time; using 0.1s")
        ramp_time = 0.1

    # Build ramp points (10% steps) and ensure we end exactly at target speed
    points = list(range(0, speed + 1, 10))
    if not points or points[-1] != speed:
        points.append(speed)

    step_time = ramp_time / len(points)
    print(f"Ramping wheel motors to {speed}% over {ramp_time} seconds...")
    for s in points:
        print(f"  Setting wheel motors to {s}%")
        bot.set_motor(s, s, s, s)
        time.sleep(step_time)

    print(f"Holding at {speed}% for {plateau_time} seconds...")
    time.sleep(plateau_time)

    print("Stopping wheel motors...")
    bot.set_motor(0, 0, 0, 0)

    del bot

def set_car_run(speed):
    print("Hello from rovi-control-board in car run mode!")
    bot = Rosmaster(com="/dev/my_ros_board")
    bot.create_receive_threading()

    voltage = bot.get_battery_voltage()
    print(f"Battery voltage: {voltage}V")

    print(f"Setting car to run forward at {speed}% for 4 seconds...")
    state_forward = 0x01  # Forward
    bot.set_car_run(state=state_forward, speed=speed)
    time.sleep(4)

    print("Stopping car...")
    state_stop = 0x00  # Stop
    bot.set_car_run(state=state_stop, speed=0)

    del bot

def cli():
    parser = argparse.ArgumentParser(description="Rovi control board CLI")

    def _speed_arg(v: str) -> int:
        iv = int(v)
        if iv < 0 or iv > 100:
            raise argparse.ArgumentTypeError("speed must be 0-100")
        return iv

    parser.add_argument(
        "command",
        nargs="?",
        default="info",
        help="""
        - info: show version and battery voltage;
        - help: show Rosmaster help()
        - debug: show log of received data
        - car_get: get car status
        - car_set: set car type to 0x02 (CAR_MECANUM_MAX)
        - wheel <speed>: set wheel motors to <speed> percentage (0-100)
        """
    )
    # First parse known args to determine the command without failing on extra args
    known_args, _ = parser.parse_known_args()
    if known_args.command in ["wheel", "ramp", "run"]:
        parser.add_argument(
            "speed",
            type=_speed_arg,
            help="Wheel speed percentage (0-100) for 'wheel' command."
        )
    args = parser.parse_args()
    if args.command == "car_set":
        # Hardcoded car type value (0x02)
        args.car_type = 0x02
    return args

def set_car_motion(velocity):
    print("Hello from rovi-control-board in car motion mode!")
    bot = Rosmaster(com="/dev/my_ros_board")
    bot.create_receive_threading()

    voltage = bot.get_battery_voltage()
    print(f"Battery voltage: {voltage}V")

    print(f"Setting car to move at velocity {velocity} m/s for 4 seconds...")
    vy = velocity
    bot.set_car_motion(0, -vy, 0)  # 0 for angular velocity
    time.sleep(4)

    print("Stopping car...")
    bot.set_car_motion(0, 0, 0)

    del bot

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
    if args.command == "wheel":
        set_wheel_motor(args.speed)
    if args.command == "ramp":
        set_wheel_motor_ramp(args.speed, ramp_time=6, plateau_time=4)
    if args.command == "run":
        set_car_run(args.speed)
    if args.command == "motion":
        set_car_motion(0.2)

if __name__ == "__main__":
    main()

