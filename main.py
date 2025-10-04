from Rosmaster_Lib import Rosmaster
import argparse

def info():
    print("Hello from rovi-control-board!")
    bot = Rosmaster(com="/dev/ttyUSB0")
    bot.create_receive_threading()
    version = bot.get_version()
    print(f"Version: {version}")
    voltage = bot.get_battery_voltage()
    print(f"Battery voltage: {voltage}V")
    del bot

def cli():
    parser = argparse.ArgumentParser(description="Rovi control board CLI")
    parser.add_argument(
        "command",
        nargs="?",
        default="info",
        choices=["info", "help"],
        help="info: show version and battery voltage; help: show Rosmaster help()"
    )
    return parser.parse_args()

def main():
    args = cli()
    if args.command == "help":
        bot = Rosmaster(com="/dev/ttyUSB0")
        help(bot)
        del bot
        return
    if args.command == "info":
        info()

if __name__ == "__main__":
    main()

