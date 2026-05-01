import argparse
import asyncio
import curses
import time
from math import pi

import can

from openarm.bus import Bus
from .config import MOTOR_CONFIGS
from .detect import detect_motors
from .encoding import ControlMode, MitControlParams
from .motor import Motor


OPEN_DEG = -160
CLOSE_DEG = 0

KP = 0.1
KD = 0.04
TAU = 0.8

STREAM_HZ = 1000
DT = 1.0 / STREAM_HZ

UI_REFRESH = 0.05


def deg_to_rad(deg: float):
    return deg * pi / 180


def rad_to_deg(rad: float):
    return rad * 180 / pi


async def setup_motor(can_bus):
    slave_ids = [config.slave_id for config in MOTOR_CONFIGS]
    detected = list(detect_motors(can_bus, slave_ids, timeout=0.1))

    if not detected:
        raise RuntimeError("No motors detected")

    info = detected[0]
    config = next(c for c in MOTOR_CONFIGS if c.slave_id == info.slave_id)

    bus = Bus(can_bus)

    motor = Motor(
        bus,
        slave_id=config.slave_id,
        master_id=config.master_id,
        motor_type=config.type,
    )

    await motor.enable()
    await motor.set_control_mode(ControlMode.MIT)

    return motor, config.name


async def motor_stream_loop(motor: Motor, state):
    """Always stream MIT command at 1000 Hz."""
    while True:

        target_rad = deg_to_rad(state["target"])

        params = MitControlParams(
            q=target_rad,
            dq=0,
            kp=KP,
            kd=KD,
            tau=TAU * state["tau_sign"],
        )

        motor_state = await motor.control_mit(params)

        if motor_state:
            state["pos"] = rad_to_deg(motor_state.position)
            state["vel"] = rad_to_deg(motor_state.velocity)
            state["temp"] = motor_state.temp_mos

        await asyncio.sleep(DT)


async def ui_loop(stdscr, state, name):
    curses.curs_set(0)
    stdscr.nodelay(True)

    while True:

        key = stdscr.getch()

        if key == ord("q"):
            break

        elif key == ord("o"):
            state["target"] = OPEN_DEG
            state["tau_sign"] = -1

        elif key == ord("c"):
            state["target"] = CLOSE_DEG
            state["tau_sign"] = 1

        stdscr.clear()

        stdscr.addstr(0, 0, "Motor Interactive Control")
        stdscr.addstr(1, 0, "-------------------------")

        stdscr.addstr(3, 0, f"Motor: {name}")

        stdscr.addstr(5, 0, f"Position : {state['pos']:8.2f} deg")
        stdscr.addstr(6, 0, f"Velocity : {state['vel']:8.2f} deg/s")
        stdscr.addstr(7, 0, f"Temp     : {state['temp']:8.2f} C")

        stdscr.addstr(9, 0, f"Target   : {state['target']} deg")
        stdscr.addstr(10, 0, f"Tau ff   : {TAU * state['tau_sign']:+.2f} Nm")

        stdscr.addstr(12, 0, "Controls:")
        stdscr.addstr(13, 0, "o → open motor")
        stdscr.addstr(14, 0, "c → close motor")
        stdscr.addstr(15, 0, "q → quit")

        stdscr.refresh()

        await asyncio.sleep(UI_REFRESH)


async def main(args):

    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    stdscr.nodelay(True)

    can_bus = can.Bus(channel=args.channel, interface=args.interface)

    state = {
        "pos": 0,
        "vel": 0,
        "temp": 0,
        "target": 0,  # default target always 0°
        "tau_sign": 0,
    }

    try:
        motor, name = await setup_motor(can_bus)

        stream_task = asyncio.create_task(motor_stream_loop(motor, state))

        try:
            await ui_loop(stdscr, state, name)
        finally:
            stream_task.cancel()
            curses.nocbreak()
            curses.echo()
            curses.endwin()

    finally:
        try:
            await motor.disable()
        except Exception:
            pass

        can_bus.shutdown()


def parse_arguments():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--channel",
        "-c",
        required=True,
        help="CAN channel",
    )

    parser.add_argument(
        "--interface",
        "-i",
        default="socketcan",
    )

    return parser.parse_args()


def run():
    args = parse_arguments()

    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    run()
