"""Example of moving a motor using the supervisor with WASD control."""

import argparse
import curses
import time

from bindings import RobstrideMotorsSupervisor


def main(stdscr: curses.window) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port-name", type=str, default="/dev/ttyUSB0")
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--motor-type", type=str, default="01")
    # parser.add_argument("--second-motor-id", type=int, default=2)
    # parser.add_argument("--second-motor-type", type=str, default="01")
    parser.add_argument("--zero-on-init", action="store_true")
    parser.add_argument("--max-update-rate", type=float, default=1000.0)
    parser.add_argument("--kd", type=float, default=1.0)
    parser.add_argument("--kp", type=float, default=10.0)
    args = parser.parse_args()

    supervisor = RobstrideMotorsSupervisor(
        port_name=args.port_name,
        motor_infos={
            args.motor_id: args.motor_type,
            # args.second_motor_id: args.second_motor_type,
        },
        target_update_rate=args.max_update_rate,
        zero_on_init=args.zero_on_init,
    )
    supervisor.add_motor_to_zero(args.motor_id)
    # supervisor.add_motor_to_zero(args.second_motor_id)

    position_motor_1 = 0.0  # Initial position for motor 1
    # position_motor_2 = 0.0  # Initial position for motor 2
    normal_step_size = 0.5  # Normal step size for position change
    # high_step_size = 1  # High step size for position change
    # low_step_size = 0.25  # Low step size for position change

    stdscr.nodelay(True)  # Make getch non-blocking
    stdscr.clear()

    supervisor.set_kd(args.motor_id, args.kd)
    supervisor.set_kp(args.motor_id, args.kp)
    # supervisor.set_kd(args.second_motor_id, args.kd)
    # supervisor.set_kp(args.second_motor_id, args.kp)

    try:
        while True:
            time.sleep(0.01)

            # Check for key presses
            key = stdscr.getch()

            if key == ord("a"):
                position_motor_1 -= normal_step_size  # Move motor 1 counter-clockwise
            elif key == ord("d"):
                position_motor_1 += normal_step_size  # Move motor 1 clockwise
            elif key == ord("w"):
                position_motor_2 += normal_step_size  # Move motor 2 clockwise
            elif key == ord("s"):
                position_motor_2 -= normal_step_size  # Move motor 2 counter-clockwise
            elif key == ord("q"):
                supervisor.stop()
                break

            # Set target position for both motors
            supervisor.set_position(args.motor_id, position_motor_1)
            # supervisor.set_position(args.second_motor_id, position_motor_2)

            feedback = supervisor.get_latest_feedback()
            stdscr.addstr(
                0,
                0,
                f"Motor 1 Position: {position_motor_1:.2f}, "
                # + f"Motor 2 Position: {position_motor_2:.2f}, "
                + f"Feedback: {feedback}",
            )
            stdscr.refresh()

    except KeyboardInterrupt:
        supervisor.stop()
        time.sleep(0.1)
        raise


if __name__ == "__main__":
    # python -m examples.wasd_control
    curses.wrapper(main)
