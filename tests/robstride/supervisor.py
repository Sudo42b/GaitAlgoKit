"""Example of moving a motor using the supervisor."""

import argparse
import math
import time

from motors.robstride.bindings import RobstrideMotorsSupervisor
# from actuator import RobstrideMotorsSupervisor


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port-name", type=str, default="/dev/ttyUSB0")
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--motor-type", type=str, default="01")
    parser.add_argument("--sleep", type=float, default=0.00)
    parser.add_argument("--period", type=float, default=2.5)
    parser.add_argument("--amplitude", type=float, default=1)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    amplitude = args.amplitude
    period = args.period

    supervisor = RobstrideMotorsSupervisor(args.port_name, {args.motor_id: args.motor_type})

    supervisor.set_kp(args.motor_id, 10)
    supervisor.set_kd(args.motor_id, 1)

    start_time = time.time()

    try:
        while True:
            elapsed_time = time.time() - start_time
            target_position = amplitude * math.sin(2 * math.pi / period * elapsed_time)
            # 라디안을 각도로
            angle = target_position * 180 / math.pi
            print(f"target_position: {target_position:.2f}, angle: {angle:.2f}")
            target_velocity = amplitude * 2 * math.pi / period * math.cos(2 * math.pi / period)
            supervisor.set_position(args.motor_id, target_position)
            supervisor.set_velocity(args.motor_id, target_velocity)
            time.sleep(args.sleep)
            if args.verbose:
                feedback = supervisor.get_latest_feedback()
                print(feedback)
    except KeyboardInterrupt:
        supervisor.stop()
        time.sleep(0.1)
        supervisor.add_motor_to_zero(args.motor_id)
        raise


if __name__ == "__main__":
    # python -m examples.supervisor
    main()
