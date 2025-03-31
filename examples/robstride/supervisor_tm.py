"""Example of moving a motor using the supervisor."""

import argparse
import math
import time

from .motors.robstride.bindings import RobstrideMotorsSupervisor
# from actuator import RobstrideMotorsSupervisor


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port-name", type=str, default="/dev/ttyUSB0")
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--motor-type", type=str, default="01")
    parser.add_argument("--second-motor-id", type=int, default=2)
    parser.add_argument("--second-motor-type", type=str, default="01")
    parser.add_argument("--sleep", type=float, default=0.00)
    parser.add_argument("--period", type=float, default=2.5)
    parser.add_argument("--amplitude", type=float, default=1.0)
    parser.add_argument("--zero-on-init", action="store_true")
    parser.add_argument("--max-update-rate", type=float, default=1000.0)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    amplitude = args.amplitude
    period = args.period

    supervisor = RobstrideMotorsSupervisor(port_name=args.port_name,
                                           motor_infos={
            args.motor_id: args.motor_type,
            args.second_motor_id: args.second_motor_type,
        },
        target_update_rate=args.max_update_rate,
        zero_on_init=args.zero_on_init,
                                           )

    supervisor.set_kp(args.motor_id, 10)
    supervisor.set_kd(args.motor_id, 1)
    supervisor.set_kd(args.second_motor_id, 10)
    supervisor.set_kp(args.second_motor_id, 1)
    start_time = time.time()

    try:
        while True:
            elapsed_time = time.time() - start_time
            target_position = amplitude * math.sin(elapsed_time * 2 * math.pi / period)
            target_velocity = amplitude * 2 * math.pi / period * math.cos(elapsed_time * 2 * math.pi / period)
            supervisor.set_position(args.motor_id, target_position)
            supervisor.set_position(args.second_motor_id, target_position)
            supervisor.set_velocity(args.motor_id, target_velocity)
            supervisor.set_velocity(args.second_motor_id, target_velocity)
            time.sleep(args.sleep)
            if args.verbose:
                feedback = supervisor.get_latest_feedback()
                print(feedback)
    except KeyboardInterrupt:
        supervisor.stop()
        time.sleep(0.1)
        supervisor.add_motor_to_zero(args.motor_id)
        supervisor.add_motor_to_zero(args.second_motor_id)
        raise


if __name__ == "__main__":
    # python -m examples.supervisor
    main()
