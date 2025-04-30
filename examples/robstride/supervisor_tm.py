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
    parser.add_argument("--second-motor-id", type=int, default=2)
    parser.add_argument("--second-motor-type", type=str, default="01")
    parser.add_argument("--sleep", type=float, default=0.00)
    
    parser.add_argument("--period", type=float, default=2.5)
    parser.add_argument("--amplitude", type=float, default=1.0)
    parser.add_argument("--target-angle", type=float, default=30.0)  # 목표 각도 (도 단위)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    supervisor = RobstrideMotorsSupervisor(
        port_name=args.port_name,
        motor_infos={
            args.motor_id: args.motor_type,
            args.second_motor_id: args.second_motor_type,
        },
        target_update_rate=1000.0,
        zero_on_init=True,
    )

    # 기본 제어 파라미터 설정
    supervisor.set_kp(args.motor_id, 10)
    supervisor.set_kd(args.motor_id, 1)
    
    supervisor.set_kp(args.second_motor_id, 10)
    supervisor.set_kd(args.second_motor_id, 2)
    
    
    # 2. 비대칭 진동 운동 시작
    start_time = time.time()
    
    try:
        while True:
            elapsed_time = time.time() - start_time
            
            # 비대칭 진동 (-1 ~ 2 라디안)
            # 중심점을 0.5로 설정하고 진폭을 1.5로 설정하여 -1 ~ 2 범위 생성
            center = 0.5  # 중심점
            amplitude = 1  # 진폭
            
            # 첫 번째 모터
            angle1 = center + amplitude * math.sin(elapsed_time * 2 * math.pi / args.period)
            velocity1 = amplitude * 2 * math.pi / args.period * math.cos(elapsed_time * 2 * math.pi / args.period)
            
            # 두 번째 모터 (반대 위상)
            angle2 = center + amplitude * math.sin(elapsed_time * 2 * math.pi / args.period + math.pi)
            velocity2 = amplitude * 2 * math.pi / args.period * math.cos(elapsed_time * 2 * math.pi / args.period + math.pi)
            
            # 위치와 속도 설정
            supervisor.set_position(args.motor_id, angle1)
            supervisor.set_velocity(args.motor_id, velocity1)
            
            supervisor.set_position(args.second_motor_id, angle2)
            supervisor.set_velocity(args.second_motor_id, velocity2)
            
            time.sleep(args.sleep)
            
            
            feedbacks = supervisor.get_latest_feedback()
            for motor_id, feedback in feedbacks.items():
                print(f"모터 {motor_id}:")
                print(f"  현재 각도: {(feedback.position * 180 / math.pi):.1f}도")
                print(f"  현재 속도: {feedback.velocity:.2f} rad/s")
                print(f"  현재 토크: {feedback.torque:.2f}")
                    
    except KeyboardInterrupt:
        supervisor.stop()
        time.sleep(0.1)
        supervisor.add_motor_to_zero(args.motor_id)
        supervisor.add_motor_to_zero(args.second_motor_id)
        raise


if __name__ == "__main__":
    # python -m examples.supervisor
    main()
