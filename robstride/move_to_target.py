"""현재 각도 확인 후 목표 각도로 이동"""

import argparse
import math
import time

from motors.robstride.bindings import RobstrideMotorsSupervisor

def get_current_angles(supervisor: RobstrideMotorsSupervisor, max_attempts: int = 5) -> dict:
    """모터의 현재 각도를 확인합니다."""
    for attempt in range(max_attempts):
        feedbacks = supervisor.get_latest_feedback()
        if feedbacks:
            current_angles = {}
            for motor_id, feedback in feedbacks.items():
                angle_rad = feedback.position
                angle_deg = angle_rad * 180 / math.pi
                current_angles[motor_id] = {
                    'radian': angle_rad,
                    'degree': angle_deg
                }
            return current_angles
        time.sleep(0.1)
    return None

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port-name", type=str, default="/dev/ttyUSB0")
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--motor-type", type=str, default="01")
    parser.add_argument("--second-motor-id", type=int, default=2)
    parser.add_argument("--second-motor-type", type=str, default="01")
    args = parser.parse_args()

    print("모터 초기화 중...")
    supervisor = RobstrideMotorsSupervisor(
        port_name=args.port_name,
        motor_infos={
            args.motor_id: args.motor_type,
            args.second_motor_id: args.second_motor_type,
        },
        target_update_rate=1000.0,
        zero_on_init=True  # 영점 조정을 하지 않음
    )
    """
        위치 오차 = 목표 위치 - 현재 위치
        속도 오차 = 목표 속도 - 현재 속도
        제어 토크 = kp * 위치 오차 + kd * 속도 오차
        출력 토크 = kp * (목표위치 - 현재위치) + kd * (목표속도 - 현재속도) + 초기토크
        
        Robstride 01모터의 경우
        
        초기 속도는 0.0
        v_min = -44.0
        v_max = 44.0
        초기 피드백 제어 파라미터
        kd(속도 제어 게인)는 0.0
        kd_min=0.0 ()
        kd_max=5.0 
        kp(위치 제어 게인)는 0.0
        kp_min=0.0
        kp_max=500.0
        초기토크는 0.0
        t_min = -12.0
        t_max = 12.0
    """
    # 제어 게인 설정
    supervisor.set_kp(args.motor_id, 10)
    supervisor.set_kd(args.motor_id, 1)
    supervisor.set_kp(args.second_motor_id, 10)
    supervisor.set_kd(args.second_motor_id, 1)

    try:
        # 현재 각도 확인
        print("\n현재 각도 확인 중...")
        current_angles = get_current_angles(supervisor)
        
        if current_angles:
            print("\n현재 각도:")
            for motor_id, angles in current_angles.items():
                print(f"모터 {motor_id}:")
                print(f"  라디안: {angles['radian']:.2f} rad")
                print(f"  각도: {angles['degree']:.2f}°")
        else:
            print("현재 각도를 확인할 수 없습니다.")
            supervisor.stop()
            return

        # 목표 각도 설정 (45도)
        target_angle = 45.0 
        target_rad = target_angle * math.pi / 180
        print(f"\n목표 각도로 이동 중: {target_rad:.2f} rad ({(target_angle * 180 / math.pi):.2f}°)")
        
        # 두 모터를 목표 각도로 이동
        supervisor.set_position(args.motor_id, target_rad)
        supervisor.set_position(args.second_motor_id, target_rad)

        # 모터가 목표 위치에 도달할 때까지 모니터링
        print("\n각도 모니터링 중...")
        while True:
            feedbacks = supervisor.get_latest_feedback()
            print("\n현재 각도 정보:")
            all_motors_reached = True
            
            for motor_id, feedback in feedbacks.items():
                current_rad = feedback.position
                current_deg = current_rad * 180 / math.pi
                error = abs(current_rad - target_rad)
                
                print(f"모터 {motor_id}:")
                print(f"  현재 각도: {current_rad:.3f} rad ({current_deg:.2f}°)")
                print(f"  목표까지 차이: {error:.3f} rad")
                
                # 오차가 0.05 라디안(약 2.9도) 이상이면 아직 도달하지 않은 것으로 판단
                if error > 0.01:
                    all_motors_reached = False

            if all_motors_reached:
                print("\n모든 모터가 목표 위치에 도달했습니다!")
                break
                
            time.sleep(0.1)  # 0.1초 간격으로 확인

    except KeyboardInterrupt:
        print("\n프로그램 종료")
    finally:
        supervisor.stop()

if __name__ == "__main__":
    main()