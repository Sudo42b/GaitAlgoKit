#!/usr/bin/env python3
"""
모터 제어 시나리오 테스트 스크립트 - 자연스러운 전환 구현
"""

import argparse
import time
import math
import logging
from typing import Dict, List, Optional, Tuple
from motors.robstride.can_communication import MotorType
from motors.robstride.bindings import (
    RobstrideMotorsSupervisor,
    RobstrideMotorControlParams,
    RobstrideMotorFeedback
)

def calculate_motor_positions(elapsed_time: float, period: float, amplitude: float, phase_right: float = 0, phase_left: float = math.pi, 
                             right_scale: float = 1.0, left_scale: float = 1.0) -> Tuple[float, float, float, float]:
    """
    시간에 따른 모터 위치 및 속도 계산
    
    Args:
        elapsed_time: 경과 시간
        period: 주기
        amplitude: 진폭
        phase_right: 오른쪽 다리 위상
        phase_left: 왼쪽 다리 위상
        right_scale: 오른쪽 진폭 스케일 (부상 시뮬레이션용)
        left_scale: 왼쪽 진폭 스케일 (부상 시뮬레이션용)
        
    Returns:
        right_pos, right_vel, left_pos, left_vel
    """
    # 오른쪽 다리
    right_pos = amplitude * right_scale * math.sin(2 * math.pi / period * elapsed_time + phase_right)
    right_vel = amplitude * right_scale * (2 * math.pi / period) * math.cos(2 * math.pi / period * elapsed_time + phase_right)
    
    # 왼쪽 다리
    left_pos = amplitude * left_scale * math.sin(2 * math.pi / period * elapsed_time + phase_left)
    left_vel = amplitude * left_scale * (2 * math.pi / period) * math.cos(2 * math.pi / period * elapsed_time + phase_left)
    
    return right_pos, right_vel, left_pos, left_vel

def transition_positions(superior: RobstrideMotorsSupervisor, 
                         start_right_pos: float, start_left_pos: float,
                         end_right_pos: float, end_left_pos: float,
                         duration: float = 0.5) -> None:
    """
    시작 위치에서 종료 위치로 부드럽게 전환
    
    Args:
        superior: 모터 관리자
        start_right_pos: 오른쪽 다리 시작 위치
        start_left_pos: 왼쪽 다리 시작 위치
        end_right_pos: 오른쪽 다리 종료 위치
        end_left_pos: 왼쪽 다리 종료 위치
        duration: 전환 시간 (초)
    """
    print(f"위치 전환 시작: 오른쪽 {start_right_pos:.2f} -> {end_right_pos:.2f}, 왼쪽 {start_left_pos:.2f} -> {end_left_pos:.2f}")
    
    start_time = time.time()
    while True:
        elapsed = time.time() - start_time
        if elapsed >= duration:
            break
            
        # 선형 보간
        t = elapsed / duration  # 0에서 1 사이의 값
        
        # 부드러운 전환을 위한 easing 함수 (사인 기반)
        smooth_t = (1 - math.cos(t * math.pi)) / 2
        
        # 현재 위치 계산
        right_pos = start_right_pos + (end_right_pos - start_right_pos) * smooth_t
        left_pos = start_left_pos + (end_left_pos - start_left_pos) * smooth_t
        
        # 속도 계산 (이동 방향에 맞게)
        right_vel = (end_right_pos - start_right_pos) * math.pi/2 * math.sin(t * math.pi) / duration
        left_vel = (end_left_pos - start_left_pos) * math.pi/2 * math.sin(t * math.pi) / duration
        
        # 위치 및 속도 설정
        superior.set_position(1, right_pos)
        superior.set_velocity(1, right_vel)
        
        superior.set_position(2, left_pos)
        superior.set_velocity(2, left_vel)
        
        right_angle = right_pos * 180 / math.pi
        left_angle = left_pos * 180 / math.pi
        
        print(f"전환 중: right_pos: {right_pos:.2f}({right_angle:.2f}°), left_pos: {left_pos:.2f}({left_angle:.2f}°)")
        time.sleep(0.01)  # 서보 업데이트 시간 확보
    
    # 최종 위치 설정 확인
    superior.set_position(1, end_right_pos)
    superior.set_position(2, end_left_pos)
    print("위치 전환 완료")

def fast_walk(superior: RobstrideMotorsSupervisor, period: float = 1.0, amplitude: float = 1.0, duration: float = 3.0) -> Tuple[float, float]:
    """
    빠른 걷기 시나리오
    
    Returns:
        마지막 오른쪽/왼쪽 다리 위치
    """
    print(f"빠른 걷기 시작 (주기: {period}초, 진폭: {amplitude}, 지속시간: {duration}초)")
    start_time = time.time()
    last_right_pos = 0
    last_left_pos = 0
    
    try:
        while time.time() - start_time < duration:
            elapsed_time = time.time() - start_time
            
            # 위치 및 속도 계산
            right_pos, right_vel, left_pos, left_vel = calculate_motor_positions(
                elapsed_time, period, amplitude, 0, math.pi, 1.0, 1.0
            )
            
            # 모터에 위치 및 속도 설정
            superior.set_position(1, right_pos)
            superior.set_velocity(1, right_vel)
            
            superior.set_position(2, left_pos)
            superior.set_velocity(2, left_vel)
            
            right_angle = right_pos * 180 / math.pi
            left_angle = left_pos * 180 / math.pi
            
            print(f"빠른 걷기: right: {right_pos:.2f}({right_angle:.2f}°), left: {left_pos:.2f}({left_angle:.2f}°)")
            
            # 마지막 위치 저장
            last_right_pos = right_pos
            last_left_pos = left_pos
            
            time.sleep(0.01)  # CPU 부하 방지
            
    except Exception as e:
        print(f"빠른 걷기 오류: {e}")
        raise
    finally:
        print("빠른 걷기 완료")
    
    return last_right_pos, last_left_pos

def slow_walk(superior: RobstrideMotorsSupervisor, period: float = 2.0, amplitude: float = 1.0, 
              duration: float = 3.0, start_right: float = None, start_left: float = None) -> Tuple[float, float]:
    """
    느린 걷기 시나리오
    
    Returns:
        마지막 오른쪽/왼쪽 다리 위치
    """
    print(f"느린 걷기 시작 (주기: {period}초, 진폭: {amplitude}, 지속시간: {duration}초)")
    
    # 시작 위치가 주어진 경우, 전환 수행
    if start_right is not None and start_left is not None:
        # 첫 위치 계산 (느린 걷기의 시작 위치)
        init_right, _, init_left, _ = calculate_motor_positions(0, period, amplitude, 0, math.pi, 1.0, 1.0)
        
        # 부드러운 전환
        transition_positions(superior, start_right, start_left, init_right, init_left, 0.5)
    
    start_time = time.time()
    last_right_pos = 0
    last_left_pos = 0
    
    try:
        while time.time() - start_time < duration:
            elapsed_time = time.time() - start_time
            
            # 위치 및 속도 계산
            right_pos, right_vel, left_pos, left_vel = calculate_motor_positions(
                elapsed_time, period, amplitude, 0, math.pi, 1.0, 1.0
            )
            
            # 모터에 위치 및 속도 설정
            superior.set_position(1, right_pos)
            superior.set_velocity(1, right_vel)
            
            superior.set_position(2, left_pos)
            superior.set_velocity(2, left_vel)
            
            right_angle = right_pos * 180 / math.pi
            left_angle = left_pos * 180 / math.pi
            
            print(f"느린 걷기: right: {right_pos:.2f}({right_angle:.2f}°), left: {left_pos:.2f}({left_angle:.2f}°)")
            
            # 마지막 위치 저장
            last_right_pos = right_pos
            last_left_pos = left_pos
            
            time.sleep(0.01)  # CPU 부하 방지
            
    except Exception as e:
        print(f"느린 걷기 오류: {e}")
        raise
    finally:
        print("느린 걷기 완료")
    
    return last_right_pos, last_left_pos

def right_leg_injury(superior: RobstrideMotorsSupervisor, period: float = 2.0, amplitude: float = 1.0, 
                    duration: float = 3.0, start_right: float = None, start_left: float = None) -> Tuple[float, float]:
    """
    오른쪽 다리 부상 시나리오
    
    Returns:
        마지막 오른쪽/왼쪽 다리 위치
    """
    print(f"오른쪽 다리 부상 시뮬레이션 시작 (주기: {period}초, 진폭: {amplitude}, 지속시간: {duration}초)")
    
    # 부상 상황에서의 오른쪽 다리 스케일 (더 작은 움직임)
    right_scale = 0.25
    left_scale = 1.0
    
    # 시작 위치가 주어진 경우, 전환 수행
    if start_right is not None and start_left is not None:
        # 첫 위치 계산 (오른쪽 다리 부상의 시작 위치)
        init_right, _, init_left, _ = calculate_motor_positions(0, period, amplitude, 0, math.pi, right_scale, left_scale)
        
        # 부드러운 전환
        transition_positions(superior, start_right, start_left, init_right, init_left, 0.5)
    
    start_time = time.time()
    last_right_pos = 0
    last_left_pos = 0
    
    try:
        while time.time() - start_time < duration:
            elapsed_time = time.time() - start_time
            
            # 위치 및 속도 계산 (오른쪽은 제한된 움직임)
            right_pos, right_vel, left_pos, left_vel = calculate_motor_positions(
                elapsed_time, period, amplitude, 0, math.pi, right_scale, left_scale
            )
            
            # 모터에 위치 및 속도 설정
            superior.set_position(1, right_pos)
            superior.set_velocity(1, right_vel)
            
            superior.set_position(2, left_pos)
            superior.set_velocity(2, left_vel)
            
            right_angle = right_pos * 180 / math.pi
            left_angle = left_pos * 180 / math.pi
            
            print(f"오른쪽 부상: right: {right_pos:.2f}({right_angle:.2f}°), left: {left_pos:.2f}({left_angle:.2f}°)")
            print(f"토크: {superior.get_torque(1):.2f}, {superior.get_torque(2):.2f}")
            
            # 마지막 위치 저장
            last_right_pos = right_pos
            last_left_pos = left_pos
            
            time.sleep(0.01)  # CPU 부하 방지
            
    except Exception as e:
        print(f"오른쪽 다리 부상 오류: {e}")
        raise
    finally:
        print("오른쪽 다리 부상 시뮬레이션 완료")
    
    return last_right_pos, last_left_pos

def left_leg_injury(superior: RobstrideMotorsSupervisor, period: float = 2.0, amplitude: float = 1.0, 
                   duration: float = 3.0, start_right: float = None, start_left: float = None) -> Tuple[float, float]:
    """
    왼쪽 다리 부상 시나리오
    
    Returns:
        마지막 오른쪽/왼쪽 다리 위치
    """
    print(f"왼쪽 다리 부상 시뮬레이션 시작 (주기: {period}초, 진폭: {amplitude}, 지속시간: {duration}초)")
    
    # 부상 상황에서의 왼쪽 다리 스케일 (더 작은 움직임)
    right_scale = 1.0
    left_scale = 0.25
    
    # 시작 위치가 주어진 경우, 전환 수행
    if start_right is not None and start_left is not None:
        # 첫 위치 계산 (왼쪽 다리 부상의 시작 위치)
        init_right, _, init_left, _ = calculate_motor_positions(0, period, amplitude, 0, math.pi, right_scale, left_scale)
        
        # 부드러운 전환
        transition_positions(superior, start_right, start_left, init_right, init_left, 0.5)
    
    start_time = time.time()
    last_right_pos = 0
    last_left_pos = 0
    
    try:
        while time.time() - start_time < duration:
            elapsed_time = time.time() - start_time
            
            # 위치 및 속도 계산 (왼쪽은 제한된 움직임)
            right_pos, right_vel, left_pos, left_vel = calculate_motor_positions(
                elapsed_time, period, amplitude, 0, math.pi, right_scale, left_scale
            )
            
            # 모터에 위치 및 속도 설정
            superior.set_position(1, right_pos)
            superior.set_velocity(1, right_vel)
            
            superior.set_position(2, left_pos)
            superior.set_velocity(2, left_vel)
            
            right_angle = right_pos * 180 / math.pi
            left_angle = left_pos * 180 / math.pi
            
            print(f"왼쪽 부상: right: {right_pos:.2f}({right_angle:.2f}°), left: {left_pos:.2f}({left_angle:.2f}°)")
            print(f"토크: {superior.get_torque(1):.2f}, {superior.get_torque(2):.2f}")
            
            # 마지막 위치 저장
            last_right_pos = right_pos
            last_left_pos = left_pos
            
            time.sleep(0.01)  # CPU 부하 방지
            
    except Exception as e:
        print(f"왼쪽 다리 부상 오류: {e}")
        raise
    finally:
        print("왼쪽 다리 부상 시뮬레이션 완료")
    
    return last_right_pos, last_left_pos

def safe_stop(superior: RobstrideMotorsSupervisor, 
              current_right: float = 0, current_left: float = 0) -> None:
    """안전하게 원점으로 복귀 후 정지"""
    print("안전하게 모터 정지...")
    
    # 현재 위치에서 원점으로 부드럽게 전환
    transition_positions(superior, current_right, current_left, 0, 0, 1.0)
    
    # 모터 정지
    superior.add_motor_to_zero(1)
    superior.add_motor_to_zero(2)
    print("모터 정지 완료")

def main() -> None:
    parser = argparse.ArgumentParser(description="모터 제어 시나리오 테스트")
    parser.add_argument("-v", "--verbose", action="store_true", help="상세 출력 활성화")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="시리얼 포트 이름")
    parser.add_argument("--period", type=float, default=2.0, help="주기 (초)")
    parser.add_argument("--amplitude", type=float, default=1.0, help="진폭 (기본값: 1.0)")
    parser.add_argument("--kp", type=float, default=10.0, help="kp (기본값: 10.0)")
    parser.add_argument("--kd", type=float, default=1.0, help="kd (기본값: 1.0)")
    parser.add_argument("--motor-id", type=int, default=1, help="모터 ID (기본값: 1)")
    parser.add_argument("--motor-type", type=str, default="01", help="모터 타입 (기본값: 01)")
    parser.add_argument("--second-motor-id", type=int, default=2, help="모터 ID (기본값: 2)")
    parser.add_argument("--second-motor-type", type=str, default="01", help="모터 타입 (기본값: 01)")
    parser.add_argument("--duration", type=float, default=3.0, help="각 모드의 실행 시간 (초)")
    parser.add_argument("--cycles", type=int, default=1, help="전체 사이클 반복 횟수")
    
    args = parser.parse_args()
    
    # 로깅 설정
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    superior = RobstrideMotorsSupervisor(port_name=args.port, 
                                         motor_infos={args.motor_id: args.motor_type, 
                                                     args.second_motor_id: args.second_motor_type},
                                         target_update_rate=500,
                                         zero_on_init=True)
    
    # PD 제어 파라미터 설정
    superior.set_kp(args.motor_id, args.kp)
    superior.set_kd(args.motor_id, args.kd)
    
    superior.set_kp(args.second_motor_id, args.kp)
    superior.set_kd(args.second_motor_id, args.kd)
    
    try:
        # 각 시나리오를 순차적으로 실행하고 자연스럽게 전환
        for cycle in range(args.cycles):
            print(f"\n===== 사이클 {cycle+1}/{args.cycles} 시작 =====\n")
            
            # 빠른 걷기 시작 (처음 시작이므로 시작 위치 지정 없음)
            right_pos, left_pos = fast_walk(superior, period=args.period/2, amplitude=args.amplitude, duration=args.duration)
            
            # 느린 걷기로 전환 (이전 마지막 위치에서 시작)
            right_pos, left_pos = slow_walk(superior, period=args.period, amplitude=args.amplitude, 
                                           duration=args.duration, start_right=right_pos, start_left=left_pos)
            
            # 오른쪽 다리 부상으로 전환
            right_pos, left_pos = right_leg_injury(superior, period=args.period, amplitude=args.amplitude, 
                                                 duration=args.duration, start_right=right_pos, start_left=left_pos)
            
            # 왼쪽 다리 부상으로 전환
            right_pos, left_pos = left_leg_injury(superior, period=args.period, amplitude=args.amplitude, 
                                                duration=args.duration, start_right=right_pos, start_left=left_pos)
            
            print(f"\n===== 사이클 {cycle+1}/{args.cycles} 완료 =====\n")
        
        # 안전하게 정지
        safe_stop(superior, right_pos, left_pos)
            
    except KeyboardInterrupt:
        print("사용자에 의해 중단됨")
        # 마지막 위치에서 안전하게 정지
        try:
            right_pos = superior.get_position(args.motor_id)
            left_pos = superior.get_position(args.second_motor_id)
            safe_stop(superior, right_pos, left_pos)
        except:
            # 위치를 읽을 수 없는 경우 강제 정지
            superior.stop()
            superior.add_motor_to_zero(args.motor_id)
            superior.add_motor_to_zero(args.second_motor_id)
    except Exception as e:
        print(f"오류 발생: {e}")
        # 오류 발생 시 강제 정지
        superior.stop()
        superior.add_motor_to_zero(args.motor_id)
        superior.add_motor_to_zero(args.second_motor_id)
        raise

if __name__ == "__main__":
    main()