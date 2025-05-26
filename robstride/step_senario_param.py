#!/usr/bin/env python3
"""
모터 제어 시나리오 테스트 스크립트 - 자연스러운 전환 구현
"""
import argparse
import time
import math
import logging
from typing import Dict, List, Optional, Tuple
import threading
from motors.robstride.bindings import RobstrideMotorsSupervisor
# math.sin(2 * math.pi / period * elapsed_time + phase_right) + right_offset

POS = lambda A, T, t, phase, offset: A*math.sin(2 * math.pi / T * t + phase) + offset
VEL = lambda A, T, t, phase: A*(2 * math.pi / T) * math.cos(2 * math.pi / T * t + phase)

def calculate_motor_positions(elapsed_time: float, period: float, amplitude: float, 
                             phase_right: float = 0, phase_left: float = math.pi,
                             right_scale: float = 1.0, left_scale: float = 1.0,
                             right_offset: float = 0.0, left_offset: float = 0.0,
                             right_min_angle: float = None, right_max_angle: float = None,
                             left_min_angle: float = None, left_max_angle: float = None) -> Tuple[float, float, float, float]:
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
        right_offset: 오른쪽 다리 중심 위치 오프셋 (라디안)
        left_offset: 왼쪽 다리 중심 위치 오프셋 (라디안)
        right_min_angle: 오른쪽 다리 최소 각도 (라디안)
        right_max_angle: 오른쪽 다리 최대 각도 (라디안)
        left_min_angle: 왼쪽 다리 최소 각도 (라디안)
        left_max_angle: 왼쪽 다리 최대 각도 (라디안)
        
    Returns:
        right_pos, right_vel, left_pos, left_vel
    """
    # 오른쪽 다리
    right_pos = right_scale * POS(amplitude, period, elapsed_time, phase_right, right_offset)
    right_vel = right_scale * VEL(amplitude, period, elapsed_time, phase_right, right_offset)
    
    # 왼쪽 다리
    left_pos = left_scale * POS(amplitude, period, elapsed_time, phase_left, left_offset)
    left_vel = left_scale * VEL(amplitude, period, elapsed_time, phase_left, left_offset)
    
    # 각도 제한 적용 (설정된 경우)
    if right_min_angle is not None and right_pos < right_min_angle:
        #! DEPRECATED: 현재 각도 제한을 초과하면 속도를 0으로 설정
        #! TODO: 각도 제한을 사전에 계산하여
        right_pos = right_min_angle
        right_vel = 0
    if right_max_angle is not None and right_pos > right_max_angle:
        right_pos = right_max_angle
        right_vel = 0
        
    if left_min_angle is not None and left_pos < left_min_angle:
        left_pos = left_min_angle
        left_vel = 0
    if left_max_angle is not None and left_pos > left_max_angle:
        left_pos = left_max_angle
        left_vel = 0
    
    return right_pos, right_vel, left_pos, left_vel

def transition_positions(superior: RobstrideMotorsSupervisor, 
                         start_right_pos: float, start_left_pos: float,
                         end_right_pos: float, end_left_pos: float,
                         duration: float = 0.5) -> None:
    """
    시작 위치에서 종료 위치로 부드럽게 전환 - 역학적 관계 고려
    
    Args:
        superior: 모터 관리자
        start_right_pos: 오른쪽 다리 시작 위치
        start_left_pos: 왼쪽 다리 시작 위치
        end_right_pos: 오른쪽 다리 종료 위치
        end_left_pos: 왼쪽 다리 종료 위치
        duration: 전환 시간 (초)
    Reference:
        JK, Barath Kumar. "Dynamic gait modelling of lower limb dynamics: a mathematical approach." arXiv preprint arXiv:2310.09731 (2023).
    }
    """
    print(f"위치 전환 시작: 오른쪽 {start_right_pos:.2f} -> {end_right_pos:.2f}, 왼쪽 {start_left_pos:.2f} -> {end_left_pos:.2f}")
    
    start_time = time.time()
    while True:
        elapsed = time.time() - start_time
        #!TODO:  왼쪽, 오른쪽 Threading으로 동시성 확보
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
        
        superior.set_position(2, -left_pos)
        superior.set_velocity(2, left_vel)
        
        right_angle = right_pos * 180 / math.pi
        left_angle = left_pos * 180 / math.pi
        
        print(f"전환 중: right_pos: {right_pos:.2f}({right_angle:.2f}°), left_pos: {left_pos:.2f}({left_angle:.2f}°)")
        # time.sleep(0.5)  # 서보 업데이트 시간 확보
        print(f"경과시간: {elapsed:.2f}초, t: {t:.2f}, smooth_t: {smooth_t:.2f}")
        if right_pos <= 0.1 and left_pos <= 0.1:
            break
    
    # 최종 위치 설정 확인
    superior.set_position(1, end_right_pos)
    superior.set_position(2, end_left_pos)
    print("위치 전환 완료")

def fast_walk(superior: RobstrideMotorsSupervisor, period: float = 1.0, amplitude: float = 1.0, 
              duration: float = 3.0, right_offset: float = 0.0, left_offset: float = 0.0,
              right_min_angle: float = None, right_max_angle: float = None, 
              left_min_angle: float = None, left_max_angle: float = None, verbose=None) -> Tuple[float, float]:
    """
    빠른 걷기 시나리오
    
    Args:
        superior: 모터 제어 객체
        period: 주기 (초)
        amplitude: 진폭 (라디안)
        duration: 지속 시간 (초)
        right_offset: 오른쪽 다리 중심 위치 오프셋 (라디안)
        left_offset: 왼쪽 다리 중심 위치 오프셋 (라디안)
        right_min_angle: 오른쪽 다리 최소 각도 (라디안), None이면 제한 없음
        right_max_angle: 오른쪽 다리 최대 각도 (라디안), None이면 제한 없음
        left_min_angle: 왼쪽 다리 최소 각도 (라디안), None이면 제한 없음
        left_max_angle: 왼쪽 다리 최대 각도 (라디안), None이면 제한 없음
    
    Returns:
        마지막 오른쪽/왼쪽 다리 위치
    """
    print(f"빠른 걷기 시작 (주기: {period}초, 진폭: {amplitude}, 지속시간: {duration}초)")
    # print(f"각도 제한 - 오른쪽: [{right_min_angle and right_min_angle*180/math.pi:.1f}°, {right_max_angle and right_max_angle*180/math.pi:.1f}°], " + 
    #       f"왼쪽: [{left_min_angle and left_min_angle*180/math.pi:.1f}°, {left_max_angle and left_max_angle*180/math.pi:.1f}°]")
    
    start_time = time.time()
    last_right_pos = 0
    last_left_pos = 0
    
    try:
        while time.time() - start_time < duration:
            elapsed_time = time.time() - start_time
            #!TODO:  왼쪽, 오른쪽 Threading으로 동시성 확보
            # 위치 및 속도 계산
            right_pos, right_vel, left_pos, left_vel = calculate_motor_positions(
                elapsed_time, period, amplitude, 0, math.pi, 1.0, 1.0,
                right_offset, left_offset,
                right_min_angle, right_max_angle,
                left_min_angle, left_max_angle
            )
            
            # 모터에 위치 및 속도 설정
            superior.set_position(1, right_pos)
            superior.set_velocity(1, right_vel)
            
            superior.set_position(2, -left_pos)
            superior.set_velocity(2, left_vel)
            
            right_angle = right_pos * 180 / math.pi
            left_angle = left_pos * 180 / math.pi
            if verbose:
                print(f"빠른 걷기: right: {right_pos:.2f}({right_angle:.2f}°), left: {left_pos:.2f}({left_angle:.2f}°)")
                print(f"속도: {right_vel:.2f}, {left_vel:.2f}")
            
            # 마지막 위치 저장
            last_right_pos = right_pos
            last_left_pos = left_pos
            
            # time.sleep(0.5)  # CPU 부하 방지
            
    except Exception as e:
        print(f"빠른 걷기 오류: {e}")
        raise
    finally:
        print("빠른 걷기 완료")
    
    return last_right_pos, last_left_pos

def slow_walk(superior: RobstrideMotorsSupervisor, period: float = 2.0, amplitude: float = 1.0, 
              duration: float = 3.0, start_right: float = None, start_left: float = None,
              right_offset: float = 0.0, left_offset: float = 0.0,
              right_min_angle: float = None, right_max_angle: float = None,
              left_min_angle: float = None, left_max_angle: float = None, verbose:bool=None) -> Tuple[float, float]:
    """
    느린 걷기 시나리오
    
    Args:
        superior: 모터 제어 객체
        period: 주기 (초)
        amplitude: 진폭 (라디안)
        duration: 지속 시간 (초)
        start_right: 오른쪽 다리 시작 위치 (이전 모드에서 전환시)
        start_left: 왼쪽 다리 시작 위치 (이전 모드에서 전환시)
        right_offset: 오른쪽 다리 중심 위치 오프셋 (라디안)
        left_offset: 왼쪽 다리 중심 위치 오프셋 (라디안)
        right_min_angle: 오른쪽 다리 최소 각도 (라디안), None이면 제한 없음
        right_max_angle: 오른쪽 다리 최대 각도 (라디안), None이면 제한 없음
        left_min_angle: 왼쪽 다리 최소 각도 (라디안), None이면 제한 없음
        left_max_angle: 왼쪽 다리 최대 각도 (라디안), None이면 제한 없음
    
    Returns:
        마지막 오른쪽/왼쪽 다리 위치
    """
    print(f"느린 걷기 시작 (주기: {period}초, 진폭: {amplitude}, 지속시간: {duration}초)")
    # print(f"각도 제한 - 오른쪽: [{right_min_angle and right_min_angle*180/math.pi:.1f}°, {right_max_angle and right_max_angle*180/math.pi:.1f}°], " + 
    #       f"왼쪽: [{left_min_angle and left_min_angle*180/math.pi:.1f}°, {left_max_angle and left_max_angle*180/math.pi:.1f}°]")
    
    # 시작 위치가 주어진 경우, 전환 수행
    if start_right is not None and start_left is not None:
        # 첫 위치 계산 (느린 걷기의 시작 위치)
        init_right, _, init_left, _ = calculate_motor_positions(
            0, period, amplitude, 0, math.pi, 1.0, 1.0,
            right_offset, left_offset
        )
        
        # 부드러운 전환
        transition_positions(superior, start_right, start_left, init_right, init_left, 0.5)
    
    start_time = time.time()
    last_right_pos = 0
    last_left_pos = 0
    
    try:
        while time.time() - start_time < duration:
            elapsed_time = time.time() - start_time
            #!TODO:  왼쪽, 오른쪽 Threading으로 동시성 확보
            # 위치 및 속도 계산
            right_pos, right_vel, left_pos, left_vel = calculate_motor_positions(
                elapsed_time, period, amplitude, 0, math.pi, 1.0, 1.0,
                right_offset, left_offset,
                right_min_angle, right_max_angle,
                left_min_angle, left_max_angle
            )
            
            # 모터에 위치 및 속도 설정
            superior.set_position(1, right_pos)
            superior.set_velocity(1, right_vel)
            
            superior.set_position(2, -left_pos)
            superior.set_velocity(2, left_vel)
            
            right_angle = right_pos * 180 / math.pi
            left_angle = left_pos * 180 / math.pi
            if verbose:
                print(f"느린 걷기: right: {right_pos:.2f}({right_angle:.2f}°), left: {left_pos:.2f}({left_angle:.2f}°)")
                print(f"속도: {right_vel:.2f}, {left_vel:.2f}")
            
            # 마지막 위치 저장
            last_right_pos = right_pos
            last_left_pos = left_pos
            
            # time.sleep(0.5)  # CPU 부하 방지
            
    except Exception as e:
        print(f"느린 걷기 오류: {e}")
        raise
    finally:
        print("느린 걷기 완료")
    
    return last_right_pos, last_left_pos

def right_leg_injury(superior: RobstrideMotorsSupervisor, period: float = 2.0, amplitude: float = 1.0, 
                   duration: float = 3.0, start_right: float = None, start_left: float = None,
                   right_scale: float = 0.25, left_scale: float = 1.0,
                   right_offset: float = 0.0, left_offset: float = 0.0,
                   right_min_angle: float = None, right_max_angle: float = None,
                   left_min_angle: float = None, left_max_angle: float = None, verbose:bool=None) -> Tuple[float, float]:
    """
    오른쪽 다리 부상 시나리오
    
    Args:
        superior: 모터 제어 객체
        period: 주기 (초)
        amplitude: 진폭 (라디안)
        duration: 지속 시간 (초)
        start_right: 오른쪽 다리 시작 위치 (이전 모드에서 전환시)
        start_left: 왼쪽 다리 시작 위치 (이전 모드에서 전환시)
        right_scale: 오른쪽 다리 진폭 스케일 (부상 정도)
        left_scale: 왼쪽 다리 진폭 스케일
        right_offset: 오른쪽 다리 중심 위치 오프셋 (라디안)
        left_offset: 왼쪽 다리 중심 위치 오프셋 (라디안)
        right_min_angle: 오른쪽 다리 최소 각도 (라디안), None이면 제한 없음
        right_max_angle: 오른쪽 다리 최대 각도 (라디안), None이면 제한 없음
        left_min_angle: 왼쪽 다리 최소 각도 (라디안), None이면 제한 없음
        left_max_angle: 왼쪽 다리 최대 각도 (라디안), None이면 제한 없음
    
    Returns:
        마지막 오른쪽/왼쪽 다리 위치
    """
    print(f"오른쪽 다리 부상 시뮬레이션 시작 (주기: {period}초, 진폭: {amplitude}, 지속시간: {duration}초)")
    print(f"오른쪽 다리 스케일: {right_scale}, 왼쪽 다리 스케일: {left_scale}")
    # print(f"각도 제한 - 오른쪽: [{right_min_angle and right_min_angle*180/math.pi:.1f}°, {right_max_angle and right_max_angle*180/math.pi:.1f}°], " + 
    #       f"왼쪽: [{left_min_angle and left_min_angle*180/math.pi:.1f}°, {left_max_angle and left_max_angle*180/math.pi:.1f}°]")
    
    # 시작 위치가 주어진 경우, 전환 수행
    if start_right is not None and start_left is not None:
        # 첫 위치 계산 (오른쪽 다리 부상의 시작 위치)
        init_right, _, init_left, _ = calculate_motor_positions(
            0, period, amplitude, 0, math.pi, right_scale, left_scale,
            right_offset, left_offset
        )
        
        # 부드러운 전환
        transition_positions(superior, start_right, start_left, init_right, init_left, 0.5)
    
    start_time = time.time()
    last_right_pos = 0
    last_left_pos = 0
    
    try:
        while time.time() - start_time < duration:
            elapsed_time = time.time() - start_time
            #!TODO:  왼쪽, 오른쪽 Threading으로 동시성 확보
            # 위치 및 속도 계산 (오른쪽은 제한된 움직임)
            right_pos, right_vel, left_pos, left_vel = calculate_motor_positions(
                elapsed_time, period, amplitude, 0, math.pi, right_scale, left_scale,
                right_offset, left_offset,
                right_min_angle, right_max_angle,
                left_min_angle, left_max_angle
            )
            
            # 모터에 위치 및 속도 설정
            superior.set_position(1, right_pos)
            superior.set_velocity(1, right_vel)
            
            superior.set_position(2, -left_pos)
            superior.set_velocity(2, left_vel)
            
            right_angle = right_pos * 180 / math.pi
            left_angle = left_pos * 180 / math.pi
            if verbose:
                print(f"오른쪽 부상: right: {right_pos:.2f}({right_angle:.2f}°), left: {left_pos:.2f}({left_angle:.2f}°)")
                print(f"토크: {superior.get_torque(1):.2f}, {superior.get_torque(2):.2f}")
                print(f"속도: {right_vel:.2f}, {left_vel:.2f}")
            
            
            # 마지막 위치 저장
            last_right_pos = right_pos
            last_left_pos = left_pos
            
            # time.sleep(0.5)  # CPU 부하 방지
            
    except Exception as e:
        print(f"오른쪽 다리 부상 오류: {e}")
        raise
    finally:
        print("오른쪽 다리 부상 시뮬레이션 완료")
    
    return last_right_pos, last_left_pos

def left_leg_injury(superior: RobstrideMotorsSupervisor, period: float = 2.0, amplitude: float = 1.0, 
                   duration: float = 3.0, start_right: float = None, start_left: float = None,
                   right_scale: float = 0.25, left_scale: float = 1.0,
                   right_offset: float = 0.0, left_offset: float = 0.0,
                   right_min_angle: float = None, right_max_angle: float = None,
                   left_min_angle: float = None, left_max_angle: float = None, verbose:bool=None) -> Tuple[float, float]:
    """
    왼쪽 다리 부상 시나리오
    
    Returns:
        마지막 오른쪽/왼쪽 다리 위치
    """
    print(f"왼쪽 다리 부상 시뮬레이션 시작 (주기: {period}초, 진폭: {amplitude}, 지속시간: {duration}초)")
    
    # 부상 상황에서의 왼쪽 다리 스케일 (더 작은 움직임)
    # right_scale = 1.0
    # left_scale = 0.25
    
    # 시작 위치가 주어진 경우, 전환 수행
    if start_right is not None and start_left is not None:
        # 첫 위치 계산 (왼쪽 다리 부상의 시작 위치)
        init_right, _, init_left, _ = calculate_motor_positions(0, period, amplitude, 0, math.pi, right_scale, left_scale,
                                                                right_offset, left_offset, right_min_angle, right_max_angle,
                                                                left_min_angle, left_max_angle)
        
        # 부드러운 전환
        transition_positions(superior, start_right, start_left, init_right, init_left, 0.5)
    
    start_time = time.time()
    last_right_pos = 0
    last_left_pos = 0
    
    try:
        while time.time() - start_time < duration:
            elapsed_time = time.time() - start_time
            #!TODO:  왼쪽, 오른쪽 Threading으로 동시성 확보
            # 위치 및 속도 계산 (왼쪽은 제한된 움직임)
            right_pos, right_vel, left_pos, left_vel = calculate_motor_positions(
                elapsed_time, period, amplitude, 0, math.pi, right_scale, left_scale
            )
            
            # 모터에 위치 및 속도 설정
            superior.set_position(1, right_pos)
            superior.set_velocity(1, right_vel)
            
            superior.set_position(2, -left_pos)
            superior.set_velocity(2, left_vel)
            
            right_angle = right_pos * 180 / math.pi
            left_angle = left_pos * 180 / math.pi
            if verbose:
                print(f"왼쪽 부상: right: {right_pos:.2f}({right_angle:.2f}°), left: {left_pos:.2f}({left_angle:.2f}°)")
                print(f"토크: {superior.get_torque(1):.2f}, {superior.get_torque(2):.2f}")
                print(f"속도: {right_vel:.2f}, {left_vel:.2f}")
            # 마지막 위치 저장
            last_right_pos = right_pos
            last_left_pos = left_pos
            
            # # time.sleep(0.5)  # CPU 부하 방지
            
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
    parser.add_argument("--amplitude", type=float, default=1.0, help="진폭 (기본값: 1.0 라디안)")
    parser.add_argument("--kp", type=float, default=10.0, help="kp (기본값: 10.0)")
    parser.add_argument("--kd", type=float, default=1.0, help="kd (기본값: 1.0)")
    parser.add_argument("--motor-id", type=int, default=1, help="모터 ID (기본값: 1)")
    parser.add_argument("--motor-type", type=str, default="01", help="모터 타입 (기본값: 01)")
    parser.add_argument("--second-motor-id", type=int, default=2, help="모터 ID (기본값: 2)")
    parser.add_argument("--second-motor-type", type=str, default="01", help="모터 타입 (기본값: 01)")
    parser.add_argument("--duration", type=float, default=10.0, help="각 모드의 실행 시간 (초)")
    parser.add_argument("--cycles", type=int, default=10, help="전체 사이클 반복 횟수")
    
    # 각도 제한 옵션 추가
    parser.add_argument("--right-min-angle", type=float, help="오른쪽 다리 최소 각도 (도)")
    parser.add_argument("--right-max-angle", type=float, help="오른쪽 다리 최대 각도 (도)")
    parser.add_argument("--left-min-angle", type=float,  help="왼쪽 다리 최소 각도 (도)")
    parser.add_argument("--left-max-angle", type=float, help="왼쪽 다리 최대 각도 (도)")
    
    # 오프셋 옵션 추가
    parser.add_argument("--right-offset", type=float, default=0.0, help="오른쪽 다리 중심 오프셋 (도)")
    parser.add_argument("--left-offset", type=float, default=0.0, help="왼쪽 다리 중심 오프셋 (도)")
    
    # 부상 시뮬레이션을 위한 스케일 옵션
    parser.add_argument("--right-scale", type=float, default=0.25, help="오른쪽 다리 부상 시 스케일 (기본값: 0.25)")
    parser.add_argument("--left-scale", type=float, default=0.25, help="왼쪽 다리 부상 시 스케일 (기본값: 0.25)")
    
    args = parser.parse_args()
    
    
    # 도(degree)를 라디안으로 변환
    right_min_rad = math.radians(args.right_min_angle) if args.right_min_angle is not None else None
    right_max_rad = math.radians(args.right_max_angle) if args.right_max_angle is not None else None
    left_min_rad = math.radians(args.left_min_angle) if args.left_min_angle is not None else None
    left_max_rad = math.radians(args.left_max_angle) if args.left_max_angle is not None else None
    right_offset_rad = math.radians(args.right_offset)
    left_offset_rad = math.radians(args.left_offset)
    
    # 로깅 설정
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    print(f"각도 설정 - 오른쪽: 제한[{args.right_min_angle}°~{args.right_max_angle}°], 오프셋: {args.right_offset}°")
    print(f"각도 설정 - 왼쪽: 제한[{args.left_min_angle}°~{args.left_max_angle}°], 오프셋: {args.left_offset}°")
    
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
            right_pos, left_pos = fast_walk(
                superior, period=args.period/2, amplitude=args.amplitude, duration=args.duration,
                right_offset=right_offset_rad, left_offset=left_offset_rad,
                right_min_angle=right_min_rad, right_max_angle=right_max_rad,
                left_min_angle=left_min_rad, left_max_angle=left_max_rad
            )
            time.sleep(0.5)
            # 느린 걷기로 전환 (이전 마지막 위치에서 시작)
            right_pos, left_pos = slow_walk(
                superior, period=args.period, amplitude=args.amplitude, duration=args.duration,
                start_right=right_pos, start_left=left_pos,
                right_offset=right_offset_rad, left_offset=left_offset_rad,
                right_min_angle=right_min_rad, right_max_angle=right_max_rad,
                left_min_angle=left_min_rad, left_max_angle=left_max_rad
            )
            time.sleep(0.5)
            # 오른쪽 다리 부상으로 전환
            right_pos, left_pos = right_leg_injury(
                superior, period=args.period, amplitude=args.amplitude, duration=args.duration,
                start_right=right_pos, start_left=left_pos,
                right_scale=args.right_scale, left_scale=1.0,
                right_offset=right_offset_rad, left_offset=left_offset_rad,
                right_min_angle=right_min_rad, right_max_angle=right_max_rad,
                left_min_angle=left_min_rad, left_max_angle=left_max_rad
            )
            time.sleep(0.5)
            # 왼쪽 다리 부상으로 전환
            right_pos, left_pos = left_leg_injury(
                superior, period=args.period, amplitude=args.amplitude, duration=args.duration,
                start_right=right_pos, start_left=left_pos,
                right_scale=1.0, left_scale=args.left_scale,
                right_offset=right_offset_rad, left_offset=left_offset_rad,
                right_min_angle=right_min_rad, right_max_angle=right_max_rad,
                left_min_angle=left_min_rad, left_max_angle=left_max_rad
            )
            time.sleep(0.5)
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
    finally:
                # 모든 모터를 원점으로 복귀
        superior.add_motor_to_zero(args.motor_id)
        superior.add_motor_to_zero(args.second_motor_id)
        print("모터 원점 복귀 완료")
        superior.stop()
        print("모터 정지 완료")

if __name__ == "__main__":
    main()