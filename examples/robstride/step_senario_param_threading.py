#!/usr/bin/env python3
"""
모터 제어 시나리오 테스트 스크립트 - 자연스러운 전환 구현
멀티스레딩을 통한 왼쪽/오른쪽 다리 동시성 제어
"""

import argparse
import time
import math
import sys
import logging
import threading
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Optional, Tuple
from motors.robstride.can_communication import MotorType
from motors.robstride.bindings import (
    RobstrideMotorsSupervisor,
    RobstrideMotorControlParams,
    RobstrideMotorFeedback
)

# 스레드 안전성을 위한 락
motor_lock = threading.RLock()

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
    # 위치(라디안), 속도함수는 P(t) = A * sin(2 * pi / T * t + phi) + offset
    # 오른쪽 다리
    right_pos = amplitude * right_scale * math.sin(2 * math.pi / period * elapsed_time + phase_right) + right_offset
    right_vel = amplitude * right_scale * (2 * math.pi / period) * math.cos(2 * math.pi / period * elapsed_time + phase_right)
    
    # 왼쪽 다리
    left_pos = amplitude * left_scale * math.sin(2 * math.pi / period * elapsed_time + phase_left) + left_offset
    left_vel = amplitude * left_scale * (2 * math.pi / period) * math.cos(2 * math.pi / period * elapsed_time + phase_left)
    
    # 각도 제한 적용 (설정된 경우)
    if right_min_angle is not None and right_pos < right_min_angle:
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

def set_right_leg_position(superior: RobstrideMotorsSupervisor, position: float, velocity: float) -> None:
    """오른쪽 다리 위치 및 속도 설정 (스레드 안전)"""
    with motor_lock:
        superior.set_position(1, position)
        superior.set_velocity(1, velocity)

def set_left_leg_position(superior: RobstrideMotorsSupervisor, position: float, velocity: float) -> None:
    """왼쪽 다리 위치 및 속도 설정 (스레드 안전)"""
    with motor_lock:
        superior.set_position(2, -position)  # 왼쪽 다리는 방향이 반대
        superior.set_velocity(2, velocity)

def transition_positions(superior: RobstrideMotorsSupervisor, 
                         start_right_pos: float, start_left_pos: float,
                         end_right_pos: float, end_left_pos: float,
                         duration: float = 0.5) -> None:
    """
    시작 위치에서 종료 위치로 부드럽게 전환 - 역학적 관계 고려 (병렬 처리 적용)
    
    Args:
        superior: 모터 관리자
        start_right_pos: 오른쪽 다리 시작 위치
        start_left_pos: 왼쪽 다리 시작 위치
        end_right_pos: 오른쪽 다리 종료 위치
        end_left_pos: 왼쪽 다리 종료 위치
        duration: 전환 시간 (초)
    Reference:
        JK, Barath Kumar. "Dynamic gait modelling of lower limb dynamics: a mathematical approach." arXiv preprint arXiv:2310.09731 (2023).
    """
    print(f"위치 전환 시작: 오른쪽 {start_right_pos:.2f} -> {end_right_pos:.2f}, 왼쪽 {start_left_pos:.2f} -> {end_left_pos:.2f}")
    
    start_time = time.time()
    
    def update_right_leg():
        """오른쪽 다리 위치 업데이트 스레드 함수"""
        while True:
            elapsed = time.time() - start_time
            if elapsed >= duration:
                # 최종 위치 설정
                set_right_leg_position(superior, end_right_pos, 0)
                break
                
            # 부드러운 전환을 위한 easing 함수 (사인 기반)
            t = elapsed / duration
            smooth_t = (1 - math.cos(t * math.pi)) / 2
            
            # 현재 위치 계산
            right_pos = start_right_pos + (end_right_pos - start_right_pos) * smooth_t
            
            # 속도 계산 (이동 방향에 맞게)
            right_vel = (end_right_pos - start_right_pos) * math.pi/2 * math.sin(t * math.pi) / duration
            
            # 위치 및 속도 설정
            set_right_leg_position(superior, right_pos, right_vel)
            
            right_angle = right_pos * 180 / math.pi
            print(f"오른쪽 전환 중: {right_pos:.2f}({right_angle:.2f}°), vel: {right_vel:.2f}")
            
            # 종료 조건 추가 체크
            if abs(right_pos - end_right_pos) < 0.05:
                set_right_leg_position(superior, end_right_pos, 0)
                break
    
    def update_left_leg():
        """왼쪽 다리 위치 업데이트 스레드 함수"""
        while True:
            elapsed = time.time() - start_time
            if elapsed >= duration:
                # 최종 위치 설정
                set_left_leg_position(superior, end_left_pos, 0)
                break
                
            # 부드러운 전환을 위한 easing 함수 (사인 기반)
            t = elapsed / duration
            smooth_t = (1 - math.cos(t * math.pi)) / 2
            
            # 현재 위치 계산
            left_pos = start_left_pos + (end_left_pos - start_left_pos) * smooth_t
            
            # 속도 계산 (이동 방향에 맞게)
            left_vel = (end_left_pos - start_left_pos) * math.pi/2 * math.sin(t * math.pi) / duration
            
            # 위치 및 속도 설정
            set_left_leg_position(superior, left_pos, left_vel)
            
            left_angle = left_pos * 180 / math.pi
            print(f"왼쪽 전환 중: {left_pos:.2f}({left_angle:.2f}°), vel: {left_vel:.2f}")
            
            # 종료 조건 추가 체크
            if abs(left_pos - end_left_pos) < 0.05:
                set_left_leg_position(superior, end_left_pos, 0)
                break
    
    # 스레드 생성 및 실행
    with ThreadPoolExecutor(max_workers=2) as executor:
        right_future = executor.submit(update_right_leg)
        left_future = executor.submit(update_left_leg)
        
        # 모든 스레드가 완료될 때까지 대기
        right_future.result()
        left_future.result()
    
    print("위치 전환 완료")

def fast_walk(superior: RobstrideMotorsSupervisor, period: float = 1.0, amplitude: float = 1.0, 
              duration: float = 3.0, right_offset: float = 0.0, left_offset: float = 0.0,
              right_min_angle: float = None, right_max_angle: float = None, 
              left_min_angle: float = None, left_max_angle: float = None, verbose=None) -> Tuple[float, float]:
    """
    빠른 걷기 시나리오 (병렬 처리 적용)
    
    Returns:
        마지막 오른쪽/왼쪽 다리 위치
    """
    print(f"빠른 걷기 시작 (주기: {period}초, 진폭: {amplitude}, 지속시간: {duration}초)")
    
    start_time = time.time()
    right_pos, left_pos = 0, 0
    stop_event = threading.Event()
    position_lock = threading.Lock()
    
    # 최종 위치 저장을 위한 공유 변수
    last_positions = {'right_pos': 0, 'left_pos': 0}
    
    def update_right_leg():
        """오른쪽 다리 제어 스레드 함수"""
        while not stop_event.is_set():
            elapsed_time = time.time() - start_time
            if elapsed_time >= duration:
                break
                
            # 위치 및 속도 계산
            right_pos, right_vel, _, _ = calculate_motor_positions(
                elapsed_time, period, amplitude, 0, math.pi, 1.0, 1.0,
                right_offset, left_offset,
                right_min_angle, right_max_angle,
                None, None
            )
            
            # 모터에 위치 및 속도 설정
            set_right_leg_position(superior, right_pos, right_vel)
            
            right_angle = right_pos * 180 / math.pi
            if verbose:
                print(f"빠른 걷기(오른쪽): {right_pos:.2f}({right_angle:.2f}°), 속도: {right_vel:.2f}")
            
            # 마지막 위치 저장
            with position_lock:
                last_positions['right_pos'] = right_pos
            
            # 짧은 대기 (CPU 사용량 조절)
            time.sleep(0.01)
    
    def update_left_leg():
        """왼쪽 다리 제어 스레드 함수"""
        while not stop_event.is_set():
            elapsed_time = time.time() - start_time
            if elapsed_time >= duration:
                break
                
            # 위치 및 속도 계산
            _, _, left_pos, left_vel = calculate_motor_positions(
                elapsed_time, period, amplitude, 0, math.pi, 1.0, 1.0,
                right_offset, left_offset,
                None, None,
                left_min_angle, left_max_angle
            )
            
            # 모터에 위치 및 속도 설정
            set_left_leg_position(superior, left_pos, left_vel)
            
            left_angle = left_pos * 180 / math.pi
            if verbose:
                print(f"빠른 걷기(왼쪽): {left_pos:.2f}({left_angle:.2f}°), 속도: {left_vel:.2f}")
            
            # 마지막 위치 저장
            with position_lock:
                last_positions['left_pos'] = left_pos
            
            # 짧은 대기 (CPU 사용량 조절)
            time.sleep(0.01)
    
    try:
        # 스레드 생성 및 실행
        with ThreadPoolExecutor(max_workers=2) as executor:
            right_future = executor.submit(update_right_leg)
            left_future = executor.submit(update_left_leg)
            
            # 모든 스레드가 완료될 때까지 대기 또는 타임아웃
            remaining_time = duration
            start_wait = time.time()
            
            # 실행 시간 동안 메인 스레드는 기다림
            time.sleep(duration)
            
            # 종료 신호 전송
            stop_event.set()
            
            # 스레드 완료 대기
            right_future.result()
            left_future.result()
        
    except Exception as e:
        print(f"빠른 걷기 오류: {e}")
        stop_event.set()  # 오류 발생 시 모든 스레드 종료
        raise
    finally:
        print("빠른 걷기 완료")
    
    return last_positions['right_pos'], last_positions['left_pos']

def slow_walk(superior: RobstrideMotorsSupervisor, period: float = 2.0, amplitude: float = 1.0, 
              duration: float = 3.0, start_right: float = None, start_left: float = None,
              right_offset: float = 0.0, left_offset: float = 0.0,
              right_min_angle: float = None, right_max_angle: float = None,
              left_min_angle: float = None, left_max_angle: float = None, verbose:bool=None) -> Tuple[float, float]:
    """
    느린 걷기 시나리오 (병렬 처리 적용)
    
    Returns:
        마지막 오른쪽/왼쪽 다리 위치
    """
    print(f"느린 걷기 시작 (주기: {period}초, 진폭: {amplitude}, 지속시간: {duration}초)")
    
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
    stop_event = threading.Event()
    position_lock = threading.Lock()
    
    # 최종 위치 저장을 위한 공유 변수
    last_positions = {'right_pos': 0, 'left_pos': 0}
    
    def update_right_leg():
        """오른쪽 다리 제어 스레드 함수"""
        while not stop_event.is_set():
            elapsed_time = time.time() - start_time
            if elapsed_time >= duration:
                break
                
            # 위치 및 속도 계산
            right_pos, right_vel, _, _ = calculate_motor_positions(
                elapsed_time, period, amplitude, 0, math.pi, 1.0, 1.0,
                right_offset, left_offset,
                right_min_angle, right_max_angle,
                None, None
            )
            
            # 모터에 위치 및 속도 설정
            set_right_leg_position(superior, right_pos, right_vel)
            
            right_angle = right_pos * 180 / math.pi
            if verbose:
                print(f"느린 걷기(오른쪽): {right_pos:.2f}({right_angle:.2f}°), 속도: {right_vel:.2f}")
            
            # 마지막 위치 저장
            with position_lock:
                last_positions['right_pos'] = right_pos
            
            # 짧은 대기 (CPU 사용량 조절)
            time.sleep(0.01)
    
    def update_left_leg():
        """왼쪽 다리 제어 스레드 함수"""
        while not stop_event.is_set():
            elapsed_time = time.time() - start_time
            if elapsed_time >= duration:
                break
                
            # 위치 및 속도 계산
            _, _, left_pos, left_vel = calculate_motor_positions(
                elapsed_time, period, amplitude, 0, math.pi, 1.0, 1.0,
                right_offset, left_offset,
                None, None,
                left_min_angle, left_max_angle
            )
            
            # 모터에 위치 및 속도 설정
            set_left_leg_position(superior, left_pos, left_vel)
            
            left_angle = left_pos * 180 / math.pi
            if verbose:
                print(f"느린 걷기(왼쪽): {left_pos:.2f}({left_angle:.2f}°), 속도: {left_vel:.2f}")
            
            # 마지막 위치 저장
            with position_lock:
                last_positions['left_pos'] = left_pos
            
            # 짧은 대기 (CPU 사용량 조절)
            time.sleep(0.01)
    
    try:
        # 스레드 생성 및 실행
        with ThreadPoolExecutor(max_workers=2) as executor:
            right_future = executor.submit(update_right_leg)
            left_future = executor.submit(update_left_leg)
            
            # 실행 시간 동안 메인 스레드는 기다림
            time.sleep(duration)
            
            # 종료 신호 전송
            stop_event.set()
            
            # 스레드 완료 대기
            right_future.result()
            left_future.result()
        
    except Exception as e:
        print(f"느린 걷기 오류: {e}")
        stop_event.set()  # 오류 발생 시 모든 스레드 종료
        raise
    finally:
        print("느린 걷기 완료")
    
    return last_positions['right_pos'], last_positions['left_pos']

def right_leg_injury(superior: RobstrideMotorsSupervisor, period: float = 2.0, amplitude: float = 1.0, 
                   duration: float = 3.0, start_right: float = None, start_left: float = None,
                   right_scale: float = 0.25, left_scale: float = 1.0,
                   right_offset: float = 0.0, left_offset: float = 0.0,
                   right_min_angle: float = None, right_max_angle: float = None,
                   left_min_angle: float = None, left_max_angle: float = None, verbose:bool=None) -> Tuple[float, float]:
    """
    오른쪽 다리 부상 시나리오 (병렬 처리 적용)
    
    Returns:
        마지막 오른쪽/왼쪽 다리 위치
    """
    print(f"오른쪽 다리 부상 시뮬레이션 시작 (주기: {period}초, 진폭: {amplitude}, 지속시간: {duration}초)")
    print(f"오른쪽 다리 스케일: {right_scale}, 왼쪽 다리 스케일: {left_scale}")
    
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
    stop_event = threading.Event()
    position_lock = threading.Lock()
    
    # 최종 위치 저장을 위한 공유 변수
    last_positions = {'right_pos': 0, 'left_pos': 0}
    
    def update_right_leg():
        """오른쪽 다리 제어 스레드 함수 (부상 시뮬레이션)"""
        while not stop_event.is_set():
            elapsed_time = time.time() - start_time
            if elapsed_time >= duration:
                break
                
            # 위치 및 속도 계산 (오른쪽은 제한된 움직임)
            right_pos, right_vel, _, _ = calculate_motor_positions(
                elapsed_time, period, amplitude, 0, math.pi, right_scale, left_scale,
                right_offset, left_offset,
                right_min_angle, right_max_angle,
                None, None
            )
            
            # 모터에 위치 및 속도 설정
            set_right_leg_position(superior, right_pos, right_vel)
            
            right_angle = right_pos * 180 / math.pi
            if verbose:
                print(f"왼쪽 부상(오른쪽): {right_pos:.2f}({right_angle:.2f}°), 속도: {right_vel:.2f}")
                with motor_lock:
                    print(f"토크(오른쪽): {superior.get_torque(1):.2f}")
            
            # 마지막 위치 저장
            with position_lock:
                last_positions['right_pos'] = right_pos
            
            # 짧은 대기 (CPU 사용량 조절)
            time.sleep(0.01)
    
    def update_left_leg():
        """왼쪽 다리 제어 스레드 함수 (부상 시뮬레이션)"""
        while not stop_event.is_set():
            elapsed_time = time.time() - start_time
            if elapsed_time >= duration:
                break
                
            # 위치 및 속도 계산 (왼쪽은 제한된 움직임)
            _, _, left_pos, left_vel = calculate_motor_positions(
                elapsed_time, period, amplitude, 0, math.pi, right_scale, left_scale,
                right_offset, left_offset,
                None, None,
                left_min_angle, left_max_angle
            )
            
            # 모터에 위치 및 속도 설정
            set_left_leg_position(superior, left_pos, left_vel)
            
            left_angle = left_pos * 180 / math.pi
            if verbose:
                print(f"왼쪽 부상(왼쪽): {left_pos:.2f}({left_angle:.2f}°), 속도: {left_vel:.2f}")
                with motor_lock:
                    print(f"토크(왼쪽): {superior.get_torque(2):.2f}")
            
            # 마지막 위치 저장
            with position_lock:
                last_positions['left_pos'] = left_pos
            
            # 짧은 대기 (CPU 사용량 조절)
            time.sleep(0.01)
    
    try:
        # 스레드 생성 및 실행
        with ThreadPoolExecutor(max_workers=2) as executor:
            right_future = executor.submit(update_right_leg)
            left_future = executor.submit(update_left_leg)
            
            # 실행 시간 동안 메인 스레드는 기다림
            time.sleep(duration)
            
            # 종료 신호 전송
            stop_event.set()
            
            # 스레드 완료 대기
            right_future.result()
            left_future.result()
        
    except Exception as e:
        print(f"왼쪽 다리 부상 오류: {e}")
        stop_event.set()  # 오류 발생 시 모든 스레드 종료
        raise
    finally:
        print("왼쪽 다리 부상 시뮬레이션 완료")
    
    return last_positions['right_pos'], last_positions['left_pos']
                None, None
            )
            
            # 모터에 위치 및 속도 설정
            set_right_leg_position(superior, right_pos, right_vel)
            
            right_angle = right_pos * 180 / math.pi
            if verbose:
                print(f"오른쪽 부상(오른쪽): {right_pos:.2f}({right_angle:.2f}°), 속도: {right_vel:.2f}")
                with motor_lock:
                    print(f"토크(오른쪽): {superior.get_torque(1):.2f}")
            
            # 마지막 위치 저장
            with position_lock:
                last_positions['right_pos'] = right_pos
            
            # 짧은 대기 (CPU 사용량 조절)
            time.sleep(0.01)
    
    def update_left_leg():
        """왼쪽 다리 제어 스레드 함수"""
        while not stop_event.is_set():
            elapsed_time = time.time() - start_time
            if elapsed_time >= duration:
                break
                
            # 위치 및 속도 계산
            _, _, left_pos, left_vel = calculate_motor_positions(
                elapsed_time, period, amplitude, 0, math.pi, right_scale, left_scale,
                right_offset, left_offset,
                None, None,
                left_min_angle, left_max_angle
            )
            
            # 모터에 위치 및 속도 설정
            set_left_leg_position(superior, left_pos, left_vel)
            
            left_angle = left_pos * 180 / math.pi
            if verbose:
                print(f"오른쪽 부상(왼쪽): {left_pos:.2f}({left_angle:.2f}°), 속도: {left_vel:.2f}")
                with motor_lock:
                    print(f"토크(왼쪽): {superior.get_torque(2):.2f}")
            
            # 마지막 위치 저장
            with position_lock:
                last_positions['left_pos'] = left_pos
            
            # 짧은 대기 (CPU 사용량 조절)
            time.sleep(0.01)
    
    try:
        # 스레드 생성 및 실행
        with ThreadPoolExecutor(max_workers=2) as executor:
            right_future = executor.submit(update_right_leg)
            left_future = executor.submit(update_left_leg)
            
            # 실행 시간 동안 메인 스레드는 기다림
            time.sleep(duration)
            
            # 종료 신호 전송
            stop_event.set()
            
            # 스레드 완료 대기
            right_future.result()
            left_future.result()
        
    except Exception as e:
        print(f"오른쪽 다리 부상 오류: {e}")
        stop_event.set()  # 오류 발생 시 모든 스레드 종료
        raise
    finally:
        print("오른쪽 다리 부상 시뮬레이션 완료")
    
    return last_positions['right_pos'], last_positions['left_pos']

def left_leg_injury(superior: RobstrideMotorsSupervisor, period: float = 2.0, amplitude: float = 1.0, 
                   duration: float = 3.0, start_right: float = None, start_left: float = None,
                   right_scale: float = 1.0, left_scale: float = 0.25,
                   right_offset: float = 0.0, left_offset: float = 0.0,
                   right_min_angle: float = None, right_max_angle: float = None,
                   left_min_angle: float = None, left_max_angle: float = None, verbose:bool=None) -> Tuple[float, float]:
    """
    왼쪽 다리 부상 시나리오 (병렬 처리 적용)
    
    Returns:
        마지막 오른쪽/왼쪽 다리 위치
    """
    print(f"왼쪽 다리 부상 시뮬레이션 시작 (주기: {period}초, 진폭: {amplitude}, 지속시간: {duration}초)")
    print(f"오른쪽 다리 스케일: {right_scale}, 왼쪽 다리 스케일: {left_scale}")
    
    # 시작 위치가 주어진 경우, 전환 수행
    if start_right is not None and start_left is not None:
        # 첫 위치 계산 (왼쪽 다리 부상의 시작 위치)
        init_right, _, init_left, _ = calculate_motor_positions(
            0, period, amplitude, 0, math.pi, right_scale, left_scale,
            right_offset, left_offset
        )
        
        # 부드러운 전환
        transition_positions(superior, start_right, start_left, init_right, init_left, 0.5)
    
    start_time = time.time()
    stop_event = threading.Event()
    position_lock = threading.Lock()
    
    # 최종 위치 저장을 위한 공유 변수
    last_positions = {'right_pos': 0, 'left_pos': 0}
    
    def update_right_leg():
        """오른쪽 다리 제어 스레드 함수"""
        while not stop_event.is_set():
            elapsed_time = time.time() - start_time
            if elapsed_time >= duration:
                break
                
            # 위치 및 속도 계산
            right_pos, right_vel, _, _ = calculate_motor_positions(
                elapsed_time, period, amplitude, 0, math.pi, right_scale, left_scale,
                right_offset, left_offset,
                right_min_angle, right_max_angle,