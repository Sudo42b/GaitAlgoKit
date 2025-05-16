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


class Motor_Controller:
    def __init__(self, 
                 amplitude:float=1.0, 
                 phase:float=0, 
                 period:float=1, 
                 offset:float=0.0) -> None:
        """
        시간에 모터 위치 및 속도 계산
        
        Args:
            period: 주기
            amplitude: 진폭
            phase: 다리 위상
            offset: 다리 중심 위치 오프셋 (도)
            
        Returns:
            right_pos, right_vel, left_pos, left_vel
        """
        self.amplitude = amplitude
        self.phase = phase
        self.period = period
        self._offset = offset
        self.rad_offset = Motor_Controller.deg2rad(offset)
        
        self.position = 0.0
        self.deg_position = 0.0
        self.velocity = 0.0
        self.deg_velocity = 0.0
        self.last_position = 0.0
    
    def calculate_motor_positions(self, delta_t:float)-> Tuple[float, float]:
        """모터 위치 및 속도 계산
            return position(radian/dt), velocity(radian/dt)
        """
        
        # 새 위치 및 속도 계산
        self.position = self.pos(self.amplitude, 
                                 self.period, 
                                 delta_t, 
                                 self.phase, 
                                 self.rad_offset)
        self.deg_position = self.rad2deg(self.position)
        self.velocity = self.vel(self.amplitude, 
                                 self.period, 
                                 delta_t, 
                                 self.phase)
        self.deg_velocity = self.rad2deg(self.velocity)
        
        # 각도 제한 적용 (설정된 경우)
        
        # 마지막 위치 저장
        self.last_position = self.position
        return self.position, self.velocity
    @property
    def offset(self):
        """오프셋 (도)"""
        return self._offset
    @offset.setter
    def offset(self, value: float):
        """오프셋 설정 (도)"""
        self._offset = value
        self.rad_offset = Motor_Controller.deg2rad(value)
    
    @staticmethod
    def vel(A: float, T: float, t: float, phase: float):
        """모터 속도 계산"""
        return A * (2 * math.pi / T) * math.cos(2 * math.pi / T * t + phase)
    
    @staticmethod
    def pos(A: float, T: float, t: float, phase: float, offset: float):
        """모터 위치 계산"""
        return A * math.sin(2 * math.pi / T * t + phase) + offset
    
    @staticmethod
    def rad2deg(rad: float) -> float:
        """라디안 -> 도 변환"""
        return rad * 180 / math.pi
    
    @staticmethod
    def deg2rad(deg: float) -> float:
        """도 -> 라디안 변환"""
        return deg * math.pi / 180
    

def walk(superior: RobstrideMotorsSupervisor, 
              right_mc: Motor_Controller,
              left_mc: Motor_Controller,
              duration: float = 3.0,
              verbose:bool=False) -> Tuple[Motor_Controller, Motor_Controller]:
    """
    빠른 걷기 시나리오
    
    Args:
        superior: 모터 제어 객체
        right_mc: 오른쪽 다리 모터 제어 객체
        left_mc: 왼쪽 다리 모터 제어 객체
        duration: 지속 시간 (초)
    
    Returns:
        마지막 오른쪽/왼쪽 다리 위치
    """
    msg = "빠른 걷기 시작" + f"주기: right: {right_mc.period}초, left: {left_mc.period}초\n" +\
    f"진폭: right: {right_mc.amplitude*180/math.pi}°, left: {left_mc.amplitude*180/math.pi}°\n"+ \
    f"오프셋: right: {right_mc.rad_offset*180/math.pi:.1f}°, left: {left_mc.rad_offset*180/math.pi:.1f}°\v" + \
    f"지속시간: {duration}초"
    print(msg)
    # print(f"각도 제한 - 오른쪽: [{right_mc.rad_min_angle and right_mc.rad_min_angle*180/math.pi:.1f}°, {right_mc.rad_max_angle and right_mc.rad_max_angle*180/math.pi:.1f}°], " +
    #       f"왼쪽: [{left_mc.rad_min_angle and left_mc.rad_min_angle*180/math.pi:.1f}°, {left_mc.rad_max_angle and left_mc.rad_max_angle*180/math.pi:.1f}°]")
    
    
    t_0 = time.time()
    
    try:
        while time.time() - t_0 < duration:
            delta_t = time.time() - t_0 #dt
            
            # 위치 및 속도 계산
            right_mc.calculate_motor_positions(delta_t)
            left_mc.calculate_motor_positions(delta_t)
            
            # 모터에 위치 및 속도 설정
            superior.set_position(1, right_mc.position)
            superior.set_velocity(1, right_mc.velocity)
            
            superior.set_position(2, left_mc.position)
            superior.set_velocity(2, left_mc.velocity)
            
            # m1_t = superior.get_torque(1)
            # m2_t = superior.get_torque(2)
            
            m1_kp = superior.get_kp(1)
            m2_kp = superior.get_kp(2)
            
            m1_kd = superior.get_kd(1)
            m2_kd = superior.get_kd(2)
            if verbose:
                # print one-line
                status_line = f"빠른 걷기: R: {right_mc.position:5.2f}({right_mc.deg_position:5.2f}°), L: {left_mc.position:5.2f}({left_mc.deg_position:5.2f}°) | 속도: R: {right_mc.velocity:5.2f}, L: {left_mc.deg_velocity:5.2f}"
                # f"| 토크: M1: {m1_t:5.2f}, M2: {m2_t:5.2f} | kp: M1: {m1_kp:5.2f}, M2: {m2_kp:5.2f} | kd: M1: {m1_kd:5.2f}, M2: {m2_kd:5.2f}"
                print(status_line, end="\r")
            # time.sleep(0.01)  # CPU 부하 방지
            
    except Exception as e:
        print(f"빠른 걷기 오류: {e}")
        raise
    finally:
        print("빠른 걷기 완료")
    
    return right_mc, left_mc

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
            0, period, amplitude, 0, math.pi, right_scale, 1.0,
            right_offset, left_offset
        )
        # 부드러운 전환
        calib_positions(superior, start_right, start_left, init_right, init_left, 0.5)
    
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
        calib_positions(superior, start_right, start_left, init_right, init_left, 0.5)
    
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


def sefe_stop_v2(superior: RobstrideMotorsSupervisor, 
              current_right: float = 0, 
              current_left: float = 0) -> None:
    """안전하게 원점으로 복귀 후 정지"""
    print("안전하게 모터 정지...")
    
    # 현재 위치에서 원점으로 부드럽게 전환
    from step_senario_param import transition_positions
    
    # 현재 위치에서 원점으로 부드럽게 전환
    transition_positions(superior, current_right, current_left, 0, 0, 1.0)
    
    # 모터 정지
    superior.add_motor_to_zero(1)
    superior.add_motor_to_zero(2)
    print("모터 정지 완료")
def safe_stop(superior: RobstrideMotorsSupervisor, 
              current_right: float = 0, current_left: float = 0) -> None:
    """안전하게 원점으로 복귀 후 정지"""
    print("안전하게 모터 정지...")
    
    # 현재 위치에서 원점으로 부드럽게 전환
    # calib_positions(superior, current_right, current_left, 0, 0, 1.0)
    
    # 모터 정지
    superior.add_motor_to_zero(1)
    superior.add_motor_to_zero(2)
    print("모터 정지 완료")

def main(args) -> None:
    # 도(degree)를 라디안으로 변환
    # right_min_rad = math.radians(args.right_min_angle) if args.right_min_angle is not None else None
    # right_max_rad = math.radians(args.right_max_angle) if args.right_max_angle is not None else None
    # left_min_rad = math.radians(args.left_min_angle) if args.left_min_angle is not None else None
    # left_max_rad = math.radians(args.left_max_angle) if args.left_max_angle is not None else None
    # right_offset_rad = math.radians(args.right_offset)
    # left_offset_rad = math.radians(args.left_offset)
    
    right_mc = Motor_Controller(amplitude=args.amplitude, 
                                period=args.period, 
                                phase=0,
                                offset=args.right_offset)
    left_mc = Motor_Controller(amplitude= -args.amplitude, 
                                period= args.period, 
                                phase=math.pi,
                                offset= args.left_offset)
    # 로깅 설정
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    print(f"각도 설정 - 오른쪽 오프셋: {args.right_offset}°")
    print(f"각도 설정 - 왼쪽 오프셋: {args.left_offset}°")
    
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
            right_mc, left_mc = walk(superior= superior, 
                    right_mc= right_mc, 
                    left_mc= left_mc,
                    duration=args.duration,
                    verbose=args.verbose)
            # time.sleep(0.5)
            # 느린 걷기로 전환 (이전 마지막 위치에서 시작)
            # right_mc.period /=2
            # left_mc.period /=2
            # walk(superior= superior, 
            #                     right_mc= right_mc, 
            #                     left_mc= left_mc,
            #                     duration=args.duration)
            # right_pos, left_pos = slow_walk(
            #     superior, 
            #     period=args.period, 
            #     amplitude=args.amplitude, 
            #     duration=args.duration,
            #     start_right=right_pos, 
            #     start_left=left_pos,
            # )
            # time.sleep(0.5)
            # # 오른쪽 다리 부상으로 전환
            # right_pos, left_pos = right_leg_injury(
            #     superior, period=args.period, amplitude=args.amplitude, duration=args.duration,
            #     start_right=right_pos, start_left=left_pos,
            #     right_scale=args.right_scale, left_scale=1.0,
            #     right_offset=right_offset_rad, left_offset=left_offset_rad,
            #     right_min_angle=right_min_rad, right_max_angle=right_max_rad,
            #     left_min_angle=left_min_rad, left_max_angle=left_max_rad
            # )
            # time.sleep(0.5)
            # # 왼쪽 다리 부상으로 전환
            # right_pos, left_pos = left_leg_injury(
            #     superior, period=args.period, amplitude=args.amplitude, duration=args.duration,
            #     start_right=right_pos, start_left=left_pos,
            #     right_scale=1.0, left_scale=args.left_scale,
            #     right_offset=right_offset_rad, left_offset=left_offset_rad,
            #     right_min_angle=right_min_rad, right_max_angle=right_max_rad,
            #     left_min_angle=left_min_rad, left_max_angle=left_max_rad
            # )
            # time.sleep(0.5)
            print(f"\n===== 사이클 {cycle+1}/{args.cycles} 완료 =====\n")
        
        # 안전하게 정지
        sefe_stop_v2(superior, right_mc.position, left_mc.position)
            
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
        print("모터 원점 복귀 완료")
        superior.stop()
        # 모든 모터를 원점으로 복귀
        superior.add_motor_to_zero(args.motor_id)
        superior.add_motor_to_zero(args.second_motor_id)
        print("모터 정지 완료")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="모터 제어 시나리오 테스트")
    parser.add_argument("-v", "--verbose", action="store_true", help="상세 출력 활성화")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="시리얼 포트 이름")
    parser.add_argument("--period", type=float, default=6.0, help="주기 (초)")
    parser.add_argument("--amplitude", type=float, default=1.2, help="진폭 (기본값: 1.0 라디안)")
    parser.add_argument("--kp", type=float, default=10.0, help="kp (기본값: 10.0)")
    parser.add_argument("--kd", type=float, default=2.0, help="kd (기본값: 1.0)")
    parser.add_argument("--motor-id", type=int, default=1, help="모터 ID (기본값: 1)")
    parser.add_argument("--motor-type", type=str, default="01", help="모터 타입 (기본값: 01)")
    parser.add_argument("--second-motor-id", type=int, default=2, help="모터 ID (기본값: 2)")
    parser.add_argument("--second-motor-type", type=str, default="01", help="모터 타입 (기본값: 01)")
    parser.add_argument("--duration", type=float, default=3600.0, help="각 모드의 실행 시간 (초)")
    parser.add_argument("--cycles", type=int, default=1, help="전체 사이클 반복 횟수")
    
    # 부상 시뮬레이션을 위한 스케일 옵션
    parser.add_argument("--scale", type=float, default=0.25, help="다리 부상 시 스케일 (기본값: 0.25)")
    # 오프셋 옵션 추가
    parser.add_argument("--right-offset", type=float, default=0.0, help="오른쪽 다리 중심 오프셋 (도)")
    parser.add_argument("--left-offset", type=float, default=0.0, help="왼쪽 다리 중심 오프셋 (도)")
    
    args = parser.parse_args()
    main(args)