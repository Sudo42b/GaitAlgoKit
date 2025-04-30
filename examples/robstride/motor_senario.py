#!/usr/bin/env python3
"""
모터 제어 시나리오 테스트 스크립트
"""

import argparse
import time
import math
import sys
import logging
from typing import Dict, List, Optional
import threading
from motors.robstride.can_communication import MotorType
from motors.robstride.bindings import (
    RobstrideMotorsSupervisor,
    RobstrideMotorControlParams,
    RobstrideMotorFeedback
)

        


def fast_walk(superior:RobstrideMotorsSupervisor, period: float = 1.0, amplitude: float = 1.0):
    """빠른 걷기 시나리오"""
    print(f"빠른 걷기 시작 (주기: {period}초, 진폭: {amplitude})")
    start_time = time.time()
    
    # 3초동안 작동
    try:
        elapsed_time = time.time() - start_time
        RIGHT = lambda x: amplitude * math.sin(2 * math.pi / period * x)
        R_VEL = lambda x: amplitude * 2 * math.pi / period * math.cos(x*2 * math.pi / period)
        
        LEFT = lambda x: -amplitude * math.sin(2 * math.pi / period * x + math.pi)
        L_VEL = lambda x: amplitude * 2 * math.pi / period * math.cos(x*2 * math.pi / period)
        while elapsed_time < 3:
            # 오른쪽 다리 (0도에서 )
            
            right_pos = RIGHT(elapsed_time)
            right_angle = right_pos * 180 / math.pi
            right_vel = R_VEL(elapsed_time)
            
            # 왼쪽 다리 (반대 위상)
            left_pos = LEFT(elapsed_time)
            left_angle = left_pos * 180 / math.pi
            left_vel = L_VEL(elapsed_time)
            
            superior.set_position(1, right_pos)
            superior.set_velocity(1, right_vel)
            
            superior.set_position(2, left_pos)
            superior.set_velocity(2, -left_vel)
            
            print(f"right_pos: {right_pos:.2f}, right_angle: {right_angle:.2f}, right_vel: {right_vel:.2f}")
            print(f"left_pos: {left_pos:.2f}, left_angle: {left_angle:.2f}, left_vel: {left_vel:.2f}")
            
            elapsed_time = time.time() - start_time
    except:
        print("오류 발생")
        superior.stop()
        time.sleep(0.1)
        superior.add_motor_to_zero(1)
        superior.add_motor_to_zero(2)
        
        raise
    finally:
        # 모터를 정지시키고 원점으로 이동
        print("모터 원점(처음 구동시점)으로 이동 후 정지")
        
        # superior.stop()
        
        superior.add_motor_to_zero(1)
        superior.add_motor_to_zero(2)
        
    # 원점으로 이동
    
    
def slow_walk(superior:RobstrideMotorsSupervisor, 
              period: float = 2.0, amplitude: float = 1.0):
    """느린 걷기 시나리오"""
    print(f"느린 걷기 시작 (주기: {period}초, 진폭: {amplitude})")
    print(f"빠른 걷기 시작 (주기: {period}초, 진폭: {amplitude})")
    
    start_time = time.time()
    # 3초동안 작동
    try:
        elapsed_time = time.time() - start_time
        RIGHT = lambda x: amplitude * math.sin(2 * math.pi / period * x)
        R_VEL = lambda x: amplitude * 2 * math.pi / period * math.cos(x*2 * math.pi / period)
        
        LEFT = lambda x: -amplitude * math.sin(2 * math.pi / period * x + math.pi)
        L_VEL = lambda x: amplitude * 2 * math.pi / period * math.cos(x*2 * math.pi / period)
        while elapsed_time < 3:
            # 오른쪽 다리 (0도에서 )
            
            right_pos = RIGHT(elapsed_time)
            right_angle = right_pos * 180 / math.pi
            right_vel = R_VEL(elapsed_time)
            
            # 왼쪽 다리 (반대 위상)
            left_pos = LEFT(elapsed_time)
            left_angle = left_pos * 180 / math.pi
            left_vel = L_VEL(elapsed_time)
            
            superior.set_position(1, right_pos)
            superior.set_velocity(1, right_vel)
            
            superior.set_position(2, left_pos)
            superior.set_velocity(2, -left_vel)
            
            print(f"right_pos: {right_pos:.2f}, right_angle: {right_angle:.2f}, right_vel: {right_vel:.2f}")
            print(f"left_pos: {left_pos:.2f}, left_angle: {left_angle:.2f}, left_vel: {left_vel:.2f}")
            
            elapsed_time = time.time() - start_time
    except:
        print("오류 발생")
        superior.stop()
        time.sleep(0.1)
        superior.add_motor_to_zero(1)
        superior.add_motor_to_zero(2)
        
        raise
    finally:
        # 모터를 정지시키고 원점으로 이동
        print("모터 원점(처음 구동시점)으로 이동 후 정지")
        
        superior.add_motor_to_zero(1)
        superior.add_motor_to_zero(2)
        
        # superior.stop()
        
def right_leg_injury(superior:RobstrideMotorsSupervisor, 
                     right_angle: float = 45.0, 
                     left_angle: float = 90.0, 
                     period: float = 2.0, 
                     amplitude: float = 1.0):
    """오른쪽 다리 부상 시나리오""" 
    print(f"오른쪽 다리 부상 시뮬레이션 시작 (오른쪽: {right_angle}도, 왼쪽: {left_angle}도)")
    start_time = time.time()
    
    while True:
        elapsed_time = time.time() - start_time
        
        # 오른쪽 다리
        right_pos = (amplitude/4) * (math.sin(2 * math.pi / period * elapsed_time))
        right_vel = (amplitude/4) * (2 * math.pi / period) * math.cos(2 * math.pi / period)
        
        # 왼쪽 다리
        left_pos = -(amplitude) * (math.sin(2 * math.pi / period * elapsed_time + math.pi))
        left_vel = (amplitude) * (2 * math.pi / period) * math.cos(2 * math.pi / period)
        
        right_angle = right_pos * 180 / math.pi
        left_angle = left_pos * 180 / math.pi
        
        superior.set_position(1, right_pos)
        superior.set_position(2, left_pos)
        
        superior.set_velocity(1, right_vel)
        superior.set_velocity(2, left_vel)
        
        print(f"right_pos: {right_pos:.2f}, right_angle: {right_angle:.2f}, right_vel: {right_vel:.2f}")
        print(f"left_pos: {left_pos:.2f}, left_angle: {left_angle:.2f}, left_vel: {left_vel:.2f}")
        print(f"torque: {superior.get_torque(1):.2f}, {superior.get_torque(2):.2f}")
        

def left_leg_injury(superior: RobstrideMotorsSupervisor, 
                    period: float = 2.0, 
                    amplitude: float = 1.0):
    """왼쪽 다리 부상 시나리오"""
    print(f"왼쪽 다리 부상 시뮬레이션 시작 (주기: {period}초, 진폭: {amplitude})")
    start_time = time.time()
    
    while True:
        try:
            elapsed_time = time.time() - start_time
            
            # 오른쪽 다리 (정상, 0도에서 right_angle도로 왔다갔다)
            right_pos = (amplitude) * (math.sin(2 * math.pi / period * elapsed_time))
            right_vel = (amplitude) * (2 * math.pi / period) * math.cos(2 * math.pi / period)
            
            # 왼쪽 다리 (제한된 움직임, 0도에서 left_angle도로 왔다갔다)
            left_pos = (amplitude/4) * (math.sin(2 * math.pi / period * elapsed_time + math.pi))
            left_vel = (amplitude/4) * (2 * math.pi / period) * math.cos(2 * math.pi / period)
            
            right_angle = right_pos * 180 / math.pi
            left_angle = left_pos * 180 / math.pi
            
            superior.set_position(1, right_pos)
            superior.set_position(2, left_pos)
            
            superior.set_velocity(1, right_vel)
            superior.set_velocity(2, left_vel)  
            
            print(f"right_pos: {right_pos:.2f}, right_angle: {right_angle:.2f}, right_vel: {right_vel:.2f}")
            print(f"left_pos: {left_pos:.2f}, left_angle: {left_angle:.2f}, left_vel: {left_vel:.2f}")
            print(f"torque: {superior.get_torque(1):.2f}, {superior.get_torque(2):.2f}")
        except:
            print("오류 발생")
            superior.stop()
            time.sleep(0.1)
            superior.add_motor_to_zero(1)
            superior.add_motor_to_zero(2)
            raise

def main() -> None:
    parser = argparse.ArgumentParser(description="모터 제어 시나리오 테스트")
    parser.add_argument("-v", "--verbose", action="store_true", help="상세 출력 활성화")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="시리얼 포트 이름")
    parser.add_argument("--period", type=float, default=2.0, help="주기 (초)")
    parser.add_argument("--amplitude", type=float, default=1, help="진폭 (기본값: 1.0)")
    parser.add_argument("--kp", type=float, default=10.0, help="kp (기본값: 10.0)")
    parser.add_argument("--kd", type=float, default=1.0, help="kd (기본값: 1.0)")
    parser.add_argument("--motor-id", type=int, default=1, help="모터 ID (기본값: 1)")
    parser.add_argument("--motor-type", type=str, default="01", help="모터 타입 (기본값: 01)")
    parser.add_argument("--second-motor-id", type=int, default=2, help="모터 ID (기본값: 2)")
    parser.add_argument("--second-motor-type", type=str, default="01", help="모터 타입 (기본값: 01)")
    parser.add_argument("--sleep", type=float, default=0.00, help="sleep (기본값: 0.00)")
    
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
    
    superior.set_kp(args.motor_id, args.kp)
    superior.set_kd(args.motor_id, args.kd)
    
    superior.set_kp(args.second_motor_id, args.kp)
    superior.set_kd(args.second_motor_id, args.kd)
    
    try:
        start_time = time.time()
        #시간에 따라 다리 움직임을 조절
        # Ex) 3초 마다 다리 움직임을 조절
        # 3초동안은 빠르게 걷기, 3초동안은 느리게 걷기
        # 3초동안은 오른쪽 다리 부상, 3초동안은 왼쪽 다리 부상
        
        
        while(True):
            # 예시: 1초마다 다리 움직임을 조절
            # 1초동안은 빠르게 걷기, 1초동안은 느리게 걷기
            # 1초동안은 오른쪽 다리 부상, 1초동안은 왼쪽 다리 부상
            fast_walk(superior, args.period, args.amplitude) # %0
            
            
            slow_walk(superior, args.period, args.amplitude) # %1
            
            # right_leg_injury(superior, args.period, args.amplitude) # %2
        
            # left_leg_injury(superior, args.period, args.amplitude) # %3
    
                
    except Exception as e:
        print(f"오류 발생: {e}")
        superior.stop()
        superior.add_motor_to_zero(args.motor_id)
        superior.add_motor_to_zero(args.second_motor_id)
        raise
    

if __name__ == "__main__":
    main()
