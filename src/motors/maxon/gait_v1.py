import serial
import time
from ctypes import *
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from examples.maxon_motor.gui import EPOSConstants

ABSOLUTE = 1
RELATIVE = 0
class CONST:
    # EPOS Command Library 경로
    path = './EposCmd64.dll'
    # 라이브러리 로드
    cdll.LoadLibrary(path)
    epos = CDLL(path)
    # 라이브러리 함수에서 반환되는 변수 정의
    pErrorCode = c_uint()
    
    # NodeID 변수 정의 및 연결 구성
    nodeID = 1
    baudrate = 1000000
    timeout = 500
    start_position = 0
    target_position = 0
    move_type_var = ABSOLUTE  # 1=absolute, 0=relative
    
    # 모터 설정값
    mode = c_byte()
    velocity = 0
    acceleration = 0
    deceleration = 0
    
    default_velocity = 100
    default_acceleration = 5000 # rpm/s
    default_deceleration = 5000 # rpm/s
    # 원하는 모션 프로파일 구성
    immediately = True  # 즉시 이동
    # 위치 프로파일 값 읽기, 처음인 경우
    initialization = True
    keyHandle = 0


def show_error_information(error_code:c_int)->None:
    """오류 코드에 대한 정보를 표시합니다."""
    error_code = c_uint(error_code)
    error_info = create_string_buffer(256)
    CONST.epos.VCS_GetErrorInfo(error_code, error_info, sizeof(error_info))
    print(f"오류 발생: {error_info.value.decode('utf-8')}")

def get_position_is() -> c_long:
    """현재 위치 읽기"""
    position_is = c_long()
    if not CONST.epos.VCS_GetPositionIs(CONST.keyHandle, CONST.nodeID, byref(position_is), byref(CONST.pErrorCode)):
        show_error_information(CONST.pErrorCode.value)
        return None
    return position_is.value   # 모터 스텝

def move_to_position(target_position:int, absolute=True) -> bool:
    """목표 위치로 이동"""
    # 현재 위치 읽기
    start_position = get_position_is()
    if start_position is None:
        return False
    
    # 이동 명령 실행
    if not CONST.epos.VCS_MoveToPosition(CONST.keyHandle, 
                                         CONST.nodeID, 
                                         target_position, 
                                         absolute, 
                                         CONST.immediately, 
                                         byref(CONST.pErrorCode)):
        show_error_information(CONST.pErrorCode.value)
        return False
    
    return True


def open_device() -> bool:
    """EPOS 장치 연결"""
    # 만약 열려 있으면, 이전 연결 닫기
    if CONST.keyHandle:
        CONST.epos.VCS_CloseDevice(CONST.keyHandle, 
                                   byref(CONST.pErrorCode))
        CONST.keyHandle = 0

    # USB 연결
    CONST.keyHandle = CONST.epos.VCS_OpenDevice(b'EPOS2', 
                                          b'MAXON SERIAL V2', 
                                          b'USB', 
                                          b'USB0', 
                                          byref(CONST.pErrorCode))

    if not CONST.keyHandle:
        show_error_information(CONST.pErrorCode.value)
        return False

    # 통신 설정
    if not CONST.epos.VCS_SetProtocolStackSettings(CONST.keyHandle, 
                                                   CONST.baudrate, 
                                                   byref(CONST.timeout), 
                                                   byref(CONST.pErrorCode)):
        show_error_information(CONST.pErrorCode.value)
        return False

    # 오류 초기화
    if not CONST.epos.VCS_ClearFault(CONST.keyHandle, 
                                     CONST.nodeID, 
                                     byref(CONST.pErrorCode)):
        show_error_information(CONST.pErrorCode.value)
        return False

    # 작동 모드 확인
    if not CONST.epos.VCS_GetOperationMode(CONST.keyHandle, 
                                           CONST.nodeID, 
                                           byref(CONST.mode), 
                                           byref(CONST.pErrorCode)):
        show_error_information(CONST.pErrorCode.value)
        return False
    CONST.mode = CONST.mode.value
    # 위치 프로파일 값 읽기
    if not CONST.epos.VCS_SetPositionProfile(CONST.keyHandle, 
                                             CONST.nodeID, 
                                             CONST.velocity, 
                                             CONST.acceleration, 
                                             CONST.deceleration, 
                                             byref(CONST.pErrorCode)):
        show_error_information(CONST.pErrorCode.value)
        return False

    # 작동 모드를 위치 프로파일 모드로 설정
    if not CONST.epos.VCS_SetOperationMode(CONST.keyHandle, 
                                           CONST.nodeID, 
                                           EPOSConstants.OMD_PROFILE_POSITION_MODE, 
                                           byref(CONST.pErrorCode)):
        show_error_information(CONST.pErrorCode.value)
        return False

    # 현재 위치 읽기
    start_position = get_position_is().value
    CONST.start_position = start_position
    CONST.initialization = True
    return True


def set_position_profile(velocity:int, acceleration:int, deceleration:int)-> bool:
    """위치 프로파일 설정"""
    if not CONST.epos.VCS_SetPositionProfile(CONST.keyHandle, CONST.nodeID, velocity, acceleration, deceleration, byref(CONST.pErrorCode)):
        show_error_information(CONST.pErrorCode)
        return False
    
    # 설정 값 저장
    CONST.velocity = velocity
    CONST.acceleration = acceleration
    CONST.deceleration = deceleration
    
    return True

def halt_position_movement()-> bool:
    """이동 정지"""
    if not CONST.epos.VCS_HaltPositionMovement(keyHandle, CONST.nodeID, byref(CONST.pErrorCode)):
        show_error_information(CONST.pErrorCode)
        return False
    return True


def set_enable_state():
    """모터 활성화"""
    # 오류 상태 확인
    fault = c_int(0)
    if not CONST.epos.VCS_GetFaultState(CONST.keyHandle, CONST.nodeID, byref(fault), byref(CONST.pErrorCode)):
        show_error_information(CONST.pErrorCode.value)
        return False

    # 오류가 있으면 초기화
    if fault.value:
        if not CONST.epos.VCS_ClearFault(CONST.keyHandle, CONST.nodeID, byref(CONST.pErrorCode)):
            show_error_information(CONST.pErrorCode.value)
            return False

    # 모터 활성화
    if not CONST.epos.VCS_SetEnableState(CONST.keyHandle, CONST.nodeID, byref(CONST.pErrorCode)):
        show_error_information(CONST.pErrorCode.value)
        return False
    
    return True


# 각도를 모터 위치로 변환 (각도 → 스텝)
def AngleToPosition(angle):
    # HEDL 5540 엔코더에 맞게 업데이트된 값
    counts_per_revolution = 500  # HEDL 5540의 기본 CPT
    quadrature_factor = 4  # 쿼드러처 카운팅 사용 시 (일반적)
    steps_per_revolution = counts_per_revolution * quadrature_factor  # 2000 스텝/회전
    
    # 감속기 없음 - 기어비 1:1
    gear_ratio = 1
    
    # 각도를 모터 회전으로 변환
    motor_revs = (angle / 360.0) * gear_ratio
    
    # 모터 회전을 엔코더 스텝으로 변환
    position = int(motor_revs * steps_per_revolution)
    
    return position

# 모터 위치를 각도로 변환 (스텝 → 각도)
def PositionToAngle(position):
    # HEDL 5540 엔코더에 맞게 업데이트된 값
    counts_per_revolution = 500  # HEDL 5540의 기본 CPT
    quadrature_factor = 4  # 쿼드러처 카운팅 사용 시 (일반적)
    steps_per_revolution = counts_per_revolution * quadrature_factor  # 2000 스텝/회전
    
    # 감속기 없음 - 기어비 1:1
    gear_ratio = 1
    
    # 스텝을 회전으로 변환
    motor_revs = position / steps_per_revolution
    
    # 회전을 각도로 변환
    angle = (motor_revs / gear_ratio) * 360.0
    
    return angle

# 보행 주기 데이터를 모터 명령으로 변환하여 실행
def RunGaitCycle(data, cycle_duration=1.0, num_cycles=1):
    """
    보행 주기 데이터를 바탕으로 모터를 제어합니다.
    
    Parameters:
    -----------
    data : DataFrame
        보행 주기 데이터가 들어있는 데이터프레임 (time, angle 컬럼 필요)
    cycle_duration : float
        한 주기의 지속 시간(초)
    num_cycles : int
        반복할 주기 횟수
    """
    print(f"보행 주기 실행 시작: {num_cycles}회, 각 주기당 {cycle_duration}초")
    
    # 보행 주기 각도 범위 계산
    min_angle = data['angle'].min()
    max_angle = data['angle'].max()
    angle_range = max_angle - min_angle
    print(f"보행 주기 각도 범위: {min_angle:.2f}° ~ {max_angle:.2f}° (범위: {angle_range:.2f}°)")
    
    # 적절한 모터 속도 계산 (각도 범위에 따라 조정)
    # 감속기가 없으므로 속도를 더 높게 설정하여 토크 확보
    base_speed = 3000  # 기본 rpm
    
    # 각 주기 간격 계산
    step_duration = cycle_duration / (len(data) - 1)
    
    # 시작 위치 설정 (첫 번째 각도)
    start_angle = data.iloc[0]['angle']
    start_position = AngleToPosition(start_angle)
    
    # 시작 위치로 이동
    print(f"시작 위치로 이동 중: {start_angle:.2f}도")
    MoveToPositionSpeed(start_position, base_speed)
    time.sleep(0.5)  # 0.5초 대기
    
    # 결과 저장을 위한 배열
    times = []
    commanded_angles = []
    actual_positions = []
    
    # 주기 실행
    for cycle in range(num_cycles):
        print(f"주기 {cycle+1}/{num_cycles} 실행 중")
        
        cycle_start_time = time.time()
        
        # 보행 주기의 각 지점 실행
        for i, row in data.iterrows():
            # 주기 내 현재 시간 계산
            current_time = time.time() - cycle_start_time
            
            # 목표 각도 및 위치 계산
            target_angle = row['angle']
            target_position = AngleToPosition(target_angle)
            
            # 속도 계산 (다음 지점과의 각도 차이 기반)
            if i < len(data) - 1:
                next_angle = data.iloc[i+1]['angle']
                angle_diff = abs(next_angle - target_angle)
                # 각도 변화를 속도로 변환 (큰 변화 = 빠른 속도)
                # 감속기가 없으므로 훨씬 더 높은 속도 범위 사용
                target_speed = max(1000, min(6000, int(angle_diff * 1000)))
            else:
                target_speed = 2000  # 마지막 지점
            
            # 목표 위치로 이동
            MoveToPositionSpeed(target_position, target_speed)
            
            # 실제 위치 확인
            actual_position = get_position_is()
            
            # 결과 저장
            times.append(cycle * cycle_duration + current_time)
            commanded_angles.append(target_angle)
            actual_positions.append(actual_position)
            
            # 남은 시간 계산 및 대기
            elapsed = time.time() - cycle_start_time
            target_elapsed = i * step_duration
            if target_elapsed > elapsed:
                time.sleep(target_elapsed - elapsed)
        
        # 주기 종료 시간 확인
        cycle_elapsed = time.time() - cycle_start_time
        print(f"주기 {cycle+1} 완료: {cycle_elapsed:.2f}초 소요")
    
    # 결과 데이터프레임 생성
    results = pd.DataFrame({
        'time': times,
        'commanded_angle': commanded_angles,
        'actual_position': actual_positions,
        'actual_angle': [PositionToAngle(pos) for pos in actual_positions]
    })
    
    return results

# 결과 시각화
def PlotResults(results):
    """보행 주기 실행 결과를 시각화합니다."""
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
    
    # 각도 플롯
    ax1.plot(results['time'], results['commanded_angle'], 'b-', label='목표 각도')
    ax1.plot(results['time'], results['actual_angle'], 'r--', label='실제 각도')
    ax1.set_ylabel('각도 (도)')
    ax1.set_title('보행 주기 동안의 고관절 각도')
    ax1.legend()
    ax1.grid(True)
    
    # 위치 플롯
    ax2.plot(results['time'], results['actual_position'], 'g-', label='엔코더 위치')
    ax2.set_xlabel('시간 (초)')
    ax2.set_ylabel('위치 (스텝)')
    ax2.set_title('모터 엔코더 위치')
    ax2.grid(True)
    ax2.legend()
    
    plt.tight_layout()
    plt.savefig('gait_cycle_results.png')
    plt.show()
    
    return fig





if __name__ == "__main__":
    if open_device():
        print("장치가 성공적으로 연결되었습니다.")
    else:
        raise RuntimeError("장치를 연결할 수 없습니다.")
    
    print(f"현재 위치: {get_position_is()}")
    #match-case구문을 이용해서 할 것.
    # 동작 입력
    # 1. 모터 이동
    # 2. 모터 설정값 변경
    # 3. 모터 정지
    # 4. 모터 활성화
    # 5. 모터 비활성화
    # 6. 종료
    
    target_position = input("모터 이동 (각도): ")
    # 모터 설정값 변경
    target_velocity, target_acceleration, target_deceleration = input(f"모터 설정값 변경 (현재: {CONST.velocity} rpm, {CONST.acceleration} rpm/s, {CONST.deceleration} rpm/s): ").split()
    
    
    while set_position_profile(int(target_velocity), int(target_acceleration), int(target_deceleration)):
        pass
    
    if move_to_position(target_position, CONST.move_type_var):
        print(f"모터 이동 성공: {target_position}")
    else:
        print(f"모터 이동 실패: {target_position}")