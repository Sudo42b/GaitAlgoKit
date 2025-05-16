#pragma once

#include <stdint.h>
#include <string>
#include <functional>
#include "main.h"
#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

// 제어 모드 설정
#define Set_mode         'j'    // 제어 모드 설정
#define Set_parameter    'p'    // 파라미터 설정

// 제어 모드 정의
#define move_control_mode    0   // 토크 모드
#define Pos_control_mode     1   // 위치 모드
#define Speed_control_mode   2   // 속도 모드
#define Elect_control_mode   3   // 전류 모드
#define Set_Zero_mode        4   // 영점 모드

// 통신 주소 정의
#define Communication_Type_Get_ID             0x00    // 장치 ID 가져오기
#define Communication_Type_MotionControl      0x01    // 토크 모드 제어 명령
#define Communication_Type_MotorRequest       0x02    // 모터 상태 요청
#define Communication_Type_MotorEnable        0x03    // 모터 활성화
#define Communication_Type_MotorStop          0x04    // 모터 정지
#define Communication_Type_SetPosZero         0x06    // 현재 위치를 영점으로 설정
#define Communication_Type_Can_ID             0x07    // 현재 모터의 CAN ID 설정
#define Communication_Type_Control_Mode       0x12    // 모터 모드 설정
#define Communication_Type_GetSingleParameter 0x11    // 단일 파라미터 가져오기
#define Communication_Type_SetSingleParameter 0x12    // 단일 파라미터 설정
#define Communication_Type_ErrorFeedback      0x15    // 오류 피드백

#ifdef __cplusplus
class data_read_write_one
{
public:
    uint16_t index;
    float data;
};

static const uint16_t Index_List[] = {
    0X7005, 0X7006, 0X700A, 0X700B, 0X7010, 0X7011, 0X7014, 
    0X7016, 0X7017, 0X7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x701D
};

// 18개 통신 제어 가능한 파라미터 리스트
class data_read_write
{
public:
    data_read_write_one run_mode;         // 0:토크모드 1:위치모드 2:속도모드 3:전류모드 4:영점모드
    data_read_write_one iq_ref;           // 전류모드 Iq 지령
    data_read_write_one spd_ref;          // 속도모드 속도 지령
    data_read_write_one imit_torque;      // 토크 제한
    data_read_write_one cur_kp;           // 전류 제어기 Kp
    data_read_write_one cur_ki;           // 전류 제어기 Ki
    data_read_write_one cur_filt_gain;    // 전류 필터 계수
    data_read_write_one loc_ref;          // 위치모드 각도 지령
    data_read_write_one limit_spd;        // 위치모드 속도 제한
    data_read_write_one limit_cur;        // 속도/위치모드 전류 제한
    
    // 읽기 전용
    data_read_write_one mechPos;          // 모터축 기계적 각도
    data_read_write_one iqf;              // iq 필터값
    data_read_write_one mechVel;          // 모터축 속도
    data_read_write_one VBUS;             // 모선 전압
    data_read_write_one rotation;         // 회전수
    
    data_read_write(const uint16_t *index_list = Index_List);
};

typedef struct {
    float Angle;
    float Speed;
    float acceleration;
    float Torque;
    float Temp;
    int pattern;  // 모드 패턴 (0:위치 1:교정 2:보정)
} Motor_Pos_RobStrite_Info;

typedef struct {
    int set_motor_mode;
    float set_current;
    float set_speed;
    float set_acceleration;
    float set_Torque;
    float set_angle;
    float set_limit_cur;
    float set_Kp;
    float set_Ki;
    float set_Kd;
} Motor_Set;

class RobStrite_Motor
{
private:
    uint8_t CAN_ID;           // CAN ID (기본값 127(0x7f))
    uint16_t Master_CAN_ID;   // 마스터 ID (초기화 시 0x1F로 설정)
    float (*Motor_Offset_MotoFunc)(float Motor_Tar);
    Motor_Set Motor_Set_All;  // 설정값
    uint8_t error_code;

public:
    float output;
    int Can_Motor;
    Motor_Pos_RobStrite_Info Pos_Info;  // 피드백값
    data_read_write drw;                // 데이터 구조체
    
    RobStrite_Motor(uint8_t CAN_Id);
    RobStrite_Motor(float (*Offset_MotoFunc)(float Motor_Tar), uint8_t CAN_Id);
    void RobStrite_Get_CAN_ID();
    void Set_RobStrite_Motor_parameter(uint16_t Index, float Value, char Value_mode);
    void Get_RobStrite_Motor_parameter(uint16_t Index);
    void RobStrite_Motor_Analysis(uint8_t *DataFrame, uint32_t ID_ExtId);
    void RobStrite_Motor_move_control(float Torque, float Angle, float Speed, float Kp, float Kd);
    void RobStrite_Motor_Pos_control(float Speed, float acceleration, float Angle);
    void RobStrite_Motor_Speed_control(float Speed, float acceleration, float limit_cur);
    void RobStrite_Motor_current_control(float current);
    void RobStrite_Motor_Set_Zero_control();
    void Enable_Motor();
    void Disenable_Motor(uint8_t clear_error);
    void Set_CAN_ID(uint8_t Set_CAN_ID);
    void Set_ZeroPos();
};
#endif

#ifdef __cplusplus
}
#endif

#endif 