#include "../include/Robstrite.h"
#include "../include/MotorTypes.h"
#include <windows.h>
#include <iostream>

// CAN to USB 어댑터 핸들
extern HANDLE hDevice;

// data_read_write 생성자 구현
data_read_write::data_read_write(const uint16_t *index_list) {
    run_mode.index = index_list[0];
    iq_ref.index = index_list[1];
    spd_ref.index = index_list[2];
    imit_torque.index = index_list[3];
    cur_kp.index = index_list[4];
    cur_ki.index = index_list[5];
    cur_filt_gain.index = index_list[6];
    loc_ref.index = index_list[7];
    limit_spd.index = index_list[8];
    limit_cur.index = index_list[9];
    mechPos.index = index_list[10];
    iqf.index = index_list[11];
    mechVel.index = index_list[12];
    VBUS.index = index_list[13];
    rotation.index = index_list[14];
}

// RobStrite_Motor 생성자 구현
RobStrite_Motor::RobStrite_Motor(uint8_t CAN_Id) {
    CAN_ID = CAN_Id;
    Master_CAN_ID = 0x1F;
    Motor_Offset_MotoFunc = nullptr;
    error_code = 0;
    Can_Motor = 0;
    output = 0.0f;
}

RobStrite_Motor::RobStrite_Motor(float (*Offset_MotoFunc)(float Motor_Tar), uint8_t CAN_Id) {
    CAN_ID = CAN_Id;
    Master_CAN_ID = 0x1F;
    Motor_Offset_MotoFunc = Offset_MotoFunc;
    error_code = 0;
    Can_Motor = 0;
    output = 0.0f;
}

// CAN 메시지 전송 함수
bool SendCANMessage(uint8_t* data, uint8_t length) {
    if (hDevice == INVALID_HANDLE_VALUE) {
        std::cout << "CAN 장치가 연결되지 않았습니다." << std::endl;
        return false;
    }

    DWORD bytesWritten;
    if (!WriteFile(hDevice, data, length, &bytesWritten, NULL)) {
        std::cout << "CAN 메시지 전송 실패" << std::endl;
        return false;
    }
    return true;
}

// CAN ID 가져오기
void RobStrite_Motor::RobStrite_Get_CAN_ID() {
    uint8_t txData[8] = {0};
    txData[0] = Communication_Type_Get_ID;
    SendCANMessage(txData, 8);
}

// 모터 파라미터 설정
void RobStrite_Motor::Set_RobStrite_Motor_parameter(uint16_t Index, float Value, char Value_mode) {
    uint8_t txData[8] = {0};
    txData[0] = Communication_Type_SetSingleParameter;
    txData[1] = Value_mode;
    txData[2] = (Index >> 8) & 0xFF;
    txData[3] = Index & 0xFF;
    
    // float 값을 바이트로 변환
    uint8_t* valueBytes = (uint8_t*)&Value;
    for(int i = 0; i < 4; i++) {
        txData[4 + i] = valueBytes[i];
    }
    
    SendCANMessage(txData, 8);
}

// 모터 파라미터 가져오기
void RobStrite_Motor::Get_RobStrite_Motor_parameter(uint16_t Index) {
    uint8_t txData[8] = {0};
    txData[0] = Communication_Type_GetSingleParameter;
    txData[1] = (Index >> 8) & 0xFF;
    txData[2] = Index & 0xFF;
    
    SendCANMessage(txData, 8);
}

// CAN 메시지 분석
void RobStrite_Motor::RobStrite_Motor_Analysis(uint8_t *DataFrame, uint32_t ID_ExtId) {
    uint8_t cmd = DataFrame[0];
    
    switch(cmd) {
        case Communication_Type_Get_ID:
            // ID 응답 처리
            break;
            
        case Communication_Type_MotorRequest:
            // 모터 상태 응답 처리
            Pos_Info.Angle = *(float*)(&DataFrame[1]);
            Pos_Info.Speed = *(float*)(&DataFrame[5]);
            break;
            
        case Communication_Type_ErrorFeedback:
            error_code = DataFrame[1];
            break;
            
        default:
            break;
    }
}

// 토크 모드 제어
void RobStrite_Motor::RobStrite_Motor_move_control(float Torque, 
                    float Angle, float Speed, float Kp, float Kd) {
    Motor_Set_All.set_Torque = Torque;
    Motor_Set_All.set_angle = Angle;
    Motor_Set_All.set_speed = Speed;
    Motor_Set_All.set_Kp = Kp;
    Motor_Set_All.set_Kd = Kd;
    
    // 파라미터 설정
    Set_RobStrite_Motor_parameter(drw.imit_torque.index, Torque, Set_parameter);
    Set_RobStrite_Motor_parameter(drw.loc_ref.index, Angle, Set_parameter);
    Set_RobStrite_Motor_parameter(drw.spd_ref.index, Speed, Set_parameter);
    Set_RobStrite_Motor_parameter(drw.cur_kp.index, Kp, Set_parameter);
    Set_RobStrite_Motor_parameter(drw.cur_ki.index, Kd, Set_parameter);
    
    // 모드 설정
    Set_RobStrite_Motor_parameter(drw.run_mode.index, move_control_mode, Set_mode);
}

// 위치 제어
void RobStrite_Motor::RobStrite_Motor_Pos_control(float Speed, float acceleration, float Angle) {
    Motor_Set_All.set_speed = Speed;
    Motor_Set_All.set_acceleration = acceleration;
    Motor_Set_All.set_angle = Angle;
    
    // 파라미터 설정
    Set_RobStrite_Motor_parameter(drw.spd_ref.index, Speed, Set_parameter);
    Set_RobStrite_Motor_parameter(drw.limit_spd.index, acceleration, Set_parameter);
    Set_RobStrite_Motor_parameter(drw.loc_ref.index, Angle, Set_parameter);
    
    // 모드 설정
    Set_RobStrite_Motor_parameter(drw.run_mode.index, Pos_control_mode, Set_mode);
}

// 속도 제어
void RobStrite_Motor::RobStrite_Motor_Speed_control(float Speed, float acceleration, float limit_cur) {
    Motor_Set_All.set_speed = Speed;
    Motor_Set_All.set_acceleration = acceleration;
    Motor_Set_All.set_limit_cur = limit_cur;
    
    // 파라미터 설정
    Set_RobStrite_Motor_parameter(drw.spd_ref.index, Speed, Set_parameter);
    Set_RobStrite_Motor_parameter(drw.limit_spd.index, acceleration, Set_parameter);
    Set_RobStrite_Motor_parameter(drw.limit_cur.index, limit_cur, Set_parameter);
    
    // 모드 설정
    Set_RobStrite_Motor_parameter(drw.run_mode.index, Speed_control_mode, Set_mode);
}

// 전류 제어
void RobStrite_Motor::RobStrite_Motor_current_control(float current) {
    Motor_Set_All.set_current = current;
    
    // 파라미터 설정
    Set_RobStrite_Motor_parameter(drw.iq_ref.index, current, Set_parameter);
    
    // 모드 설정
    Set_RobStrite_Motor_parameter(drw.run_mode.index, Elect_control_mode, Set_mode);
}

// 영점 설정 제어
void RobStrite_Motor::RobStrite_Motor_Set_Zero_control() {
    uint8_t txData[8] = {0};
    txData[0] = Communication_Type_SetPosZero;
    SendCANMessage(txData, 8);
}

// 모터 활성화
void RobStrite_Motor::Enable_Motor() {
    uint8_t txData[8] = {0};
    txData[0] = Communication_Type_MotorEnable;
    SendCANMessage(txData, 8);
}

// 모터 비활성화
void RobStrite_Motor::Disenable_Motor(uint8_t clear_error) {
    uint8_t txData[8] = {0};
    txData[0] = Communication_Type_MotorStop;
    txData[1] = clear_error;
    SendCANMessage(txData, 8);
}

// CAN ID 설정
void RobStrite_Motor::Set_CAN_ID(uint8_t Set_CAN_ID) {
    uint8_t txData[8] = {0};
    txData[0] = Communication_Type_Can_ID;
    txData[1] = Set_CAN_ID;
    SendCANMessage(txData, 8);
    
    CAN_ID = Set_CAN_ID;
}

// 영점 위치 설정
void RobStrite_Motor::Set_ZeroPos() {
    RobStrite_Motor_Set_Zero_control();
}