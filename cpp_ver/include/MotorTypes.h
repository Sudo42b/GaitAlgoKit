#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <array>

// CAN 통신 모드 열거형
enum class CanComMode : uint8_t {
    AnnounceDevId = 0,
    MotorCtrl,
    MotorFeedback,
    MotorIn,
    MotorReset,
    MotorCali,
    MotorZero,
    MotorId,
    ParaWrite,
    ParaRead,
    ParaUpdate,
    OtaStart,
    OtaInfo,
    OtaIng,
    OtaEnd,
    CaliIng,
    CaliRst,
    SdoRead,
    SdoWrite,
    ParaStrInfo,
    MotorBrake,
    FaultWarn,
    ModeTotal
};

// 모터 모드 열거형
enum class MotorMode : uint8_t {
    MOVE_CONTROL = 0,
    POSITION_CONTROL = 1,
    SPEED_CONTROL = 2,
    CURRENT_CONTROL = 3,
    SET_ZERO = 4
};

// 실행 모드 열거형
enum class RunMode : int8_t {
    UnsetMode = -1,
    MitMode = 0,
    PositionMode = 1,
    SpeedMode = 2,
    CurrentMode = 3,
    ToZeroMode = 4,
    CspPositionMode = 5
};

// 모터 타입 열거형
enum class MotorType {
    UNKNOWN = 0,
    Type01,
    Type02,
    Type03,
    Type04
};

// CAN ID 상수
constexpr uint8_t CAN_ID_MASTER = 0x00;
constexpr uint8_t CAN_ID_MOTOR_DEFAULT = 0x7F;
constexpr uint8_t CAN_ID_BROADCAST = 0xFE;
constexpr uint8_t CAN_ID_DEBUG_UI = 0xFD;

// 확장 ID 구조체
struct ExId {
    uint8_t id;
    uint16_t data;
    CanComMode mode;
    uint8_t res;
};

// CAN 패킷 구조체
struct CanPack {
    ExId ex_id;
    uint8_t len;
    std::vector<uint8_t> data;
};

// 모터 피드백 구조체
struct MotorFeedback {
    uint8_t can_id;
    float position;
    float velocity;
    float torque;
    RunMode mode;
    uint16_t faults;
};
// 모터 제어 파라미터 구조체
struct MotorControlParams {
    float position = 0.0f;
    float velocity = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float torque = 0.0f;
};


// 모터 설정 구조체
struct MotorConfig {
    float p_min;
    float p_max;
    float v_min;
    float v_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
    float t_min;
    float t_max;
    bool zero_on_init;
    uint16_t can_timeout_command;
}; 

// MotorSet 구조체 추가
struct MotorSet {
    int set_motor_mode = 0;
    float set_current = 0.0f;
    float set_speed = 0.0f;
    float set_acceleration = 0.0f;
    float set_torque = 0.0f;
    float set_angle = 0.0f;
    float set_limit_cur = 0.0f;
    float set_kp = 0.0f;
    float set_ki = 0.0f;
    float set_kd = 0.0f;
}; 