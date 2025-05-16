#pragma once

#include "MotorTypes.h"
#include <string>
#include <map>
#include <mutex>
#include <atomic>
#include <chrono>
#include <thread>
#include <memory>
#include <vector>
#include <iomanip>
#include "SerialPort.h"
#include <functional>

// 통신 타입 정의
#define COMMUNICATION_TYPE_GET_ID 0x00
#define COMMUNICATION_TYPE_MOTION_CONTROL 0x01
#define COMMUNICATION_TYPE_MOTOR_REQUEST 0x02
#define COMMUNICATION_TYPE_MOTOR_ENABLE 0x03
#define COMMUNICATION_TYPE_MOTOR_STOP 0x04
#define COMMUNICATION_TYPE_SET_POS_ZERO 0x06
#define COMMUNICATION_TYPE_CAN_ID 0x07
#define COMMUNICATION_TYPE_CONTROL_MODE 0x12
#define COMMUNICATION_TYPE_GET_SINGLE_PARAMETER 0x11
#define COMMUNICATION_TYPE_SET_SINGLE_PARAMETER 0x12
#define COMMUNICATION_TYPE_ERROR_FEEDBACK 0x15

class MotorController {
public:
    MotorController(
        const std::string& port_name,
        const std::map<uint8_t, MotorType>& motor_mapping,
        float kp = 10.0f,
        float kd = 1.0f,
        float target_update_rate = 200.0f,
        float can_timeout = 0.1f
    );
    ~MotorController();

    // 기본 제어 메서드
    float setPosition(uint8_t motor_id, float position);
    float getPosition(uint8_t motor_id) const;
    float setVelocity(uint8_t motor_id, float velocity);
    float getVelocity(uint8_t motor_id) const;
    float setKp(uint8_t motor_id, float kp);
    float getKp(uint8_t motor_id) const;
    float setKd(uint8_t motor_id, float kd);
    float getKd(uint8_t motor_id) const;
    float setTorque(uint8_t motor_id, float torque);
    float getTorque(uint8_t motor_id) const;

    // 상태 관리 메서드
    std::map<uint8_t, MotorFeedback> getLatestFeedback() const;
    void togglePause();
    void stop();
    bool isRunning() const;
    void reset();

    // 설정 메서드
    void setMaxUpdateRate(float rate);
    float getActualUpdateRate() const;
    bool toggleSerial();
    bool getSerial() const;

    // 모터 제어 메서드
    void addMotorToZero(uint8_t motor_id);
    void setParams(uint8_t motor_id, const MotorControlParams& params);

    // 명령 카운터 관련
    uint64_t getTotalCommands() const;
    uint64_t getFailedCommands(uint8_t motor_id) const;
    void resetCommandCounters();

    // CAN 통신 메서드
    std::map<uint8_t, RunMode> sendGetMode();
    void sendSetZeros(const std::vector<uint8_t>& motor_ids);
    void sendResets();
    void sendStarts();
    std::map<uint8_t, MotorFeedback> sendMotorControls(const std::map<uint8_t, MotorControlParams>& params = {}, bool serial = false);
    std::string readStringParam(uint8_t motor_id, uint16_t index, uint8_t num_packs);
    uint16_t readUint16Param(uint8_t motor_id, uint16_t index);
    std::map<uint8_t, std::string> readNames();
    std::map<uint8_t, std::string> readBarCodes();
    std::map<uint8_t, std::string> readBuildDates();
    std::map<uint8_t, float> readCanTimeouts();
    void sendCanTimeout(float timeout);

    // Robstrite 메서드 추가
    void setMotorMode(uint8_t motor_id, RunMode mode);
    void setMotorParameter(uint8_t motor_id, uint16_t index, float value, char value_mode);
    void getMotorParameter(uint8_t motor_id, uint16_t index);
    void setCanId(uint8_t motor_id, uint8_t new_can_id);
    void setZeroPosition(uint8_t motor_id);
    void enableMotor(uint8_t motor_id);
    void disableMotor(uint8_t motor_id, bool clear_error = false);
    void moveControl(uint8_t motor_id, float torque, float angle, float speed, float kp, float kd);
    void positionControl(uint8_t motor_id, float speed, float acceleration, float angle);
    void speedControl(uint8_t motor_id, float speed, float acceleration, float limit_current);
    void currentControl(uint8_t motor_id, float current);

private:
    // 내부 상태 변수
    std::string port_name_;
    std::map<uint8_t, MotorType> motor_mapping_;
    std::map<uint8_t, MotorConfig> motor_configs_;
    std::map<uint8_t, MotorControlParams> target_params_;
    std::map<uint8_t, MotorFeedback> latest_feedback_;
    std::map<uint8_t, uint64_t> failed_commands_;
    std::vector<uint8_t> motors_to_zero_;
    
    std::unique_ptr<SerialPort> port_;
    mutable std::mutex mutex_;
    
    bool running_;
    bool paused_;
    bool restart_;
    bool serial_;
    uint64_t total_commands_;
    float max_update_rate_;
    float actual_update_rate_;
    float can_timeout_;
    RunMode current_mode_;
    
    // 내부 메서드
    void startControlThread();
    void controlLoop();
    void sendReset();
    void sendStart();
    void sendSetZero();
    
    // CAN 통신 헬퍼 메서드
    CanPack packMotorParams(uint8_t id, const MotorControlParams& params);
    MotorFeedback unpackFeedback(const CanPack& pack);
    uint32_t packBits(const std::vector<uint32_t>& values, const std::vector<uint8_t>& bit_lengths);
    std::vector<uint32_t> unpackBits(uint32_t value, const std::vector<uint8_t>& bit_lengths);
    std::array<uint8_t, 4> packExId(const ExId& ex_id);
    ExId unpackExId(const std::array<uint8_t, 4>& addr);
    void txPacks(const std::vector<CanPack>& packs, bool verbose);
    std::vector<CanPack> rxUnpack(size_t len, bool verbose);

    // Robstrite 관련 멤버 변수 추가
    static const uint16_t INDEX_LIST[];
    uint8_t master_can_id_ = 0x1F;
    std::map<uint8_t, MotorSet> motor_sets_;

    // 헬퍼 함수들 추가
    float clampValue(float value, float min_val, float max_val) const;
    void sendCanCommand(uint8_t motor_id, CanComMode mode, const std::vector<uint8_t>& data);
    void updateMotorState(uint8_t motor_id, const MotorControlParams& params);
    bool validateMotorId(uint8_t motor_id) const;
    
    // 중복 제거를 위한 통합 제어 함수
    void controlMotor(uint8_t motor_id, RunMode mode, const std::function<void(MotorSet&)>& update_params);
};

