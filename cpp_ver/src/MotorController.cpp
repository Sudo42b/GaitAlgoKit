#include "../include/MotorController.h"
#include "../include/SerialPort.h"
#include "../include/device_handle.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <algorithm>
#include <cstring>

// 모터 설정 맵 초기화
static const std::map<MotorType, MotorConfig> ROBSTRIDE_CONFIGS = {
    {MotorType::Type01, {
        -12.5f, 12.5f,    // p_min, p_max
        -44.0f, 44.0f,    // v_min, v_max
        0.0f, 500.0f,     // kp_min, kp_max
        0.0f, 5.0f,       // kd_min, kd_max
        -12.0f, 12.0f,    // t_min, t_max
        true,             // zero_on_init
        0x200c            // can_timeout_command
    }},
    {MotorType::Type02, {
        -12.5f, 12.5f,
        -44.0f, 44.0f,
        0.0f, 500.0f,
        0.0f, 5.0f,
        -12.0f, 12.0f,
        false,
        0x200b
    }},
    {MotorType::Type03, {
        -12.5f, 12.5f,
        -20.0f, 20.0f,
        0.0f, 5000.0f,
        0.0f, 100.0f,
        -60.0f, 60.0f,
        false,
        0x200b
    }},
    {MotorType::Type04, {
        -12.5f, 12.5f,
        -15.0f, 15.0f,
        0.0f, 5000.0f,
        0.0f, 100.0f,
        -120.0f, 120.0f,
        false,
        0x200b
    }}
};

// INDEX_LIST 정의
const uint16_t MotorController::INDEX_LIST[] = {
    0X7005, 0X7006, 0X700A, 0X700B, 0X7010, 0X7011, 0X7014, 0X7016,
    0X7017, 0X7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x701D
};

// float를 uint로 변환하는 헬퍼 함수 추가
uint16_t floatToUint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x = x_max;
    else if(x < x_min) x = x_min;
    return static_cast<uint16_t>((x - offset) * ((1 << bits) - 1) / span);
}

MotorController::MotorController(
    const std::string& port_name,
    const std::map<uint8_t, MotorType>& motor_mapping,
    float kp,
    float kd,
    float target_update_rate,
    float can_timeout
) : port_name_(port_name),
    motor_mapping_(motor_mapping),
    running_(true),
    paused_(false),
    restart_(false),
    serial_(true),
    total_commands_(0),
    max_update_rate_(target_update_rate),
    actual_update_rate_(0.0f),
    can_timeout_(can_timeout),
    current_mode_(RunMode::UnsetMode)
{
    // 시리얼 포트는 이미 device_handle.cpp에서 초기화되었으므로
    // 여기서는 포인터만 설정
    port_ = std::make_unique<SerialPort>(port_name, 921600);
    
    // hDevice가 유효한지 확인
    if (hDevice < 0) {
        throw std::runtime_error("시리얼 포트가 초기화되지 않았습니다.");
    }
    
    port_->setHandle(hDevice);  // 이미 열린 핸들 사용

    // 모터 설정 초기화
    for (const auto& [id, type] : motor_mapping_) {
        if (ROBSTRIDE_CONFIGS.find(type) != ROBSTRIDE_CONFIGS.end()) {
            motor_configs_[id] = ROBSTRIDE_CONFIGS.at(type);
            target_params_[id] = MotorControlParams{0.0f, 0.0f, kp, kd, 0.0f};
            failed_commands_[id] = 0;
        }
    }

    // 제어 스레드 시작
    startControlThread();
}

MotorController::~MotorController() {
    stop();
}

void MotorController::startControlThread() {
    std::thread([this]() {
        controlLoop();
    }).detach();
}

void MotorController::controlLoop() {
    try {
        // 초기화
        sendResets();
        sendStarts();
        
        auto last_update_time = std::chrono::steady_clock::now();
        
        while (running_) {
            if (paused_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            
            if (restart_) {
                restart_ = false;
                sendResets();
                sendStarts();
            }
            
            // 제로 설정이 필요한 모터 처리
            if (!motors_to_zero_.empty()) {
                std::vector<uint8_t> motors_to_zero = motors_to_zero_;
                motors_to_zero_.clear();
                
                // 제로 토크 명령 전송
                std::map<uint8_t, MotorControlParams> zero_params;
                for (uint8_t id : motors_to_zero) {
                    zero_params[id] = MotorControlParams{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
                }
                auto feedbacks = sendMotorControls(zero_params, true);
                for (const auto& [id, feedback] : feedbacks) {
                    latest_feedback_[id] = feedback;
                }
                sendSetZeros(motors_to_zero);
            }
            
            // 모터 제어 명령 전송
            auto loop_start_time = std::chrono::steady_clock::now();
            auto feedbacks = sendMotorControls();
            for (const auto& [id, feedback] : feedbacks) {
                latest_feedback_[id] = feedback;
            }
            
            // 업데이트 레이트 계산
            auto elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(
                loop_start_time - last_update_time
            ).count();
            last_update_time = loop_start_time;
            
            float current_rate = 1.0f / elapsed;
            actual_update_rate_ = actual_update_rate_ * 0.9f + current_rate * 0.1f;
            
            // 업데이트 레이트 제어
            auto target_duration = std::chrono::duration<float>(1.0f / max_update_rate_);
            auto elapsed_loop = std::chrono::steady_clock::now() - loop_start_time;
            if (target_duration > elapsed_loop + std::chrono::microseconds(1)) {
                std::this_thread::sleep_for(target_duration - elapsed_loop);
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(1));
            }
        }
        
        // 종료 시 제로 토크 설정
        std::map<uint8_t, MotorControlParams> zero_params;
        for (const auto& [id, _] : target_params_) {
            zero_params[id] = MotorControlParams{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        }
        sendMotorControls(zero_params, true);
        sendResets();
        
    } catch (const std::exception& e) {
        std::cerr << "제어 루프 오류: " << e.what() << std::endl;
        running_ = false;
    }
}

// 기본 제어 메서드 구현
float MotorController::setPosition(uint8_t motor_id, float position) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = target_params_.find(motor_id); it != target_params_.end()) {
        it->second.position = position;
        return position;
    }
    throw std::runtime_error("모터 ID를 찾을 수 없습니다: " + std::to_string(motor_id));
}

float MotorController::getPosition(uint8_t motor_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = latest_feedback_.find(motor_id); it != latest_feedback_.end()) {
        return it->second.position;
    }
    throw std::runtime_error("모터 ID를 찾을 수 없습니다: " + std::to_string(motor_id));
}

float MotorController::setVelocity(uint8_t motor_id, float velocity) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = target_params_.find(motor_id); it != target_params_.end()) {
        it->second.velocity = velocity;
        return velocity;
    }
    throw std::runtime_error("모터 ID를 찾을 수 없습니다: " + std::to_string(motor_id));
}

float MotorController::getVelocity(uint8_t motor_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = latest_feedback_.find(motor_id); it != latest_feedback_.end()) {
        return it->second.velocity;
    }
    throw std::runtime_error("모터 ID를 찾을 수 없습니다: " + std::to_string(motor_id));
}

float MotorController::setKp(uint8_t motor_id, float kp) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = target_params_.find(motor_id); it != target_params_.end()) {
        it->second.kp = std::max(0.0f, kp);
        return it->second.kp;
    }
    throw std::runtime_error("모터 ID를 찾을 수 없습니다: " + std::to_string(motor_id));
}

float MotorController::getKp(uint8_t motor_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = target_params_.find(motor_id); it != target_params_.end()) {
        return it->second.kp;
    }
    throw std::runtime_error("모터 ID를 찾을 수 없습니다: " + std::to_string(motor_id));
}

float MotorController::setKd(uint8_t motor_id, float kd) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = target_params_.find(motor_id); it != target_params_.end()) {
        it->second.kd = std::max(0.0f, kd);
        return it->second.kd;
    }
    throw std::runtime_error("모터 ID를 찾을 수 없습니다: " + std::to_string(motor_id));
}

float MotorController::getKd(uint8_t motor_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = target_params_.find(motor_id); it != target_params_.end()) {
        return it->second.kd;
    }
    throw std::runtime_error("모터 ID를 찾을 수 없습니다: " + std::to_string(motor_id));
}

float MotorController::setTorque(uint8_t motor_id, float torque) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = target_params_.find(motor_id); it != target_params_.end()) {
        it->second.torque = torque;
        return torque;
    }
    throw std::runtime_error("모터 ID를 찾을 수 없습니다: " + std::to_string(motor_id));
}

float MotorController::getTorque(uint8_t motor_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = latest_feedback_.find(motor_id); it != latest_feedback_.end()) {
        return it->second.torque;
    }
    throw std::runtime_error("모터 ID를 찾을 수 없습니다: " + std::to_string(motor_id));
}

// 상태 관리 메서드 구현
std::map<uint8_t, MotorFeedback> MotorController::getLatestFeedback() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_feedback_;
}

void MotorController::togglePause() {
    paused_ = !paused_;
}

void MotorController::stop() {
    running_ = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

bool MotorController::isRunning() const {
    return running_;
}

void MotorController::reset() {
    restart_ = true;
}

// 설정 메서드 구현
void MotorController::setMaxUpdateRate(float rate) {
    max_update_rate_ = rate;
}

float MotorController::getActualUpdateRate() const {
    return actual_update_rate_;
}

bool MotorController::toggleSerial() {
    serial_ = !serial_;
    return serial_;
}

bool MotorController::getSerial() const {
    return serial_;
}

// 모터 제어 메서드 구현
void MotorController::addMotorToZero(uint8_t motor_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    setTorque(motor_id, 0.0f);
    setPosition(motor_id, 0.0f);
    setVelocity(motor_id, 0.0f);
    motors_to_zero_.push_back(motor_id);
}

void MotorController::setParams(uint8_t motor_id, const MotorControlParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = target_params_.find(motor_id); it != target_params_.end()) {
        it->second = params;
    } else {
        throw std::runtime_error("모터 ID를 찾을 수 없습니다: " + std::to_string(motor_id));
    }
}

// 명령 카운터 관련 메서드 구현
uint64_t MotorController::getTotalCommands() const {
    return total_commands_;
}

uint64_t MotorController::getFailedCommands(uint8_t motor_id) const {
    if (auto it = failed_commands_.find(motor_id); it != failed_commands_.end()) {
        return it->second;
    }
    throw std::runtime_error("모터 ID를 찾을 수 없습니다: " + std::to_string(motor_id));
}

void MotorController::resetCommandCounters() {
    total_commands_ = 0;
    for (auto& [_, count] : failed_commands_) {
        count = 0;
    }
}

// CAN 통신 관련 메서드 구현
std::map<uint8_t, MotorFeedback> MotorController::sendMotorControls(const std::map<uint8_t, MotorControlParams>& params, bool serial) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 파라미터가 비어있으면 target_params_ 사용
    const auto& control_params = params.empty() ? target_params_ : params;
    
    if (control_params.empty()) {
        return {};
    }

    try {
        std::vector<CanPack> packs;
        for (const auto& [id, params] : control_params) {
            packs.push_back(packMotorParams(id, params));
        }
        
        txPacks(packs, serial);
        
        // 피드백 수신
        std::map<uint8_t, MotorFeedback> feedbacks;
        auto received_packs = rxUnpack(control_params.size(), serial);
        
        for (const auto& pack : received_packs) {
            feedbacks[pack.ex_id.id] = unpackFeedback(pack);
        }
        
        total_commands_++;
        return feedbacks;
    } catch (const std::exception& e) {
        for (const auto& [id, _] : control_params) {
            failed_commands_[id]++;
        }
        throw;
    }
}

void MotorController::sendResets() {
    std::vector<CanPack> packs;
    for (const auto& [id, _] : motor_mapping_) {
        CanPack pack;
        pack.ex_id = {id, static_cast<uint16_t>(CAN_ID_DEBUG_UI), CanComMode::MotorReset, 0};
        pack.len = 8;
        pack.data = std::vector<uint8_t>(8, 0);
        packs.push_back(pack);
    }
    txPacks(packs, false);
}

void MotorController::sendStarts() {
    std::vector<CanPack> packs;
    for (const auto& [id, _] : motor_mapping_) {
        CanPack pack;
        pack.ex_id = {id, static_cast<uint16_t>(CAN_ID_DEBUG_UI), CanComMode::MotorIn, 0};
        pack.len = 8;
        pack.data = std::vector<uint8_t>(8, 0);
        packs.push_back(pack);
    }
    txPacks(packs, false);
}

void MotorController::sendSetZeros(const std::vector<uint8_t>& motor_ids) {
    std::vector<CanPack> packs;
    for (uint8_t id : motor_ids) {
        CanPack pack;
        pack.ex_id = {id, static_cast<uint16_t>(CAN_ID_DEBUG_UI), CanComMode::MotorZero, 0};
        pack.len = 8;
        pack.data = std::vector<uint8_t>(8, 0);
        pack.data[0] = 1;
        packs.push_back(pack);
    }
    txPacks(packs, false);
}

// CAN 통신 헬퍼 메서드 구현
CanPack MotorController::packMotorParams(uint8_t id, const MotorControlParams& params) {
    if (auto it = motor_configs_.find(id); it != motor_configs_.end()) {
        const auto& config = it->second;
        
        CanPack pack;
        pack.ex_id = {id, 0, CanComMode::MotorCtrl, 0};
        pack.len = 8;
        pack.data = std::vector<uint8_t>(8, 0);
        
        // 파라미터 변환 및 패킹
        uint16_t pos_int = static_cast<uint16_t>((params.position - config.p_min) * 65535.0f / (config.p_max - config.p_min));
        uint16_t vel_int = static_cast<uint16_t>((params.velocity - config.v_min) * 65535.0f / (config.v_max - config.v_min));
        uint16_t kp_int = static_cast<uint16_t>((params.kp - config.kp_min) * 65535.0f / (config.kp_max - config.kp_min));
        uint16_t kd_int = static_cast<uint16_t>((params.kd - config.kd_min) * 65535.0f / (config.kd_max - config.kd_min));
        uint16_t torque_int = static_cast<uint16_t>((params.torque - config.t_min) * 65535.0f / (config.t_max - config.t_min));
        
        pack.ex_id.data = torque_int;
        pack.data[0] = (pos_int >> 8) & 0xFF;
        pack.data[1] = pos_int & 0xFF;
        pack.data[2] = (vel_int >> 8) & 0xFF;
        pack.data[3] = vel_int & 0xFF;
        pack.data[4] = (kp_int >> 8) & 0xFF;
        pack.data[5] = kp_int & 0xFF;
        pack.data[6] = (kd_int >> 8) & 0xFF;
        pack.data[7] = kd_int & 0xFF;
        
        return pack;
    }
    throw std::runtime_error("모터 ID를 찾을 수 없습니다: " + std::to_string(id));
}

MotorFeedback MotorController::unpackFeedback(const CanPack& pack) {
    if (auto it = motor_configs_.find(pack.ex_id.id); it != motor_configs_.end()) {
        const auto& config = it->second;
        
        uint16_t pos_int = (pack.data[0] << 8) | pack.data[1];
        uint16_t vel_int = (pack.data[2] << 8) | pack.data[3];
        uint16_t torque_int = (pack.data[4] << 8) | pack.data[5];
        
        float position = config.p_min + (pos_int * (config.p_max - config.p_min) / 65535.0f);
        float velocity = config.v_min + (vel_int * (config.v_max - config.v_min) / 65535.0f);
        float torque = config.t_min + (torque_int * (config.t_max - config.t_min) / 65535.0f);
        
        return MotorFeedback{
            pack.ex_id.id,
            position,
            velocity,
            torque,
            static_cast<RunMode>((pack.ex_id.data >> 14) & 0x03),
            static_cast<uint16_t>((pack.ex_id.data >> 8) & 0x3F)
        };
    }
    throw std::runtime_error("모터 ID를 찾을 수 없습니다: " + std::to_string(pack.ex_id.id));
}

void MotorController::txPacks(const std::vector<CanPack>& serialData, bool verbose) {
    if (serialData.empty()) {
        return;
    }
    std::vector<uint8_t> result;
    
    if (serialData.size() >= 6) {
        // 첫 4바이트 처리
        result.push_back(serialData[0]);
        result.push_back(serialData[1]);
        result.push_back(serialData[2]);
        result.push_back(serialData[3]);
        
        // 5번째 바이트 처리
        result.push_back(serialData[4]);
        
        // 나머지 바이트 처리
        for (size_t i = 5; i < serialData.size(); ++i) {
            result.push_back(serialData[i]);
        }
    }
    
    if (verbose) {
        std::cout << "TX: ";
        for (uint8_t b : result) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ";
        }
        std::cout << std::dec << std::endl;
    }
    
    try {
        port_->write(result.data(), result.size());
        // port_->flush();
    } catch (const std::exception& e) {
        throw std::runtime_error("CAN 메시지 전송 실패: " + std::string(e.what()));
    }
}

std::vector<CanPack> MotorController::rxUnpack(size_t len, bool verbose) {
    std::vector<CanPack> packs;
    size_t max_attempts = len * 2;
    size_t attempt = 0;
    
    while (packs.size() < len && attempt < max_attempts) {
        attempt++;
        
        // AT 헤더 읽기
        std::vector<uint8_t> header(2);
        if (port_->read(header.data(), 2) != 2 || header[0] != 'A' || header[1] != 'T') {
            continue;
        }
        
        // ex_id 읽기
        std::vector<uint8_t> ex_id_bytes(4);
        if (port_->read(ex_id_bytes.data(), 4) != 4) {
            continue;
        }
        
        // 길이 읽기
        uint8_t len_byte;
        if (port_->read(&len_byte, 1) != 1) {
            continue;
        }
        
        // 데이터 읽기
        std::vector<uint8_t> data(len_byte);
        if (port_->read(data.data(), len_byte) != len_byte) {
            continue;
        }
        
        // 종료 문자 읽기
        std::vector<uint8_t> ending(2);
        if (port_->read(ending.data(), 2) != 2 || ending[0] != '\r' || ending[1] != '\n') {
            continue;
        }
        
        // 패킷 생성
        CanPack pack;
        std::array<uint8_t, 4> ex_id_array;
        std::copy(ex_id_bytes.begin(), ex_id_bytes.begin() + 4, ex_id_array.begin());
        pack.ex_id = unpackExId(ex_id_array);
        pack.len = len_byte;
        pack.data = data;
        packs.push_back(pack);
        
        if (verbose) {
            // 디버그 출력
            std::cout << "RX: ";
            for (uint8_t b : header) std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ";
            for (uint8_t b : ex_id_bytes) std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ";
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(len_byte) << " ";
            for (uint8_t b : data) std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ";
            for (uint8_t b : ending) std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ";
            std::cout << std::dec << std::endl;
        }
    }
    
    return packs;
}

void MotorController::setMotorMode(uint8_t motor_id, RunMode mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    setMotorParameter(motor_id, 0X7005, static_cast<float>(mode), 'j');
}

void MotorController::setMotorParameter(uint8_t motor_id, uint16_t index, float value, char value_mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    CanPack pack;
    // pack.ex_id = COMMUNICATION_TYPE_SET_SINGLE_PARAMETER << 24 | master_can_id_ << 8 | motor_id;
    pack.ex_id = {motor_id, 0, CanComMode::ParaWrite, 0};
    pack.len = 8;
    pack.data = std::vector<uint8_t>(pack.len, 0);
    pack.data[0] = index & 0xFF;
    pack.data[1] = (index >> 8) & 0xFF;
    pack.data[2] = 0x00;
    pack.data[3] = 0x00;

    if (value_mode == 'p') {
        memcpy(&pack.data[4], &value, 4);
    } else if (value_mode == 'j') {
        pack.data[4] = static_cast<uint8_t>(value);
        pack.data[5] = 0x00;
        pack.data[6] = 0x00;
        pack.data[7] = 0x00;
    }

    txPacks({pack}, false);
}

void MotorController::getMotorParameter(uint8_t motor_id, uint16_t index) {
    std::lock_guard<std::mutex> lock(mutex_);
    CanPack pack;
    //    pack.id = COMMUNICATION_TYPE_GET_SINGLE_PARAMETER << 24 | master_can_id_ << 8 | motor_id;
    pack.ex_id = {motor_id, 0, CanComMode::ParaRead, 0};
    pack.len = 8;
    pack.data = std::vector<uint8_t>(pack.len, 0);
    pack.data[0] = index & 0xFF;
    pack.data[1] = (index >> 8) & 0xFF;
    txPacks({pack}, false);
}

void MotorController::setCanId(uint8_t motor_id, uint8_t new_can_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    disableMotor(motor_id, false);
    CanPack pack;
    //    pack.id = COMMUNICATION_TYPE_CAN_ID << 24 | new_can_id << 16 | master_can_id_ << 8 | motor_id;
    pack.ex_id = {motor_id, static_cast<uint16_t>(new_can_id), CanComMode::MotorId, 0};
    pack.len = 8;
    pack.data = std::vector<uint8_t>(8, 0);
    pack.data[0] = new_can_id & 0xFF;
    pack.data[1] = (new_can_id >> 8) & 0xFF;
    txPacks({pack}, false);
}

void MotorController::setZeroPosition(uint8_t motor_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    disableMotor(motor_id, false);
    CanPack pack;
    //    pack.id = COMMUNICATION_TYPE_SET_POS_ZERO << 24 | master_can_id_ << 8 | motor_id;
    pack.ex_id = {motor_id, 0, CanComMode::MotorZero, 0};
    pack.len = 8;
    pack.data = std::vector<uint8_t>(8, 0);
    pack.data[0] = 1;
    txPacks({pack}, false);
    enableMotor(motor_id);
}

void MotorController::enableMotor(uint8_t motor_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    CanPack pack;
    //    pack.id = COMMUNICATION_TYPE_MOTOR_ENABLE << 24 | master_can_id_ << 8 | motor_id;
    pack.ex_id = {motor_id, 0, CanComMode::MotorIn, 0};
    pack.len = 8;
    pack.data = std::vector<uint8_t>(8, 0);
    txPacks({pack}, false);
}

void MotorController::disableMotor(uint8_t motor_id, bool clear_error) {
    std::lock_guard<std::mutex> lock(mutex_);
    CanPack pack;
    //    pack.id = COMMUNICATION_TYPE_MOTOR_STOP << 24 | master_can_id_ << 8 | motor_id;
    pack.ex_id = {motor_id, 0, CanComMode::MotorReset, 0};
    pack.len = 8;
    pack.data = std::vector<uint8_t>(8, 0);
    pack.data[0] = clear_error ? 1 : 0;
    txPacks({pack}, false);
}

void MotorController::controlMotor(uint8_t motor_id, RunMode mode, const std::function<void(MotorSet&)>& update_params) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!validateMotorId(motor_id)) {
        throw std::runtime_error("모터 ID를 찾을 수 없습니다: " + std::to_string(motor_id));
    }

    if (latest_feedback_[motor_id].mode != mode) {
        setMotorMode(motor_id, mode);
    }

    auto& motor_set = motor_sets_[motor_id];
    update_params(motor_set);

    auto motor_type = motor_mapping_.at(motor_id);
    const auto& config = ROBSTRIDE_CONFIGS.at(motor_type);

    // 제어 파라미터 준비
    std::vector<uint8_t> data(8, 0);
    switch (mode) {
        case RunMode::MitMode: {
            uint16_t torque_int = floatToUint(clampValue(motor_set.set_torque, config.t_min, config.t_max), config.t_min, config.t_max, 16);
            uint16_t angle_int = floatToUint(clampValue(motor_set.set_angle, config.p_min, config.p_max), config.p_min, config.p_max, 16);
            uint16_t speed_int = floatToUint(clampValue(motor_set.set_speed, config.v_min, config.v_max), config.v_min, config.v_max, 16);
            uint16_t kp_int = floatToUint(clampValue(motor_set.set_kp, config.kp_min, config.kp_max), config.kp_min, config.kp_max, 16);
            uint16_t kd_int = floatToUint(clampValue(motor_set.set_kd, config.kd_min, config.kd_max), config.kd_min, config.kd_max, 16);
            
            data[0] = angle_int >> 8;
            data[1] = angle_int & 0xFF;
            data[2] = speed_int >> 8;
            data[3] = speed_int & 0xFF;
            data[4] = kp_int >> 8;
            data[5] = kp_int & 0xFF;
            data[6] = kd_int >> 8;
            data[7] = kd_int & 0xFF;
            break;
        }
        case RunMode::PositionMode: {
            float speed = clampValue(motor_set.set_speed, config.v_min, config.v_max);
            float angle = clampValue(motor_set.set_angle, config.p_min, config.p_max);
            setMotorParameter(motor_id, 0X7017, speed, 'p');
            setMotorParameter(motor_id, 0X7025, motor_set.set_acceleration, 'p');
            setMotorParameter(motor_id, 0X7016, angle, 'p');
            return;
        }
        case RunMode::SpeedMode: {
            float speed = clampValue(motor_set.set_speed, config.v_min, config.v_max);
            setMotorParameter(motor_id, 0X7018, motor_set.set_limit_cur, 'p');
            setMotorParameter(motor_id, 0X7022, motor_set.set_acceleration, 'p');
            setMotorParameter(motor_id, 0X700A, speed, 'p');
            return;
        }
        case RunMode::CurrentMode: {
            float current = clampValue(motor_set.set_current, config.t_min, config.t_max);
            setMotorParameter(motor_id, 0X7006, current, 'p');
            return;
        }
        default:
            throw std::runtime_error("지원하지 않는 모터 모드입니다");
    }

    sendCanCommand(motor_id, CanComMode::MotorCtrl, data);
}

void MotorController::moveControl(uint8_t motor_id, float torque, float angle, float speed, float kp, float kd) {
    controlMotor(motor_id, RunMode::MitMode, [&](MotorSet& motor_set) {
        motor_set.set_torque = torque;
        motor_set.set_angle = angle;
        motor_set.set_speed = speed;
        motor_set.set_kp = kp;
        motor_set.set_kd = kd;
    });
}

void MotorController::positionControl(uint8_t motor_id, float speed, float acceleration, float angle) {
    controlMotor(motor_id, RunMode::PositionMode, [&](MotorSet& motor_set) {
        motor_set.set_speed = speed;
        motor_set.set_angle = angle;
        motor_set.set_acceleration = acceleration;
    });
}

void MotorController::speedControl(uint8_t motor_id, float speed, float acceleration, float limit_current) {
    controlMotor(motor_id, RunMode::SpeedMode, [&](MotorSet& motor_set) {
        motor_set.set_speed = speed;
        motor_set.set_limit_cur = limit_current;
        motor_set.set_acceleration = acceleration;
    });
}

void MotorController::currentControl(uint8_t motor_id, float current) {
    controlMotor(motor_id, RunMode::CurrentMode, [&](MotorSet& motor_set) {
        motor_set.set_current = current;
    });
}

// 헬퍼 함수 구현
float MotorController::clampValue(float value, float min_val, float max_val) const {
    return std::max(min_val, std::min(max_val, value));
}

void MotorController::sendCanCommand(uint8_t motor_id, CanComMode mode, const std::vector<uint8_t>& data) {
    CanPack pack;
    pack.ex_id = {motor_id, 0, mode, 0};
    pack.len = data.size();
    pack.data = data;
    txPacks({pack}, false);
}

void MotorController::updateMotorState(uint8_t motor_id, const MotorControlParams& params) {
    auto& motor_set = motor_sets_[motor_id];
    motor_set.set_torque = params.torque;
    motor_set.set_angle = params.position;
    motor_set.set_speed = params.velocity;
    motor_set.set_kp = params.kp;
    motor_set.set_kd = params.kd;
}

bool MotorController::validateMotorId(uint8_t motor_id) const {
    return motor_mapping_.find(motor_id) != motor_mapping_.end();
} 

std::array<uint8_t, 4> MotorController::packExId(const ExId& ex_id) {
    // Python의 pack_bits 함수와 동일한 로직 구현
    uint32_t addr = 0;
    
    // id (8비트)
    addr |= (ex_id.id & 0xFF);
    
    // data (16비트)
    addr |= (ex_id.data & 0xFFFF) << 8;
    
    // mode (5비트)
    addr |= (static_cast<uint32_t>(ex_id.mode) & 0x1F) << 24;
    
    // res (3비트)
    addr |= (ex_id.res & 0x07) << 29;
    
    // 최종 주소 계산 (Python 코드의 << 3 | 0x00000004와 동일)
    addr = (addr << 3) | 0x00000004;
    
    // big-endian으로 변환
    std::array<uint8_t, 4> result;
    result[0] = (addr >> 24) & 0xFF;
    result[1] = (addr >> 16) & 0xFF;
    result[2] = (addr >> 8) & 0xFF;
    result[3] = addr & 0xFF;
    
    return result;
}

ExId MotorController::unpackExId(const std::array<uint8_t, 4>& bytes) {
    // big-endian에서 uint32_t로 변환
    uint32_t addr = (static_cast<uint32_t>(bytes[0]) << 24) |
                   (static_cast<uint32_t>(bytes[1]) << 16) |
                   (static_cast<uint32_t>(bytes[2]) << 8) |
                   static_cast<uint32_t>(bytes[3]);
    
    // Python의 unpack_bits 함수와 동일한 로직 구현
    addr >>= 3;  // Python 코드의 >> 3과 동일
    
    ExId result;
    result.id = addr & 0xFF;                    // 8비트
    result.data = (addr >> 8) & 0xFFFF;         // 16비트
    result.mode = static_cast<CanComMode>((addr >> 24) & 0x1F);  // 5비트
    result.res = (addr >> 29) & 0x07;           // 3비트
    
    return result;
}
