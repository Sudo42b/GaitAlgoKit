#include "MotorController.h"
#include "device_handle.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <csignal>
#include <map>
#include <string>
#include <unistd.h>

// M_PI가 정의되어 있지 않은 경우를 위한 정의
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 전역 변수로 MotorController 포인터 선언 (시그널 핸들러에서 사용하기 위함)
std::unique_ptr<MotorController> supervisor;
uint8_t motor_id1, motor_id2;

// 시그널 핸들러
void signalHandler(int signum) {
    if (supervisor) {
        supervisor->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        supervisor->addMotorToZero(motor_id1);
        supervisor->addMotorToZero(motor_id2);
    }
    exit(signum);
}

int main(int argc, char* argv[]) {
    // 기본 인자값 설정
    std::string port_name = "/dev/ttyUSB0";  // Windows의 경우 COM 포트 사용
    
    // 시리얼 포트 권한 확인
    if (access(port_name.c_str(), R_OK | W_OK) != 0) {
        std::cerr << "시리얼 포트 권한 오류: " << port_name << std::endl;
        std::cerr << "다음 명령어를 실행해보세요: sudo chmod 666 " << port_name << std::endl;
        return 1;
    }
    
    // CAN 장치 초기화
    std::cout << "CAN 장치 초기화 시도 중... (" << port_name << ")" << std::endl;
    if (!initCANDevice(port_name.c_str())) {
        std::cerr << "CAN 장치 초기화 실패: " << port_name << std::endl;
        std::cerr << "다음 사항을 확인해주세요:" << std::endl;
        std::cerr << "1. 시리얼 포트가 올바르게 연결되어 있는지" << std::endl;
        std::cerr << "2. baud rate가 올바른지 (현재: 921600)" << std::endl;
        std::cerr << "3. 다른 USB 포트를 시도해보세요" << std::endl;
        return 1;
    }
    std::cout << "CAN 장치 초기화 성공!" << std::endl;
    
    uint8_t motor_id = 1;
    std::string motor_type = "01";
    uint8_t second_motor_id = 2;
    std::string second_motor_type = "01";
    float sleep_time = 0.0f;
    float period = 7.5f;
    float amplitude = 1.0f;
    float target_angle = 30.0f;
    bool verbose = false;

    // 명령행 인자 처리
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--port-name" && i + 1 < argc) port_name = argv[++i];
        else if (arg == "--motor-id" && i + 1 < argc) motor_id = std::stoi(argv[++i]);
        else if (arg == "--motor-type" && i + 1 < argc) motor_type = argv[++i];
        else if (arg == "--second-motor-id" && i + 1 < argc) second_motor_id = std::stoi(argv[++i]);
        else if (arg == "--second-motor-type" && i + 1 < argc) second_motor_type = argv[++i];
        else if (arg == "--sleep" && i + 1 < argc) sleep_time = std::stof(argv[++i]);
        else if (arg == "--period" && i + 1 < argc) period = std::stof(argv[++i]);
        else if (arg == "--amplitude" && i + 1 < argc) amplitude = std::stof(argv[++i]);
        else if (arg == "--target-angle" && i + 1 < argc) target_angle = std::stof(argv[++i]);
        else if (arg == "--verbose") verbose = true;
    }

    // 모터 매핑 설정
    std::map<uint8_t, MotorType> motor_mapping;
    motor_mapping[motor_id] = static_cast<MotorType>(std::stoi(motor_type));
    motor_mapping[second_motor_id] = static_cast<MotorType>(std::stoi(second_motor_type));

    // 전역 변수 설정
    motor_id1 = motor_id;
    motor_id2 = second_motor_id;

    // 시그널 핸들러 설정
    signal(SIGINT, signalHandler);

    try {
        // MotorController 초기화
        supervisor = std::make_unique<MotorController>(
            port_name,
            motor_mapping,
            10.0f,  // kp
            1.0f,   // kd
            1000.0f // target_update_rate
        );

        // 기본 제어 파라미터 설정
        supervisor->setKp(motor_id, 10.0f);
        supervisor->setKd(motor_id, 1.0f);
        supervisor->setKp(second_motor_id, 10.0f);
        supervisor->setKd(second_motor_id, 2.0f);

        // 모터 초기화 및 활성화
        std::cout << "모터 초기화 중..." << std::endl;
        supervisor->sendResets();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        supervisor->sendStarts();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 시작 시간 기록
        auto start_time = std::chrono::steady_clock::now();

        while (supervisor->isRunning()) {
            // 경과 시간 계산
            auto current_time = std::chrono::steady_clock::now();
            float elapsed_time = std::chrono::duration<float>(current_time - start_time).count();

            try {
                // 첫 번째 모터
                float angle1 = amplitude * std::sin(elapsed_time * 2 * M_PI / period);
                float velocity1 = amplitude * 2 * M_PI / period * std::cos(elapsed_time * 2 * M_PI / period);

                // 두 번째 모터 (반대 위상)
                float angle2 = amplitude * std::sin(elapsed_time * 2 * M_PI / period + M_PI);
                float velocity2 = amplitude * 2 * M_PI / period * std::cos(elapsed_time * 2 * M_PI / period + M_PI);

                // 위치와 속도 설정
                supervisor->setPosition(motor_id, angle1);
                supervisor->setVelocity(motor_id, velocity1);
                supervisor->setPosition(second_motor_id, angle2);
                supervisor->setVelocity(second_motor_id, velocity2);

                // 피드백 출력
                if (verbose) {
                    auto feedbacks = supervisor->getLatestFeedback();
                    for (const auto& [id, feedback] : feedbacks) {
                        std::cout << "모터 " << static_cast<int>(id) << ":" << std::endl;
                        std::cout << "  현재 각도: " << (feedback.position * 180 / M_PI) << "도" << std::endl;
                        std::cout << "  현재 속도: " << feedback.velocity << " rad/s" << std::endl;
                        std::cout << "  현재 토크: " << feedback.torque << std::endl;
                    }
                }

                // 대기
                if (sleep_time > 0) {
                    std::this_thread::sleep_for(std::chrono::duration<float>(sleep_time));
                }
            } catch (const std::exception& e) {
                std::cerr << "제어 루프 오류: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << "오류 발생: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}