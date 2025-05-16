#include "../include/Robstrite.h"
#include "SerialPort.h"
#include <iostream>
#include <string>

// 시리얼 포트 객체
SerialPort* serialPort = nullptr;
RobStrite_Motor motor(0x01);  // CAN ID 0x01로 모터 객체 생성

// CAN 메시지 전송 함수
bool SendCANMessage(uint8_t* data, uint8_t length) {
    if (!serialPort || !serialPort->isOpen()) {
        std::cout << "시리얼 포트가 연결되지 않았습니다." << std::endl;
        return false;
    }

    try {
        serialPort->write(data, length);
        return true;
    } catch (const std::exception& e) {
        std::cout << "CAN 메시지 전송 실패: " << e.what() << std::endl;
        return false;
    }
}

// CAN 메시지 수신 함수
bool ReceiveCANMessage(uint8_t* data, uint8_t* length) {
    if (!serialPort || !serialPort->isOpen()) {
        return false;
    }

    try {
        *length = serialPort->read(data, 8);
        return true;
    } catch (const std::exception& e) {
        std::cout << "CAN 메시지 수신 실패: " << e.what() << std::endl;
        return false;
    }
}

int main() {
    try {
        // 시리얼 포트 열기
        serialPort = new SerialPort("COM3");  // 포트 이름은 실제 사용하는 포트로 변경
        serialPort->open();
        std::cout << "시리얼 포트 연결 성공" << std::endl;

        // 모터 초기화
        motor.RobStrite_Get_CAN_ID();    // 장치 ID 가져오기
        motor.Set_CAN_ID(0x01);          // CAN ID 설정
        motor.Set_ZeroPos();             // 현재 위치를 영점으로 설정
        motor.Enable_Motor();            // 모터 활성화

        // 메인 루프
        while (true) {
            std::cout << "\n모터 제어 모드 선택:" << std::endl;
            std::cout << "1. 토크 모드" << std::endl;
            std::cout << "2. 위치 제어 모드" << std::endl;
            std::cout << "3. 속도 제어 모드" << std::endl;
            std::cout << "4. 전류 제어 모드" << std::endl;
            std::cout << "5. 종료" << std::endl;
            std::cout << "선택: ";

            int choice;
            std::cin >> choice;

            switch (choice) {
                case 1: {  // 토크 모드
                    float torque, angle, speed, kp, kd;
                    std::cout << "토크 (Nm): ";
                    std::cin >> torque;
                    std::cout << "각도 (rad): ";
                    std::cin >> angle;
                    std::cout << "속도 (rad/s): ";
                    std::cin >> speed;
                    std::cout << "Kp: ";
                    std::cin >> kp;
                    std::cout << "Kd: ";
                    std::cin >> kd;
                    motor.RobStrite_Motor_move_control(torque, angle, speed, kp, kd);
                    break;
                }
                case 2: {  // 위치 제어 모드
                    float speed, accel, angle;
                    std::cout << "속도 (rad/s): ";
                    std::cin >> speed;
                    std::cout << "가속도 (rad/s^2): ";
                    std::cin >> accel;
                    std::cout << "목표 각도 (rad): ";
                    std::cin >> angle;
                    motor.RobStrite_Motor_Pos_control(speed, accel, angle);
                    break;
                }
                case 3: {  // 속도 제어 모드
                    float speed, accel, current;
                    std::cout << "목표 속도 (rad/s): ";
                    std::cin >> speed;
                    std::cout << "가속도 (rad/s^2): ";
                    std::cin >> accel;
                    std::cout << "전류 제한 (A): ";
                    std::cin >> current;
                    motor.RobStrite_Motor_Speed_control(speed, accel, current);
                    break;
                }
                case 4: {  // 전류 제어 모드
                    float current;
                    std::cout << "목표 전류 (A): ";
                    std::cin >> current;
                    motor.RobStrite_Motor_current_control(current);
                    break;
                }
                case 5:  // 종료
                    motor.Disenable_Motor(0);
                    delete serialPort;
                    return 0;
                default:
                    std::cout << "잘못된 선택입니다." << std::endl;
            }

            // CAN 메시지 수신 처리
            uint8_t rxData[8];
            uint8_t rxLength;
            if (ReceiveCANMessage(rxData, &rxLength)) {
                motor.RobStrite_Motor_Analysis(rxData, 0);
            }
        }
    } catch (const std::exception& e) {
        std::cout << "오류 발생: " << e.what() << std::endl;
        if (serialPort) {
            delete serialPort;
        }
        return 1;
    }

    return 0;
}
