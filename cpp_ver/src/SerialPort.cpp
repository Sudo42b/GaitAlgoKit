#include "SerialPort.h"
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
typedef int HANDLE;
#define INVALID_HANDLE_VALUE -1
#endif


SerialPort::SerialPort(const std::string& portName, DWORD baudRate)
    : portName(portName), baudRate(baudRate), hSerial(INVALID_HANDLE_VALUE), opened(false) {
}

SerialPort::~SerialPort() {
    if (opened) {
        close();
    }
}

void SerialPort::open() {
    #ifdef _WIN32
        std::string full_port_name = "\\\\.\\" + portName;
        hSerial = CreateFileA(
            full_port_name.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            nullptr,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            nullptr
        );

        if (hSerial == INVALID_HANDLE_VALUE) {
            throw std::runtime_error("시리얼 포트 열기 실패: " + portName);
        }

            DCB dcb = {0};
            dcb.DCBlength = sizeof(DCB);
            if (!GetCommState(hSerial, &dcb)) {
                CloseHandle(hSerial);
                throw std::runtime_error("시리얼 포트 상태 읽기 실패");
            }

            dcb.BaudRate = baudRate;
            dcb.ByteSize = 8;
            dcb.Parity = NOPARITY;
            dcb.StopBits = ONESTOPBIT;

            if (!SetCommState(hSerial, &dcb)) {
                CloseHandle(hSerial);
                throw std::runtime_error("시리얼 포트 설정 실패");
            }

            COMMTIMEOUTS timeouts = {0};
            timeouts.ReadIntervalTimeout = 50;
            timeouts.ReadTotalTimeoutConstant = 50;
            timeouts.ReadTotalTimeoutMultiplier = 10;
            timeouts.WriteTotalTimeoutConstant = 50;
            timeouts.WriteTotalTimeoutMultiplier = 10;

            if (!SetCommTimeouts(hSerial, &timeouts)) {
                CloseHandle(hSerial);
                throw std::runtime_error("시리얼 포트 타임아웃 설정 실패");
            }
        #else
            hSerial = ::open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
            if (hSerial < 0) {
                throw std::runtime_error("시리얼 포트 열기 실패: " + portName);
            }

            struct termios tty;
            if (tcgetattr(hSerial, &tty) != 0) {
                ::close(hSerial);
                throw std::runtime_error("시리얼 포트 설정 읽기 실패");
            }

            // 입력 모드 설정
            tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
            tty.c_oflag &= ~OPOST;
            tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
            tty.c_cflag &= ~(CSIZE | PARENB);
            tty.c_cflag |= CS8;

            // 속도 설정
            speed_t speed;
            switch (baudRate) {
                case 9600: speed = B9600; break;
                case 19200: speed = B19200; break;
                case 38400: speed = B38400; break;
                case 57600: speed = B57600; break;
                case 115200: speed = B115200; break;
                default: speed = B115200;
            }
            cfsetispeed(&tty, speed);
            cfsetospeed(&tty, speed);

            // 타임아웃 설정
            tty.c_cc[VMIN] = 0;
            tty.c_cc[VTIME] = 5;

            if (tcsetattr(hSerial, TCSANOW, &tty) != 0) {
                ::close(hSerial);
                throw std::runtime_error("시리얼 포트 설정 실패");
            }
        #endif

    opened = true;
}

void SerialPort::close() {
    if (!opened) {
        return;
    }
    #ifdef _WIN32
        if (hSerial != INVALID_HANDLE_VALUE) {
            CloseHandle(hSerial);
        }
    #else
        if (hSerial >= 0) {
            ::close(hSerial);
        }
    #endif
    // 포트 핸들 초기화
    hSerial = INVALID_HANDLE_VALUE;
    opened = false;
}

void SerialPort::configurePort() {
    #ifdef _WIN32
        DCB dcbSerialParams = {0};
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

        // 현재 설정 가져오기
        if (!GetCommState(hSerial, &dcbSerialParams)) {
            CloseHandle(hSerial);
            throw std::runtime_error("포트 설정을 가져올 수 없습니다.");
        }

        // 통신 설정
        dcbSerialParams.BaudRate = baudRate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;

        // 설정 적용
        if (!SetCommState(hSerial, &dcbSerialParams)) {
            CloseHandle(hSerial);
            throw std::runtime_error("포트 설정을 적용할 수 없습니다.");
        }

        // 타임아웃 설정
        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout = 10;
        timeouts.ReadTotalTimeoutConstant = 10;
        timeouts.ReadTotalTimeoutMultiplier = 10;
        timeouts.WriteTotalTimeoutConstant = 10;
        timeouts.WriteTotalTimeoutMultiplier = 10;

        if (!SetCommTimeouts(hSerial, &timeouts)) {
            CloseHandle(hSerial);
            throw std::runtime_error("타임아웃 설정을 적용할 수 없습니다.");
        }
    #else
        struct termios tty;
        if (tcgetattr(hSerial, &tty) != 0) {
            ::close(hSerial);
            throw std::runtime_error("시리얼 포트 설정 읽기 실패");
        }

        // 입력 모드 설정
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_oflag &= ~OPOST;
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_cflag &= ~(CSIZE | PARENB);
        tty.c_cflag |= CS8;

        // 속도 설정
        speed_t speed;
        switch (baudRate) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            default: speed = B115200;
        }
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        // 타임아웃 설정
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5;

        if (tcsetattr(hSerial, TCSANOW, &tty) != 0) {
            ::close(hSerial);
            throw std::runtime_error("시리얼 포트 설정 실패");
        }
    #endif
}

size_t SerialPort::write(const uint8_t* data, size_t length) {
    if (!opened) {
        throw std::runtime_error("포트가 열려있지 않습니다.");
    }

    DWORD bytesWritten;
    #ifdef _WIN32
        if (!WriteFile(hSerial, data, length, &bytesWritten, NULL)) {
            throw std::runtime_error("데이터를 전송할 수 없습니다.");
        }
    #else
        ssize_t result = ::write(hSerial, data, length);
        if (result < 0) {
            throw std::runtime_error("데이터를 전송할 수 없습니다.");
        }
        bytesWritten = static_cast<DWORD>(result);
    #endif

    return bytesWritten;
}

size_t SerialPort::read(uint8_t* buffer, size_t length) {
    if (!opened) {
        throw std::runtime_error("포트가 열려있지 않습니다.");
    }

    DWORD bytesRead;
    #ifdef _WIN32
        if (!ReadFile(hSerial, buffer, length, &bytesRead, NULL)) {
            throw std::runtime_error("데이터를 수신할 수 없습니다.");
        }
    #else
        ssize_t result = ::read(hSerial, buffer, length);
        if (result < 0) {
            throw std::runtime_error("데이터를 수신할 수 없습니다.");
        }
        bytesRead = static_cast<DWORD>(result);
    #endif
    if (bytesRead == 0) {
        return 0; // No data read
    }

    return bytesRead;
}

bool SerialPort::isOpen() const {
    return opened;
} 