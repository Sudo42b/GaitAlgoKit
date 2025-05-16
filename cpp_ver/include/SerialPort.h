#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <string>
#include <stdexcept>
#ifdef _WIN32
    #include <windows.h>
#else
    #include <fcntl.h>
    #include <termios.h>
    #include <unistd.h>
    #include <sys/ioctl.h>
    typedef unsigned int DWORD;
#endif

class SerialPort {
public:
    SerialPort(const std::string& portName, 
                DWORD baudRate = 921600);
    ~SerialPort();

    // 시리얼 포트 열기
    void open();
    // 시리얼 포트 닫기
    void close();
    // 데이터 전송
    size_t write(const uint8_t* data, size_t length);
    // 데이터 수신
    size_t read(uint8_t* buffer, size_t length);
    // 포트가 열려있는지 확인
    bool isOpen() const;

private:
    std::string portName;
    DWORD baudRate;
    #ifdef _WIN32
        HANDLE hSerial;
    #else
        int hSerial;
    #endif
    bool opened;

    // 시리얼 포트 설정
    void configurePort();
};

#endif // SERIAL_PORT_H 