#ifndef DEVICE_HANDLE_H
#define DEVICE_HANDLE_H

#ifdef _WIN32
    #include <windows.h>
#else
    #include <unistd.h>
    typedef int HANDLE;
#endif

// 전역 변수 선언
extern HANDLE hDevice;

// CAN 장치 초기화 함수 선언
bool initCANDevice(const char* port = "/dev/ttyUSB0");

#endif // DEVICE_HANDLE_H 