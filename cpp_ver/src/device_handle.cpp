#ifdef _WIN32
    #include <windows.h>
#else
    #include <unistd.h>
    #include <fcntl.h>
    #include <termios.h>
    #include <string.h>
    #include <errno.h>
    #include <iostream>
    #include <chrono>
    #include <thread>
    typedef int HANDLE;
#endif

// 전역 변수 선언
HANDLE hDevice = -1;

#ifdef _WIN32
    // Windows 초기화 코드는 그대로 유지
#else
bool initCANDevice(const char* port = "/dev/ttyUSB0") {
    std::cout << "시리얼 포트 열기 시도 중... (" << port << ")" << std::endl;
    
    // 시리얼 포트 열기 (non-blocking 모드로 먼저 열기)
    hDevice = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (hDevice < 0) {
        std::cerr << "시리얼 포트 열기 실패: " << strerror(errno) << std::endl;
        return false;
    }
    std::cout << "시리얼 포트 열기 성공!" << std::endl;

    // non-blocking 모드 해제
    int flags = fcntl(hDevice, F_GETFL, 0);
    flags &= ~O_NONBLOCK;
    if (fcntl(hDevice, F_SETFL, flags) < 0) {
        std::cerr << "non-blocking 모드 해제 실패: " << strerror(errno) << std::endl;
        close(hDevice);
        hDevice = -1;
        return false;
    }

    // 시리얼 포트 설정
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(hDevice, &tty) != 0) {
        std::cerr << "시리얼 포트 설정 읽기 실패: " << strerror(errno) << std::endl;
        close(hDevice);
        hDevice = -1;
        return false;
    }

    // 통신 속도 설정 (921600 baud)
    speed_t baud = B921600;
    if (cfsetispeed(&tty, baud) != 0 || cfsetospeed(&tty, baud) != 0) {
        std::cerr << "통신 속도 설정 실패: " << strerror(errno) << std::endl;
        close(hDevice);
        hDevice = -1;
        return false;
    }
    std::cout << "통신 속도 설정 성공 (921600 baud)" << std::endl;

    // 8N1 모드 설정 (8비트 데이터, 패리티 없음, 1 스톱 비트)
    tty.c_cflag &= ~PARENB;  // 패리티 비트 제거
    tty.c_cflag &= ~CSTOPB;  // 1 스톱 비트
    tty.c_cflag &= ~CSIZE;   // 문자 크기 마스크 제거
    tty.c_cflag |= CS8;      // 8비트 데이터 비트
    tty.c_cflag &= ~CRTSCTS; // 하드웨어 흐름 제어 비활성화
    tty.c_cflag |= CREAD | CLOCAL; // 수신 활성화, 모뎀 제어 비활성화

    // 로컬 모드 설정
    tty.c_lflag &= ~ICANON;  // 정규 입력 비활성화
    tty.c_lflag &= ~ECHO;    // 에코 비활성화
    tty.c_lflag &= ~ECHOE;   // 에러 에코 비활성화
    tty.c_lflag &= ~ECHONL;  // 새줄 에코 비활성화
    tty.c_lflag &= ~ISIG;    // 인터럽트 시그널 비활성화

    // 입력 모드 설정
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // 소프트웨어 흐름 제어 비활성화
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // 특수 문자 처리 비활성화

    // 출력 모드 설정
    tty.c_oflag &= ~OPOST;   // 출력 처리 비활성화

    // 타임아웃 설정 (10ms)
    tty.c_cc[VMIN] = 0;      // 최소 문자 수
    tty.c_cc[VTIME] = 1;     // 타임아웃 (0.1초)

    // 설정 적용
    if (tcsetattr(hDevice, TCSANOW, &tty) != 0) {
        std::cerr << "시리얼 포트 설정 적용 실패: " << strerror(errno) << std::endl;
        close(hDevice);
        hDevice = -1;
        return false;
    }

    // 버퍼 플러시
    tcflush(hDevice, TCIOFLUSH);
    std::cout << "시리얼 포트 설정 완료!" << std::endl;

    // 잠시 대기
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return true;
}
#endif 