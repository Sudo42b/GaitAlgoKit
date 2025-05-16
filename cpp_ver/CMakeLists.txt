cmake_minimum_required(VERSION 3.10)
project(GaitAlgoKit)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 헤더 파일 경로 추가
include_directories(include)

# 소스 파일 목록 설정
set(SOURCE_FILES
    src/supervisor.cpp
    src/MotorController.cpp  # 이 파일이 추가되어 있는지 확인
    src/SerialPort.cpp       # 이 파일이 추가되어 있는지 확인
    # 필요한 추가 소스 파일
)

# 실행 파일 생성
add_executable(supervisor ${SOURCE_FILES})

# 필요한 경우 스레드 라이브러리 링크
find_package(Threads REQUIRED)
target_link_libraries(supervisor Threads::Threads)

# 기타 필요한 라이브러리 링크