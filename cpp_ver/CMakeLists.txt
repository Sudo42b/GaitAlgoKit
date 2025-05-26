cmake_minimum_required(VERSION 3.10)
project(GaitAlgoKit)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 헤더 파일 경로 추가
include_directories(include)

# 소스 파일 목록 설정
set(SOURCE_FILES
    # src/main.cpp
    src/supervisor.cpp
    src/MotorController.cpp
    src/SerialPort.cpp
    src/Robstride.cpp
    src/device_handle.cpp
)

# 실행 파일 생성
add_executable(gait_algokit ${SOURCE_FILES})

# 필요한 경우 스레드 라이브러리 링크
find_package(Threads REQUIRED)
target_link_libraries(gait_algokit Threads::Threads)

# 기타 필요한 라이브러리 링크