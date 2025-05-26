#!/bin/bash

# 스크립트 실행 권한 확인
if [ ! -x "motor_senario.py" ]; then
    chmod +x motor_senario.py
fi

# 기본값 설정
RIGHT_ANGLE=90.0
LEFT_ANGLE=90.0
PERIOD=2.5
AMPLITUDE=1.0

# 파라미터 입력 받기
read -p "오른쪽 다리 각도를 입력하세요 (기본값: 90.0): " input_right
if [ ! -z "$input_right" ]; then
    RIGHT_ANGLE=$input_right
fi

read -p "왼쪽 다리 각도를 입력하세요 (기본값: 90.0): " input_left
if [ ! -z "$input_left" ]; then
    LEFT_ANGLE=$input_left
fi

read -p "주기를 입력하세요 (기본값: 2.0): " input_period
if [ ! -z "$input_period" ]; then
    PERIOD=$input_period
fi

# 시나리오 선택 메뉴
echo "모터 제어 시나리오를 선택하세요:"
echo "1. 빠르게 걷기"
echo "2. 느리게 걷기"
echo "3. 오른쪽 다리 부상"
echo "4. 왼쪽 다리 부상"

# 사용자 입력 받기
read -p "선택 (1-4): " choice

# 선택에 따른 Python 스크립트 실행
case $choice in
    1)
        echo "빠른 걷기 시나리오를 시작합니다..."
        python3 motor_senario.py --right-angle $RIGHT_ANGLE --left-angle $LEFT_ANGLE --period $PERIOD
        ;;
    2)
        echo "느린 걷기 시나리오를 시작합니다..."
        python3 motor_senario.py --right-angle $RIGHT_ANGLE --left-angle $LEFT_ANGLE --period $PERIOD
        ;;
    3)
        echo "오른쪽 다리 부상 시나리오를 시작합니다..."
        python3 motor_senario.py --right-angle $RIGHT_ANGLE --left-angle $LEFT_ANGLE --period $PERIOD
        ;;
    4)
        echo "왼쪽 다리 부상 시나리오를 시작합니다..."
        python3 motor_senario.py --right-angle $RIGHT_ANGLE --left-angle $LEFT_ANGLE --period $PERIOD
        ;;
    *)
        echo "잘못된 선택입니다. 1-4 사이의 숫자를 입력해주세요."
        exit 1
        ;;
esac

# 스크립트 종료
exit 0
