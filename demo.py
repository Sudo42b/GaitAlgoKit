#!/usr/bin/env python3
"""
config.ini 파일을 읽어서 step_senario_param_v2.py를 실행하는 데모 스크립트
"""
import configparser
import subprocess
import sys
import os

def read_config(config_file):
    """config.ini 파일 읽기"""
    if not os.path.exists(config_file):
        print(f"오류: 설정 파일({config_file})을 찾을 수 없습니다.")
        sys.exit(1)
    
    config = configparser.ConfigParser()
    config.read(config_file)
    return config

def build_command(config: configparser.ConfigParser):
    """Python 스크립트 실행 명령어 생성"""
    demo_path = os.path.join(os.getcwd(), "GaitAlgoKit/tests/robstride/step_senario_param_v2.py")
    cmd = ["python", demo_path]
    
    # 기본설정
    cmd.extend(["--period", str(config.getfloat("기본설정", "period", fallback=1.0))])
    cmd.extend(["--amplitude", str(config.getfloat("기본설정", "amplitude", fallback=1.0))])
    cmd.extend(["--duration", str(config.getfloat("기본설정", "duration", fallback=3.0))])
    cmd.extend(["--cycles", str(config.getint("기본설정", "cycles", fallback=1))])
    cmd.extend(["--scale", str(config.getfloat("기본설정", "scale", fallback=0.25))])
    
    # 다리설정
    cmd.extend(["--right-offset", str(config.getfloat("다리설정", "right_offset", fallback=0.0))])
    cmd.extend(["--left-offset", str(config.getfloat("다리설정", "left_offset", fallback=0.0))])
    
    # 출력설정
    if config.getboolean("출력설정", "verbose", fallback=False):
        cmd.append("--verbose")
    
    # 통신설정
    cmd.extend(["--port", config.get("통신설정", "port", fallback="/dev/ttyUSB0")])
    
    # 제어설정
    cmd.extend(["--kp", str(config.getfloat("제어설정", "kp", fallback=10.0))])
    cmd.extend(["--kd", str(config.getfloat("제어설정", "kd", fallback=2.0))])
    
    # 모터설정
    cmd.extend(["--motor-id", str(config.getint("모터설정", "motor_id", fallback=1))])
    cmd.extend(["--motor-type", config.get("모터설정", "motor_type", fallback="01").strip("'\"")])
    cmd.extend(["--second-motor-id", str(config.getint("모터설정", "second_motor_id", fallback=2))])
    cmd.extend(["--second-motor-type", config.get("모터설정", "second_motor_type", fallback="01").strip("'\"")])
    
    return cmd

def main():
    """메인 함수"""
    # 명령행 인자에서 설정 파일 경로 가져오기
    config_file = "./config.ini"
    if len(sys.argv) > 1:
        config_file = sys.argv[1]
    
    # 설정 파일 읽기
    config = read_config(config_file)
    
    # 명령어 생성
    cmd = build_command(config)
    
    # 명령어 출력
    print("실행 명령어:", " ".join(cmd))
    
    # Python 스크립트 실행
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"오류: Python 스크립트 실행 중 오류가 발생했습니다. (종료 코드: {e.returncode})")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n사용자에 의해 중단되었습니다.")
        sys.exit(0)

if __name__ == "__main__":
    main()
