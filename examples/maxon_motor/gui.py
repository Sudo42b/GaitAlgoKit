import tkinter as tk
from tkinter import messagebox, StringVar, IntVar
import threading
import time
from ctypes import *

class EPOSConstants:
    """EPOS 모터 컨트롤러 상수 정의"""
    # 작동 모드 
    OMD_PROFILE_POSITION_MODE = 1
    OMD_PROFILE_VELOCITY_MODE = 3
    OMD_HOMING_MODE = 6
    OMD_INTERPOLATED_POSITION_MODE = 7
    OMD_POSITION_MODE = -1
    OMD_VELOCITY_MODE = -2
    OMD_CURRENT_MODE = -3
    OMD_MASTER_ENCODER_MODE = -5
    OMD_STEP_DIRECTION_MODE = -6

    # 상태 정의
    ST_DISABLED = 0
    ST_ENABLED = 1
    ST_QUICKSTOP = 2
    ST_FAULT = 3

    # 모터 타입
    MT_DC_MOTOR = 1
    MT_EC_SINUS_COMMUTATED_MOTOR = 10
    MT_EC_BLOCK_COMMUTATED_MOTOR = 11

    # 모드 이름 매핑
    MODE_NAMES = {
        -6: "Step/Direction Mode",
        -5: "Master Encoder Mode", 
        -3: "Current Mode", 
        -2: "Velocity Mode", 
        -1: "Position Mode", 
        1: "Profile Position Mode", 
        3: "Profile Velocity Mode", 
        6: "Homing Mode", 
        7: "Interpolated Position Mode"
    }


class EPOSController:
    """EPOS 모터 컨트롤러 클래스"""
    def __init__(self, dll_path='./EposCmd64.dll'):
        """EPOS 모터 컨트롤러 초기화"""
        # DLL 로드
        try:
            self.epos = cdll.LoadLibrary(dll_path)
        except Exception as e:
            messagebox.showerror("DLL 로드 오류", f"EPOS DLL을 로드할 수 없습니다: {str(e)}")
            raise e

        # 기본 변수 초기화
        self.key_handle = 0
        self.node_id = 1
        self.error_code = c_uint()
        self.immediately = True
        self.update_active = False
        self.update_timer = None
        self.initialization = False

        # 모터 설정값
        self.mode = 0
        self.profile_velocity = 0
        self.profile_acceleration = 0
        self.profile_deceleration = 0
        self.actual_value = 0
        self.start_position = 0
        self.target_position = 2000

    def open_device(self):
        """EPOS 장치 연결"""
        # 이전 연결 닫기
        if self.key_handle:
            self.epos.VCS_CloseDevice(self.key_handle, byref(self.error_code))
            self.key_handle = 0

        # USB 연결
        self.key_handle = self.epos.VCS_OpenDevice(b'EPOS2', b'MAXON SERIAL V2', b'USB', b'USB0', byref(self.error_code))
        
        if not self.key_handle:
            self.show_error_information(self.error_code.value)
            messagebox.showerror("연결 오류", "장치를 열 수 없습니다!")
            return False

        # 통신 설정
        if not self.epos.VCS_SetProtocolStackSettings(self.key_handle, 1000000, 500, byref(self.error_code)):
            self.show_error_information(self.error_code.value)
            return False

        # 오류 초기화
        if not self.epos.VCS_ClearFault(self.key_handle, self.node_id, byref(self.error_code)):
            self.show_error_information(self.error_code.value)
            return False

        # 작동 모드 확인
        mode = c_byte()
        if not self.epos.VCS_GetOperationMode(self.key_handle, self.node_id, byref(mode), byref(self.error_code)):
            self.show_error_information(self.error_code.value)
            return False
        self.mode = mode.value

        # 위치 프로파일 값 읽기
        default_velocity = 100
        default_acceleration = 5000
        default_deceleration = 5000
        if not self.epos.VCS_SetPositionProfile(self.key_handle, self.node_id, default_velocity, default_acceleration, default_deceleration, byref(self.error_code)):
            self.show_error_information(self.error_code.value)
            return False

        # 변수에 저장
        self.profile_velocity = default_velocity
        self.profile_acceleration = default_acceleration
        self.profile_deceleration = default_deceleration

        # 작동 모드를 위치 프로파일 모드로 설정
        if not self.epos.VCS_SetOperationMode(self.key_handle, self.node_id, EPOSConstants.OMD_PROFILE_POSITION_MODE, byref(self.error_code)):
            self.show_error_information(self.error_code.value)
            return False

        # 현재 위치 읽기
        position_is = c_long()
        if not self.epos.VCS_GetPositionIs(self.key_handle, self.node_id, byref(position_is), byref(self.error_code)):
            self.show_error_information(self.error_code.value)
            return False
        
        self.start_position = position_is.value
        self.initialization = True
        return True
    
    def set_position_profile(self, velocity, acceleration, deceleration):
        """위치 프로파일 설정"""
        if not self.epos.VCS_SetPositionProfile(self.key_handle, self.node_id, velocity, acceleration, deceleration, byref(self.error_code)):
            self.show_error_information(self.error_code.value)
            return False
        
        # 설정 값 저장
        self.profile_velocity = velocity
        self.profile_acceleration = acceleration
        self.profile_deceleration = deceleration
        
        return True
    
    def set_enable_state(self):
        """모터 활성화"""
        # 오류 상태 확인
        fault = c_int(0)
        if not self.epos.VCS_GetFaultState(self.key_handle, self.node_id, byref(fault), byref(self.error_code)):
            self.show_error_information(self.error_code.value)
            return False

        # 오류가 있으면 초기화
        if fault.value:
            if not self.epos.VCS_ClearFault(self.key_handle, self.node_id, byref(self.error_code)):
                self.show_error_information(self.error_code.value)
                return False

        # 모터 활성화
        if not self.epos.VCS_SetEnableState(self.key_handle, self.node_id, byref(self.error_code)):
            self.show_error_information(self.error_code.value)
            return False
        
        return True

    def set_disable_state(self):
        """모터 비활성화"""
        if not self.epos.VCS_SetDisableState(self.key_handle, self.node_id, byref(self.error_code)):
            self.show_error_information(self.error_code.value)
            return False
        return True

    def move_to_position(self, target_position, absolute=True):
        """목표 위치로 이동"""
        # 현재 위치 읽기
        position_is = c_long()
        if not self.epos.VCS_GetPositionIs(self.key_handle, self.node_id, byref(position_is), byref(self.error_code)):
            self.show_error_information(self.error_code.value)
            return False
        
        self.start_position = position_is.value
        
        # 이동 명령 실행
        if not self.epos.VCS_MoveToPosition(self.key_handle, self.node_id, target_position, absolute, self.immediately, byref(self.error_code)):
            self.show_error_information(self.error_code.value)
            return False
        
        return True

    def halt_position_movement(self):
        """이동 정지"""
        if not self.epos.VCS_HaltPositionMovement(self.key_handle, self.node_id, byref(self.error_code)):
            self.show_error_information(self.error_code.value)
            return False
        return True

    def get_position_is(self):
        """현재 위치 읽기"""
        position_is = c_long()
        if not self.epos.VCS_GetPositionIs(self.key_handle, self.node_id, byref(position_is), byref(self.error_code)):
            self.show_error_information(self.error_code.value)
            return None
        return position_is.value

    def update_status(self, gui=None):
        """상태 업데이트"""
        if not self.update_active:
            return False

        # 작동 모드 확인
        mode = c_byte()
        if not self.epos.VCS_GetOperationMode(self.key_handle, self.node_id, byref(mode), byref(self.error_code)):
            self.stop_timer()
            self.show_error_information(self.error_code.value)
            return False
        
        self.mode = mode.value
        
        # 모드 이름 설정
        mode_str = EPOSConstants.MODE_NAMES.get(self.mode, "Unknown Mode")
        
        # 활성화 상태 확인
        enable = c_int(0)
        if not self.epos.VCS_GetEnableState(self.key_handle, self.node_id, byref(enable), byref(self.error_code)):
            self.stop_timer()
            self.show_error_information(self.error_code.value)
            return False
        
        # 현재 위치 읽기
        position_is = c_long()
        if not self.epos.VCS_GetPositionIs(self.key_handle, self.node_id, byref(position_is), byref(self.error_code)):
            self.stop_timer()
            self.show_error_information(self.error_code.value)
            return False
        
        self.actual_value = position_is.value
        
        # GUI 업데이트
        if gui:
            gui.update_ui(mode_str, enable.value, self.actual_value)
        
        return True

    def stop_timer(self):
        """타이머 정지"""
        self.update_active = False
        if self.update_timer:
            self.update_timer.cancel()
            self.update_timer = None

    def restore_epos(self):
        """초기 설정으로 복원"""
        if self.initialization:
            self.epos.VCS_SetDisableState(self.key_handle, self.node_id, byref(self.error_code))
            self.epos.VCS_SetOperationMode(self.key_handle, self.node_id, self.mode, byref(self.error_code))
            self.epos.VCS_SetPositionProfile(self.key_handle, self.node_id, self.profile_velocity, 
                                           self.profile_acceleration, self.profile_deceleration, byref(self.error_code))
        
        self.stop_timer()
        
        if self.key_handle:
            self.epos.VCS_CloseDevice(self.key_handle, byref(self.error_code))
            self.key_handle = 0

    def show_error_information(self, error_code):
        """오류 정보 표시"""
        buffer_size = 100
        error_info = create_string_buffer(buffer_size)
        
        if self.epos.VCS_GetErrorInfo(error_code, error_info, buffer_size):
            messagebox.showerror("EPOS 오류", error_info.value.decode('utf-8', errors='ignore'))
            return True
        else:
            messagebox.showerror("오류", "오류 정보를 읽을 수 없습니다!")
            return False


class EPOSControllerGUI:
    """EPOS 모터 컨트롤러 GUI 클래스"""
    def __init__(self, master):
        """GUI 초기화"""
        self.master = master
        master.title("EPOS 모터 컨트롤러")
        # master.geometry("500x400")
        # master.resizable(False, False)
        # 아이콘 설정
        try:
            master.iconbitmap("Demo_WinDLL.ico")
        except:
            # 아이콘 파일이 없는 경우 오류 처리
            print("아이콘 파일을 찾을 수 없습니다.")
        
        # EPOS 컨트롤러 생성
        self.controller = EPOSController()
        
        # 변수 초기화
        self.node_id_var = StringVar(value="1")
        self.start_position_var = IntVar(value=0)
        self.target_position_var = IntVar(value=2000)
        self.actual_value_var = IntVar(value=0)
        self.active_mode_var = StringVar(value="Unknown Mode")
        self.move_type_var = IntVar(value=1)  # 1=absolute, 0=relative
        
        # 프레임 생성
        top_frame = tk.Frame(master)
        top_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Node ID 입력
        tk.Label(top_frame, text="Node ID:").pack(side=tk.LEFT)
        node_id_entry = tk.Entry(top_frame, textvariable=self.node_id_var, width=5)
        node_id_entry.pack(side=tk.LEFT, padx=5)
        node_id_entry.bind("<Return>", self.on_node_id_change)
        
        # 장치 설정 버튼
        self.device_settings_btn = tk.Button(top_frame, text="장치 설정", command=self.on_device_settings)
        self.device_settings_btn.pack(side=tk.LEFT, padx=10)
        
        # 상태 표시
        status_frame = tk.Frame(master)
        status_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Label(status_frame, text="현재 모드:").grid(row=0, column=0, sticky=tk.W)
        tk.Label(status_frame, textvariable=self.active_mode_var).grid(row=0, column=1, sticky=tk.W)
        
        # 위치 설정
        position_frame = tk.Frame(master)
        position_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Label(position_frame, text="목표 위치:").grid(row=0, column=0, sticky=tk.W)
        tk.Entry(position_frame, textvariable=self.target_position_var).grid(row=0, column=1, sticky=tk.W)
        
        tk.Label(position_frame, text="현재 위치:").grid(row=1, column=0, sticky=tk.W)
        tk.Label(position_frame, textvariable=self.actual_value_var).grid(row=1, column=1, sticky=tk.W)
        
        tk.Label(position_frame, text="시작 위치:").grid(row=2, column=0, sticky=tk.W)
        tk.Label(position_frame, textvariable=self.start_position_var).grid(row=2, column=1, sticky=tk.W)
        # 위치 설정 프레임 아래에 속도 설정 프레임 추가
        velocity_frame = tk.Frame(master)
        velocity_frame.pack(fill=tk.X, padx=10, pady=5)

        tk.Label(velocity_frame, text="속도(rpm):").grid(row=0, column=0, sticky=tk.W)
        self.velocity_var = IntVar(value=100)
        tk.Entry(velocity_frame, textvariable=self.velocity_var, width=10).grid(row=0, column=1, sticky=tk.W)

        tk.Label(velocity_frame, text="가속도(rpm/s):").grid(row=1, column=0, sticky=tk.W)
        self.acceleration_var = IntVar(value=1000)
        tk.Entry(velocity_frame, textvariable=self.acceleration_var, width=10).grid(row=1, column=1, sticky=tk.W)

        tk.Label(velocity_frame, text="감속도(rpm/s):").grid(row=2, column=0, sticky=tk.W)
        self.deceleration_var = IntVar(value=1000)
        tk.Entry(velocity_frame, textvariable=self.deceleration_var, width=10).grid(row=2, column=1, sticky=tk.W)

        # 위치 프로파일 설정 버튼 추가
        set_profile_btn = tk.Button(velocity_frame, text="프로파일 설정", command=self.on_set_profile)
        set_profile_btn.grid(row=3, column=0, columnspan=2, sticky=tk.W+tk.E, pady=5)
        
        # 모드 선택
        mode_frame = tk.Frame(master)
        mode_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Radiobutton(mode_frame, text="상대 이동", variable=self.move_type_var, value=0).pack(side=tk.LEFT)
        tk.Radiobutton(mode_frame, text="절대 이동", variable=self.move_type_var, value=1).pack(side=tk.LEFT)
        
        # 제어 버튼
        control_frame = tk.Frame(master)
        control_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.enable_btn = tk.Button(control_frame, text="활성화", command=self.on_enable, state=tk.DISABLED)
        self.enable_btn.pack(side=tk.LEFT, padx=5)
        
        self.disable_btn = tk.Button(control_frame, text="비활성화", command=self.on_disable, state=tk.DISABLED)
        self.disable_btn.pack(side=tk.LEFT, padx=5)
        
        self.move_btn = tk.Button(control_frame, text="이동", command=self.on_move, state=tk.DISABLED)
        self.move_btn.pack(side=tk.LEFT, padx=5)
        
        self.halt_btn = tk.Button(control_frame, text="정지", command=self.on_halt, state=tk.DISABLED)
        self.halt_btn.pack(side=tk.LEFT, padx=5)
        
        # 종료 시 자원 해제
        master.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 초기 연결 시도
        self.on_device_settings()
        # 창 크기를 내용에 맞게 조정
        master.update_idletasks()  # 모든 위젯이 배치되도록 업데이트
        
        # 최소 크기 설정 (선택 사항)
        width = max(master.winfo_reqwidth(), 300)  # 최소 너비 300
        height = max(master.winfo_reqheight(), 250)  # 최소 높이 250
        
        # 화면 중앙에 위치시키기 위한 위치 계산
        x = (master.winfo_screenwidth() - width) // 2
        y = (master.winfo_screenheight() - height) // 2
        
        # 창 크기와 위치 설정
        master.geometry(f"{width}x{height}+{x}+{y}")
        
        # 창 크기 조정 금지 (선택 사항)
        master.resizable(False, False)
    
    def on_set_profile(self):
        """속도 프로파일 설정 처리"""
        try:
            velocity = self.velocity_var.get()
            acceleration = self.acceleration_var.get()
            deceleration = self.deceleration_var.get()
            
            # 유효한 값인지 확인
            if velocity <= 0 or acceleration <= 0 or deceleration <= 0:
                messagebox.showinfo("입력 오류", "속도, 가속도, 감속도는 양수여야 합니다!")
                return
            
            # 프로파일 설정
            if not self.controller.set_position_profile(velocity, acceleration, deceleration):
                messagebox.showinfo("오류", "프로파일 설정에 실패했습니다!")
                return
                
            messagebox.showinfo("성공", "위치 프로파일이 설정되었습니다.")
        except ValueError:
            messagebox.showinfo("입력 오류", "올바른 숫자를 입력하세요!")
    
    def on_device_settings(self):
        """장치 설정 버튼 처리"""
        try:
            self.controller.node_id = int(self.node_id_var.get())
            
            if self.controller.open_device():
                self.start_update_timer()
            else:
                self.controller.initialization = False
                self.controller.stop_timer()
                
                # 버튼 비활성화
                self.enable_btn.config(state=tk.DISABLED)
                self.disable_btn.config(state=tk.DISABLED)
                self.move_btn.config(state=tk.DISABLED)
                self.halt_btn.config(state=tk.DISABLED)
        except ValueError:
            messagebox.showinfo("입력 오류", "올바른 노드 ID를 입력하세요!")

    def on_enable(self):
        """활성화 버튼 처리"""
        self.controller.set_enable_state()

    def on_disable(self):
        """비활성화 버튼 처리"""
        self.controller.set_disable_state()

    def on_move(self):
        """이동 버튼 처리"""
        self.controller.target_position = self.target_position_var.get()
        absolute = (self.move_type_var.get() == 1)
        
        if self.controller.move_to_position(self.controller.target_position, absolute):
            # 현재 위치 업데이트
            position = self.controller.get_position_is()
            if position is not None:
                self.start_position_var.set(position)

    def on_halt(self):
        """정지 버튼 처리"""
        self.controller.halt_position_movement()

    def on_node_id_change(self, event=None):
        """노드 ID 변경 처리"""
        try:
            node_id = int(self.node_id_var.get())
            if 1 <= node_id <= 127:
                self.controller.node_id = node_id
                self.start_update_timer()
            else:
                messagebox.showinfo("입력 오류", "노드 ID는 1에서 127 사이여야 합니다!")
                self.node_id_var.set("1")
        except ValueError:
            messagebox.showinfo("입력 오류", "올바른 노드 ID를 입력하세요!")
            self.node_id_var.set("1")

    def start_update_timer(self):
        """상태 업데이트 타이머 시작"""
        self.controller.update_active = True
        self.update_status()

    def update_status(self):
        """상태 업데이트 함수"""
        if self.controller.update_active:
            self.controller.update_status(self)
            # 100ms마다 업데이트
            self.controller.update_timer = threading.Timer(0.1, self.update_status)
            self.controller.update_timer.daemon = True
            self.controller.update_timer.start()

    def update_ui(self, mode_str, enable, actual_value):
        """UI 요소 업데이트"""
        self.active_mode_var.set(mode_str)
        self.actual_value_var.set(actual_value)
        
        # 버튼 상태 업데이트
        self.device_settings_btn.config(state=tk.NORMAL if not enable else tk.DISABLED)
        self.enable_btn.config(state=tk.NORMAL if not enable else tk.DISABLED)
        self.disable_btn.config(state=tk.NORMAL if enable else tk.DISABLED)
        self.move_btn.config(state=tk.NORMAL if enable else tk.DISABLED)
        self.halt_btn.config(state=tk.NORMAL if enable else tk.DISABLED)
        
        # move 버튼 텍스트 업데이트
        if self.move_type_var.get() == 0:
            self.move_btn.config(text="상대 이동")
        else:
            self.move_btn.config(text="절대 이동")

    def on_closing(self):
        """프로그램 종료 처리"""
        self.controller.restore_epos()
        self.master.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = EPOSControllerGUI(root)
    root.mainloop()