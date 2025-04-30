# bindings.py
"""
Python bindings for the Robstride motor control library.
This module provides Python classes that correspond to the original Rust implementation.
"""

from typing import Dict, List, Optional, Union, Tuple
import logging
from dataclasses import dataclass
from enum import Enum, IntEnum

# Import the motor control implementation
from .can_communication import (
    Motors as BaseMotors,
    MotorsSupervisor as BaseMotorsSupervisor,
    MotorControlParams as BaseMotorControlParams,
    MotorFeedback as BaseMotorFeedback,
    ROBSTRIDE_CONFIGS,
    MotorType,
    RunMode
)

class RobstrideMotorFeedback:
    """Python wrapper for motor feedback information."""
    def __init__(self, 
                 can_id: int, 
                 position: float, 
                 velocity: float, 
                 torque: float, 
                 mode: str, 
                 faults: int):
            
        self.can_id = can_id
        self.position = position
        self.velocity = velocity
        self.torque = torque
        self.mode = mode
        self.faults = faults
    
    def __repr__(self) -> str:
        return (f"RobstrideMotorFeedback(can_id={self.can_id}, "
                f"position={self.position:.2f}, velocity={self.velocity:.2f}, "
                f"torque={self.torque:.2f}, mode='{self.mode}', faults={self.faults})")
    
    @classmethod
    def from_base(cls, feedback: BaseMotorFeedback) -> 'RobstrideMotorFeedback':
        """Convert from internal MotorFeedback to RobstrideMotorFeedback."""
        return cls(
            can_id=feedback.can_id,
            position=feedback.position,
            velocity=feedback.velocity,
            torque=feedback.torque,
            mode=str(feedback.mode),
            faults=feedback.faults
        )


class RobstrideMotorControlParams:
    """Python wrapper for motor control parameters."""
    position: float = 0.0
    velocity: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    torque: float = 0.0
    def __init__(self, 
                 position: float = 0.0, 
                 velocity: float = 0.0, 
                 kp: float = 0.0, 
                 kd: float = 0.0, 
                 torque: float = 0.0):
        position = position
        velocity = velocity
        kp = kp
        kd = kd
        torque = torque
    
    def __repr__(self) -> str:
        return (f"RobstrideMotorControlParams(position={RobstrideMotorControlParams.position:.2f}, "
                f"velocity={RobstrideMotorControlParams.velocity:.2f}, kp={RobstrideMotorControlParams.kp:.2f}, "
                f"kd={RobstrideMotorControlParams.kd:.2f}, torque={RobstrideMotorControlParams.torque:.2f})")
    
    def to_base(self) -> BaseMotorControlParams:
        """Convert to internal MotorControlParams."""
        return BaseMotorControlParams(
            position=RobstrideMotorControlParams.position,
            velocity=RobstrideMotorControlParams.velocity,
            kp=RobstrideMotorControlParams.kp,
            kd=RobstrideMotorControlParams.kd,
            torque=RobstrideMotorControlParams.torque
        )
    
    @classmethod
    def from_base(cls, params: BaseMotorControlParams) -> 'RobstrideMotorControlParams':
        """Convert from internal MotorControlParams to RobstrideMotorControlParams."""
        return cls(
            position=params.position,
            velocity=params.velocity,
            kp=params.kp,
            kd=params.kd,
            torque=params.torque
        )


class RobstrideMotors:
    """Python wrapper for the Motors class."""
    
    def __init__(self, port_name: str, motor_infos: Dict[int, str], verbose: bool = False):
        """
        Initialize a new Motors instance.
        
        Args:
            port_name: Path to the serial port device.
            motor_infos: Dictionary mapping motor IDs to motor type strings.
            verbose: Whether to enable verbose logging.
        """
        # Convert string motor types to MotorType enum
        motor_types = {}
        for motor_id, type_str in motor_infos.items():
            try:
                motor_type = self._motor_type_from_str(type_str)
                motor_types[motor_id] = motor_type
            except ValueError as e:
                raise ValueError(f"Invalid motor type: {type_str}") from e
        
        self.inner = BaseMotors(port_name, motor_types, verbose)
    
    def _motor_type_from_str(self, type_str: str) -> MotorType:
        """Convert a string motor type to a MotorType enum value."""
        try:
            return MotorType(type_str)
        except ValueError:
            raise ValueError(f"Invalid motor type: {type_str}")
    
    def send_get_mode(self) -> Dict[int, str]:
        """Get the current mode of all motors."""
        return {id: str(mode) for id, mode in self.inner.send_get_mode().items()}
    
    def send_set_zero(self, motor_ids: Optional[List[int]] = None) -> None:
        """
        Zero specified motors or all motors if none specified.
        
        Args:
            motor_ids: Optional list of motor IDs to zero.
        """
        self.inner.send_set_zeros(motor_ids)
    
    def send_resets(self) -> None:
        """Reset all motors."""
        self.inner.send_resets()
    
    def send_starts(self) -> None:
        """Start all motors."""
        self.inner.send_starts()
    
    def send_motor_controls(
        self, 
        motor_controls: Dict[int, RobstrideMotorControlParams], 
        serial: bool = True
    ) -> Dict[int, RobstrideMotorFeedback]:
        """
        Send control commands to motors.
        
        Args:
            motor_controls: Dictionary mapping motor IDs to control parameters.
            serial: Whether to send commands serially (one at a time).
            
        Returns:
            Dictionary mapping motor IDs to feedback data.
        """
        # Convert RobstrideMotorControlParams to BaseMotorControlParams
        base_controls = {id: params.to_base() for id, params in motor_controls.items()}
        
        # Send commands and get feedback
        base_feedbacks = self.inner.send_motor_controls(base_controls, serial)
        
        # Convert BaseMotorFeedback to RobstrideMotorFeedback
        return {id: RobstrideMotorFeedback.from_base(feedback) 
                for id, feedback in base_feedbacks.items()}
    
    def __repr__(self) -> str:
        return "RobstrideMotors()"


class RobstrideMotorsSupervisor:
    """Python wrapper for the MotorsSupervisor class."""
    total_commands: int
    actual_update_rate: float
    serial: bool
    def __init__(
        self, 
        port_name: str, 
        motor_infos: Dict[int, str], 
        verbose: bool = False, 
        target_update_rate: float = 50.0, 
        zero_on_init:bool = False
    ):
        """
        Initialize a new MotorsSupervisor instance.
        
        Args:
            port_name: Path to the serial port device.
            motor_infos: Dictionary mapping motor IDs to motor type strings.
            verbose: Whether to enable verbose logging.
            target_update_rate: Target rate for updating motor controls (Hz).
            can_timeout: Timeout for CAN communication (ms).
        """
        # Convert string motor types to MotorType enum
        motor_types = {}
        for motor_id, type_str in motor_infos.items():
            try:
                motor_type = self._motor_type_from_str(type_str)
                motor_types[motor_id] = motor_type
            except ValueError as e:
                raise ValueError(f"Invalid motor type: {type_str}") from e
        
        self.inner = BaseMotorsSupervisor(
            port_name=port_name,
            motor_infos=motor_types,
            verbose=verbose,
            max_update_rate=target_update_rate,
            zero_on_init=zero_on_init
        )
    
    def _motor_type_from_str(self, type_str: str) -> MotorType:
        """Convert a string motor type to a MotorType enum value."""
        try:
            return MotorType(type_str)
        except ValueError:
            raise ValueError(f"Invalid motor type: {type_str}")
    
    def set_position(self, motor_id: int, position: float) -> float:
        """
        Set the target position for a motor.
        
        Args:
            motor_id: ID of the motor.
            position: Target position (radians).
        
        Returns:
            The set position value.
        """
        try:
            # get motor type and min/max position
            motor_config = self.inner.motors.motor_configs[motor_id]
            p_min = motor_config.p_min
            p_max = motor_config.p_max
            
            # Clamp position within limits
            clamped_position = max(p_min, min(p_max, position))
            
            return self.inner.set_position(motor_id, clamped_position)
        except ValueError as e:
            raise RuntimeError(str(e))
    
    def get_position(self, motor_id: int) -> float:
        """
        Get the current target position for a motor.
        
        Args:
            motor_id: ID of the motor.
        
        Returns:
            The current target position (radians).
        """
        try:
            return self.inner.get_position(motor_id)
        except ValueError as e:
            raise RuntimeError(str(e))
    
    def set_velocity(self, motor_id: int, velocity: float) -> float:
        """
        Set the target velocity for a motor.
        
        Args:
            motor_id: ID of the motor.
            velocity: Target velocity (radians/second).
        
        Returns:
            The set velocity value.
        """
        try:
            # get motor type and min/max velocity
            motor_config = self.inner.motors.motor_configs[motor_id]
            v_min = motor_config.v_min
            v_max = motor_config.v_max
            
            # Clamp velocity within limits
            clamped_velocity = max(v_min, min(v_max, velocity))
            return self.inner.set_velocity(motor_id, clamped_velocity)
        except ValueError as e:
            raise RuntimeError(str(e))
    
    def get_velocity(self, motor_id: int) -> float:
        """
        Get the current target velocity for a motor.
        
        Args:
            motor_id: ID of the motor.
        
        Returns:
            The current target velocity (radians/second).
        """
        try:
            return self.inner.get_velocity(motor_id)
        except ValueError as e:
            raise RuntimeError(str(e))
    
    def set_kp(self, motor_id: int, kp: float) -> float:
        """
        Set the proportional gain for a motor.
        
        Args:
            motor_id: ID of the motor.
            kp: Proportional gain.
        
        Returns:
            The set kp value.
        """
        try:
            # get motor type and min/max kp 
            motor_config = self.inner.motors.motor_configs[motor_id]
            kp_min = motor_config.kp_min
            kp_max = motor_config.kp_max
            
            # Clamp kp within limits
            clamped_kp = max(kp_min, min(kp_max, kp))
            return self.inner.set_kp(motor_id, clamped_kp)
        except ValueError as e:
            raise RuntimeError(str(e))
    
    def get_kp(self, motor_id: int) -> float:
        """
        Get the current proportional gain for a motor.
        
        Args:
            motor_id: ID of the motor.
        
        Returns:
            The current proportional gain.
        """
        try:
            return self.inner.get_kp(motor_id)
        except ValueError as e:
            raise RuntimeError(str(e))
    
    def set_kd(self, motor_id: int, kd: float) -> float:
        """
        Set the derivative gain for a motor.
        
        Args:
            motor_id: ID of the motor.
            kd: Derivative gain.
        
        Returns:
            The set kd value.
        """
        try:
            # get motor type and min/max kd 
            motor_config = self.inner.motors.motor_configs[motor_id]
            kd_min = motor_config.kd_min
            kd_max = motor_config.kd_max
            
            # Clamp kd within limits
            clamped_kd = max(kd_min, min(kd_max, kd))
            return self.inner.set_kd(motor_id, clamped_kd)
        except ValueError as e:
            raise RuntimeError(str(e))
    
    def get_kd(self, motor_id: int) -> float:
        """
        Get the current derivative gain for a motor.
        
        Args:
            motor_id: ID of the motor.
        
        Returns:
            The current derivative gain.
        """
        try:
            return self.inner.get_kd(motor_id)
        except ValueError as e:
            raise RuntimeError(str(e))
    
    def set_torque(self, motor_id: int, torque: float) -> float:
        """
        Set the target torque for a motor.
        
        Args:
            motor_id: ID of the motor.
            torque: Target torque (Nm).
        
        Returns:
            The set torque value.
        """
        try:
            motor_config = self.inner.motors.motor_configs[motor_id]
            t_min = motor_config.t_min
            t_max = motor_config.t_max
            
            # Clamp torque within limits
            clamped_torque = max(t_min, min(t_max, torque)) 
            return self.inner.set_torque(motor_id, clamped_torque)
        except ValueError as e:
            raise RuntimeError(str(e))
    
    def get_torque(self, motor_id: int) -> float:
        """
        Get the current target torque for a motor.
        
        Args:
            motor_id: ID of the motor.
        
        Returns:
            The current target torque (Nm).
        """
        try:
            return self.inner.get_torque(motor_id)
        except ValueError as e:
            raise RuntimeError(str(e))
    
    def add_motor_to_zero(self, motor_id: int) -> None:
        """
        Add a motor to the list of motors to be zeroed.
        
        Args:
            motor_id: ID of the motor to zero.
        """
        try:
            self.inner.add_motor_to_zero(motor_id)
        except ValueError as e:
            raise RuntimeError(str(e))
    
    def get_latest_feedback(self) -> Dict[int, RobstrideMotorFeedback]:
        """
        Get the latest feedback from all motors.
        
        Returns:
            Dictionary mapping motor IDs to feedback data.
        """
        base_feedbacks = self.inner.get_latest_feedback()
        return {id: RobstrideMotorFeedback.from_base(feedback) 
                for id, feedback in base_feedbacks.items()}
    
    def toggle_pause(self) -> None:
        """Toggle the paused state of the motor control loop."""
        self.inner.toggle_pause()
    
    def stop(self) -> None:
        """Stop the motor control loop."""
        self.inner.stop()
    
    def __repr__(self) -> str:
        motor_count = len(self.inner.get_latest_feedback())
        return f"RobstrideMotorsSupervisor(motor_count={motor_count})"
    
    def set_params(self, motor_id: int, params: RobstrideMotorControlParams) -> None:
        """
        Set all control parameters for a motor at once.
        
        Args:
            motor_id: ID of the motor.
            params: Control parameters.
        """
        self.inner.set_params(motor_id, params.to_base())
    
    @property
    def total_commands(self) -> int:
        """Get the total number of commands sent."""
        return self.inner.get_total_commands()
    
    def failed_commands_for(self, motor_id: int) -> int:
        """
        Get the number of failed commands for a specific motor.
        
        Args:
            motor_id: ID of the motor.
        
        Returns:
            Number of failed commands.
        """
        try:
            return self.inner.get_failed_commands(motor_id)
        except ValueError as e:
            raise RuntimeError(str(e))
    
    def reset_command_counters(self) -> None:
        """Reset all command counters."""
        self.inner.reset_command_counters()
    
    def is_running(self) -> bool:
        """Check if the motor control loop is running."""
        return self.inner.is_running()
    
    @property
    def max_update_rate(self) -> float:
        """Get the maximum update rate."""
        return self.inner.max_update_rate
    
    @max_update_rate.setter
    def max_update_rate(self, rate: float) -> None:
        """Set the maximum update rate."""
        self.inner.set_max_update_rate(rate)
    
    @property
    def actual_update_rate(self) -> float:
        """Get the actual update rate."""
        return self.inner.get_actual_update_rate()
    
    def toggle_serial(self) -> bool:
        """
        Toggle between serial and parallel command sending.
        
        Returns:
            The new serial setting.
        """
        return self.inner.toggle_serial()
    
    @property
    def serial(self) -> bool:
        """Get the current serial setting."""
        return self.inner.get_serial()


# Example usage
if __name__ == "__main__":
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    try:
        # Example motor configuration
        motor_infos = {
            1: "01",  # Type01 motor
            2: "03"   # Type03 motor
        }
        
        # Create a motors supervisor
        supervisor = RobstrideMotorsSupervisor(
            port_name="/dev/ttyUSB0",  # Replace with your actual port
            motor_infos=motor_infos,
            verbose=True,
            target_update_rate=100.0,
            can_timeout=500.0
        )
        
        logging.info("Motors initialized successfully")
        
        # Example of setting motor parameters
        supervisor.set_kp(1, 10.0)
        supervisor.set_kd(1, 0.5)
        supervisor.set_position(1, 5.0)
        
        params = RobstrideMotorControlParams(
            position=2.0,
            velocity=0.0,
            kp=20.0,
            kd=1.0,
            torque=0.0
        )
        supervisor.set_params(2, params)
        
        # Keep the script running for a short time
        try:
            import time
            for _ in range(10):
                time.sleep(0.1)
                
                # Example of getting feedback
                feedback = supervisor.get_latest_feedback()
                if feedback:
                    logging.info(f"Feedback: {feedback}")
        
        except KeyboardInterrupt:
            logging.info("User interrupted. Shutting down...")
        
        finally:
            supervisor.stop()
            logging.info("Motors stopped")
    
    except Exception as e:
        logging.error(f"Error: {e}")

"""
# Example: `python bindings.py`

이 코드는 CAN-to-USB 장치와의 통신에서 교환된 16진수 데이터를 보여줍니다. 
    "RX"는 장치로부터 받은(Received) 데이터이고, "TX"는 장치로 보낸(Transmitted) 데이터입니다. 
    각 값은 16진수로 표현된 바이트입니다.

## RX (수신 데이터) 분석:
```
41 54 10 00 0F EC 08 99 52 80 1C 7F FF 00 FD 0D 0A
```
- `41 54`: ASCII로 "AT" - 프로토콜의 시작 헤더
- `10 00 0F EC`: ExId(확장 ID) 정보 - 이 데이터는 모터 ID, 데이터, 통신 모드, 예약 비트 등을 포함
- `08`: 데이터 길이 (8바이트)
- `99 52 80 1C 7F FF 00 FD`: 실제 데이터 내용 (모터 피드백 정보를 포함)
- `0D 0A`: ASCII로 CR+LF, 패킷의 끝을 나타냄

### TX (송신 데이터) 분석:
```
41 54 20 07 E8 14 08 00 00 00 00 00 00 00 00 0D 0A
```
- `41 54`: ASCII로 "AT" - 프로토콜의 시작 헤더
- `20 07 E8 14`: ExId(확장 ID) 정보 - 이것은 명령의 유형, 대상 모터 ID 등을 포함
- `08`: 데이터 길이 (8바이트)
- `00 00 00 00 00 00 00 00`: 실제 데이터 내용 (모터에 전송되는 명령 또는 파라미터)
- `0D 0A`: ASCII로 CR+LF, 패킷의 끝을 나타냄

이 통신은 모터 제어 시스템에서 일반적으로 볼 수 있는 형태로, 특히 ExId 부분을 자세히 분석하면:

**RX의 ExId (10 00 0F EC)**:
- 이 값에서 `pack_ex_id` 함수로 추출하면 다음을 얻을 수 있습니다:
  - `id`: 모터 ID (아마도 0x7F 또는 관련 값)
  - `data`: 피드백 데이터 식별자
  - `mode`: `CanComMode::MotorFeedback`로 추정 (모터 피드백 패킷)
  - `res`: 예약된 비트

**TX의 ExId (20 07 E8 14)**:
- 이 값에서 `pack_ex_id` 함수로 추출하면 다음을 얻을 수 있습니다:
  - `id`: 모터 ID (아마도 제어하려는 모터)
  - `data`: 명령 데이터 식별자 (아마도 CAN_ID_DEBUG_UI 관련)
  - `mode`: `CanComMode::MotorReset` 또는 `CanComMode::MotorIn`으로 추정 (모터 초기화 명령)
  - `res`: 예약된 비트

이 통신 내용은 아마도 모터 컨트롤러에 초기화 또는 기본 명령을 보내고, 모터로부터 상태 피드백을 받는 과정의 일부로 보입니다.
"""