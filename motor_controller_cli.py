# motor_controller_cli.py
"""
Command-line interface for testing motor controllers.
This is a Python port of the Rust motor controller test CLI.
"""

import argparse
import time
import math
import sys
import logging
from typing import Dict, List, Optional
import threading
from can_communication import MotorType
# Import the Python bindings for the motor controller
from bindings import (
    RobstrideMotorsSupervisor,
    RobstrideMotorControlParams,
    RobstrideMotorFeedback
)

def sinusoid(
    controller: RobstrideMotorsSupervisor,
    motor_id: int,
    amplitude: float,
    duration_secs: float
) -> None:
    """
    Run a sinusoidal motion on the specified motor.
    
    Args:
        controller: Motor controller supervisor
        motor_id: ID of the motor to control
        amplitude: Amplitude of the sinusoidal motion
        duration_secs: Duration of the motion in seconds
    """
    start_time = time.time()
    
    # Set initial parameters
    controller.set_kd(motor_id, 1.0)
    controller.set_kp(motor_id, 10.0)
    controller.set_velocity(motor_id, 0.0)
    controller.set_torque(motor_id, 0.0)
    
    while time.time() - start_time < duration_secs:
        t = time.time() - start_time
        pos = amplitude * math.sin(2.0 * math.pi * t)
        controller.set_position(motor_id, pos)
        time.sleep(0.01)  # 10ms sleep
    
    # Return to zero position
    controller.set_position(motor_id, 0.0)

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Motor Controller Test CLI")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose output")
    parser.add_argument("--min-update-rate", type=float, default=10.0, help="Minimum update rate (Hz)")
    parser.add_argument("--max-update-rate", type=float, default=1000.0, help="Maximum update rate (Hz)")
    parser.add_argument("--can-timeout", type=float, default=200.0, help="CAN timeout (ms)")
    args = parser.parse_args()
    
    # Configure logging
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Get user input for test configuration
    try:
        test_id = int(input("Enter the TEST_MOTOR_ID (u8): "))
        
        port_name = input("Enter the port name (default: /dev/ttyUSB0): ")
        if not port_name:
            port_name = "/dev/ttyUSB0"
        
        motor_type_input = input("Enter the motor type (default: 01): ")
        if not motor_type_input:
            motor_type_input = MotorType.Type01
        
        # Create motor controller
        motor_infos = {test_id: motor_type_input}
        controller = RobstrideMotorsSupervisor(
            port_name=port_name,
            motor_infos=motor_infos,
            verbose=args.verbose,
            target_update_rate=args.max_update_rate,
            can_timeout=args.can_timeout
        )
        
        # Display available commands
        print("Motor Controller Test CLI")
        print("Available commands:")
        print("  p <position>")
        print("  v <velocity>")
        print("  t <torque>")
        print("  kp <kp>")
        print("  kd <kd>")
        print("  sinusoid / s")
        print("  zero / z")
        print("  get_feedback / g")
        print("  pause / w")
        print("  reset / r")
        print("  quit / q")
        
        # Main command loop
        while True:
            try:
                cmd_input = input("> ")
                parts = cmd_input.strip().split()
                
                if not parts:
                    continue
                
                cmd = parts[0]
                
                if cmd == "p":
                    if len(parts) != 2:
                        print("Usage: p <position>")
                        continue
                    position = float(parts[1])
                    controller.set_position(test_id, position)
                    print(f"Set target position to {position}")
                
                elif cmd == "v":
                    if len(parts) != 2:
                        print("Usage: v <velocity>")
                        continue
                    velocity = float(parts[1])
                    controller.set_velocity(test_id, velocity)
                    print(f"Set target velocity to {velocity}")
                
                elif cmd == "t":
                    if len(parts) != 2:
                        print("Usage: t <torque>")
                        continue
                    torque = float(parts[1])
                    controller.set_torque(test_id, torque)
                    print(f"Set target torque to {torque}")
                
                elif cmd == "kp":
                    if len(parts) != 2:
                        print("Usage: kp <kp>")
                        continue
                    kp = float(parts[1])
                    controller.set_kp(test_id, kp)
                    print(f"Set KP for motor {test_id} to {kp}")
                
                elif cmd == "kd":
                    if len(parts) != 2:
                        print("Usage: kd <kd>")
                        continue
                    kd = float(parts[1])
                    controller.set_kd(test_id, kd)
                    print(f"Set KD for motor {test_id} to {kd}")
                
                elif cmd in ["sinusoid", "s"]:
                    # Run in the main thread to handle keyboard interrupts
                    print(f"Running sinusoid test on motor {test_id}...")
                    sinusoid(controller, test_id, 1.0, 1.0)
                    print(f"Completed sinusoid test on motor {test_id}")
                
                elif cmd in ["zero", "z"]:
                    controller.add_motor_to_zero(test_id)
                    print(f"Added motor {test_id} to zero list")
                
                elif cmd in ["get_feedback", "g"]:
                    feedback = controller.get_latest_feedback()
                    for motor_id, fb in feedback.items():
                        print(f"Motor {motor_id}: {fb}")
                
                elif cmd in ["pause", "w"]:
                    controller.toggle_pause()
                    print("Toggled pause state")
                
                elif cmd in ["reset", "r"]:
                    controller.reset()
                    print("Reset motors")
                
                elif cmd in ["quit", "q"]:
                    controller.stop()
                    print("Exiting...")
                    break
                
                else:
                    print("Unknown command")
            
            except KeyboardInterrupt:
                print("\nOperation interrupted by user.")
            
            except ValueError as e:
                print(f"Invalid value: {e}")
            
            except Exception as e:
                print(f"Error: {e}")
    
    except KeyboardInterrupt:
        print("\nExiting...")
    
    except ValueError as e:
        print(f"Invalid input: {e}")
    
    except Exception as e:
        print(f"Error: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())