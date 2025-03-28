#!/usr/bin/env python3
# serial_port.py
"""
Enhanced serial port initialization with low-level control.
This implementation mirrors the Rust implementation closely.
"""

import os
import fcntl
import termios
import struct
import serial
import time
import logging
from typing import Optional

# Constants
BAUD_RATE = 921600
TIMEOUT_MS = 10  # 10ms timeout

def open_serial_port(device: str) -> serial.Serial:
    """
    Initialize a serial port with specific settings for CAN communication.
    
    Args:
        device (str): Path to the serial device (e.g., '/dev/ttyUSB0')
    
    Returns:
        serial.Serial: Configured serial port object
    
    Raises:
        IOError: If there's an error opening or configuring the serial port
    """
    try:
        # Open the device using low-level file operations
        fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        
        # Get the current flags
        flags = fcntl.fcntl(fd, fcntl.F_GETFL)
        
        # Clear the O_NONBLOCK flag to enable blocking I/O
        flags &= ~os.O_NONBLOCK
        fcntl.fcntl(fd, fcntl.F_SETFL, flags)
        
        # Get the current terminal attributes
        try:
            attrs = termios.tcgetattr(fd)
        except Exception as e:
            os.close(fd)
            raise IOError(f"Failed to get termios settings: {e}")
        
        # Build a new termios structure
        
        # Input flags - Clear all flags
        iflag = 0
        
        # Output flags - Disable output processing
        oflag = attrs[1] & ~termios.OPOST
        
        # Control flags
        cflag = attrs[2]
        # Clear parity bit, stop bits, and character size flags
        cflag &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
        # Set 8-bit characters, enable receiver, ignore modem control lines
        cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
        
        # Local flags - Disable canonical mode, echo, erasure, and signal chars
        lflag = attrs[3]
        lflag &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)
        
        # Control characters
        cc = attrs[6]
        cc[termios.VMIN] = 0  # Minimum number of characters for noncanonical read
        cc[termios.VTIME] = 10  # Timeout in deciseconds (1 second)
        
        # Create the new attributes tuple
        new_attrs = [iflag, oflag, cflag, lflag, attrs[4], attrs[5], cc]
        
        # Set baud rate
        try:
            # In Python, we need to use termios.BAUDRATE constants
            # Map 921600 to the appropriate constant or use a direct value
            baudrate_constant = getattr(termios, f'B{BAUD_RATE}', None)
            
            if baudrate_constant is None:
                # If the specific baud rate isn't defined as a constant,
                # we need to check if the system supports custom baud rates
                try:
                    # Try to set custom baud rate using cfsetispeed and cfsetospeed
                    # This is system-dependent and might not work on all platforms
                    custom_baud = BAUD_RATE
                    termios.cfsetispeed(new_attrs, custom_baud)
                    termios.cfsetospeed(new_attrs, custom_baud)
                except Exception as e:
                    os.close(fd)
                    raise IOError(f"Failed to set custom baud rate {BAUD_RATE}: {e}")
            else:
                # Use the standard baud rate constant
                termios.cfsetispeed(new_attrs, baudrate_constant)
                termios.cfsetospeed(new_attrs, baudrate_constant)
                
        except AttributeError:
            # If cfsetispeed/cfsetospeed aren't available, close and use pyserial approach
            os.close(fd)
            logging.warning("Low-level baud rate setting failed, using pyserial fallback")
            return _open_serial_port_pyserial(device)
        
        # Apply the new attributes
        try:
            termios.tcsetattr(fd, termios.TCSANOW, new_attrs)
        except Exception as e:
            os.close(fd)
            raise IOError(f"Failed to set termios attributes: {e}")
        
        # Verify the settings were applied correctly
        try:
            verify_attrs = termios.tcgetattr(fd)
            if (verify_attrs[2] != new_attrs[2] or 
                verify_attrs[0] != new_attrs[0] or 
                verify_attrs[1] != new_attrs[1] or 
                verify_attrs[3] != new_attrs[3]):
                os.close(fd)
                raise IOError("Serial port settings do not match the requested configuration")
        except Exception as e:
            os.close(fd)
            raise IOError(f"Failed to verify termios settings: {e}")
        
        # Flush the input and output buffers
        try:
            termios.tcflush(fd, termios.TCIOFLUSH)
        except Exception as e:
            os.close(fd)
            raise IOError(f"Failed to flush serial port: {e}")
        
        # Create a Serial object from the file descriptor
        try:
            # Close the fd since pyserial will open it again
            os.close(fd)
            
            # Create a pyserial object with the right settings
            port = serial.Serial(
                port=device,
                baudrate=BAUD_RATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=TIMEOUT_MS / 1000.0,  # Convert ms to seconds
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            
            # Make sure the port is opened
            if not port.is_open:
                port.open()
                
            # Additional settings that might be needed
            if hasattr(port, 'set_low_latency_mode'):
                port.set_low_latency_mode(True)
                
            return port
            
        except Exception as e:
            raise IOError(f"Failed to create serial port object: {e}")
        
    except Exception as e:
        # Catch-all for any other errors
        raise IOError(f"Failed to initialize serial port {device}: {e}")

def _open_serial_port_pyserial(device: str) -> serial.Serial:
    """
    Fallback method to initialize serial port using just pyserial.
    Used when low-level termios operations fail.
    
    Args:
        device (str): Path to the serial device
        
    Returns:
        serial.Serial: Configured serial port object
    """
    try:
        # Create a Serial object with the right settings
        port = serial.Serial(
            port=device,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT_MS / 1000.0,  # Convert ms to seconds
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        )
        
        # Make sure the port is opened
        if not port.is_open:
            port.open()
            
        # Flush buffers
        port.reset_input_buffer()
        port.reset_output_buffer()
        
        # Brief delay to ensure settings are applied
        time.sleep(0.1)
        
        # Try to set some additional low-level settings if available
        try:
            if hasattr(port, 'set_low_latency_mode'):
                port.set_low_latency_mode(True)
        except:
            pass
            
        return port
        
    except Exception as e:
        raise IOError(f"Failed to initialize serial port {device} with pyserial: {e}")

# This function tries both methods and returns the first one that works
def init_serial_port(device: str, use_low_level: bool = True) -> serial.Serial:
    """
    Open a serial port using either low-level termios or pyserial.
    
    Args:
        device: Path to the serial device
        use_low_level: Whether to try low-level termios first
        
    Returns:
        serial.Serial: Configured serial port object
    """
    if use_low_level:
        try:
            return open_serial_port(device)
        except Exception as e:
            logging.warning(f"Low-level serial port initialization failed: {e}")
            logging.info("Falling back to pyserial approach")
            return _open_serial_port_pyserial(device)
    else:
        return _open_serial_port_pyserial(device)

