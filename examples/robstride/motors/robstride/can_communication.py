import logging
import struct
import time
import threading
from enum import Enum, IntEnum
from typing import Dict, List, Optional, Set, Tuple, Union
import serial
from dataclasses import dataclass

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Constants
CAN_ID_MASTER = 0x00
CAN_ID_MOTOR_DEFAULT = 0x7F
CAN_ID_BROADCAST = 0xFE
CAN_ID_DEBUG_UI = 0xFD

BAUDRATE = 921600  # B921600 in nix terminology


class MotorType(Enum):
    Type01 = "01"
    Type02 = "02"
    Type03 = "03"
    Type04 = "04"


@dataclass
class MotorConfig:
    p_min: float
    p_max: float
    v_min: float
    v_max: float
    kp_min: float
    kp_max: float
    kd_min: float
    kd_max: float
    t_min: float
    t_max: float
    zero_on_init: bool
    can_timeout_command: int
    can_timeout_factor: float


# Static configuration for motor types
ROBSTRIDE_CONFIGS = {
    MotorType.Type01: MotorConfig(
        p_min=-12.5, # 회전 범위
        p_max=12.5,
        v_min=-44.0, # 속도 범위
        v_max=44.0,
        kp_min=0.0, # 피드백 제어 파라미터
        kp_max=500.0,
        kd_min=0.0,
        kd_max=5.0,
        t_min=-12.0,
        t_max=12.0,
        zero_on_init=False,  # Single encoder motor
        can_timeout_command=0x200c,
        can_timeout_factor=12000.0,
    ),
    MotorType.Type02: MotorConfig(
        p_min=-12.5,
        p_max=12.5,
        v_min=-44.0,
        v_max=44.0,
        kp_min=0.0,
        kp_max=500.0,
        kd_min=0.0,
        kd_max=5.0,
        t_min=-12.0,
        t_max=12.0,
        zero_on_init=False,
        can_timeout_command=0x200b,
        can_timeout_factor=12000.0,
    ),
    MotorType.Type03: MotorConfig(
        p_min=-12.5,
        p_max=12.5,
        v_min=-20.0,
        v_max=20.0,
        kp_min=0.0,
        kp_max=5000.0,
        kd_min=0.0,
        kd_max=100.0,
        t_min=-60.0,
        t_max=60.0,
        zero_on_init=False,
        can_timeout_command=0x200b,
        can_timeout_factor=6000.0,
    ),
    MotorType.Type04: MotorConfig(
        p_min=-12.5,
        p_max=12.5,
        v_min=-15.0,
        v_max=15.0,
        kp_min=0.0,
        kp_max=5000.0,
        kd_min=0.0,
        kd_max=100.0,
        t_min=-120.0,
        t_max=120.0,
        zero_on_init=False,
        can_timeout_command=0x200b,
        can_timeout_factor=12000.0,
    ),
}


class CanComMode(IntEnum):
    AnnounceDevId = 0
    MotorCtrl = 1
    MotorFeedback = 2
    MotorIn = 3
    MotorReset = 4
    MotorCali = 5
    MotorZero = 6
    MotorId = 7
    ParaWrite = 8
    ParaRead = 9
    ParaUpdate = 10
    OtaStart = 11
    OtaInfo = 12
    OtaIng = 13
    OtaEnd = 14
    CaliIng = 15
    CaliRst = 16
    SdoRead = 17
    SdoWrite = 18
    ParaStrInfo = 19
    MotorBrake = 20
    FaultWarn = 21
    ModeTotal = 22


class MotorMode(IntEnum):
    Reset = 0
    Cali = 1
    Motor = 2
    Brake = 3


class RunMode(IntEnum):
    UnsetMode = -1
    MitMode = 0
    PositionMode = 1
    SpeedMode = 2
    CurrentMode = 3
    ToZeroMode = 4
    CspPositionMode = 5


class ExId:
    def __init__(self, id: int, data: int, mode: CanComMode, res: int = 0):
        self.id = id
        self.data = data
        self.mode = mode
        self.res = res


class CanPack:
    def __init__(self, ex_id: ExId, len: int, data: List[int]):
        self.ex_id = ex_id
        self.len = len
        self.data = data


class MotorFeedback:
    def __init__(
        self,
        can_id: int = 0,
        position: float = 0.0,
        velocity: float = 0.0,
        torque: float = 0.0,
        mode: MotorMode = MotorMode.Reset,
        faults: int = 0,
    ):
        self.can_id = can_id
        self.position = position
        self.velocity = velocity
        self.torque = torque
        self.mode = mode
        self.faults = faults


class MotorFeedbackRaw:
    def __init__(
        self,
        can_id: int = 0,
        pos_int: int = 0,
        vel_int: int = 0,
        torque_int: int = 0,
        mode: MotorMode = MotorMode.Reset,
        faults: int = 0,
    ):
        self.can_id = can_id
        self.pos_int = pos_int
        self.vel_int = vel_int
        self.torque_int = torque_int
        self.mode = mode
        self.faults = faults


@dataclass
class MotorControlParams:
    position: float = 0.0
    velocity: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    torque: float = 0.0


def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    span = x_max - x_min
    offset = x_min
    return (x_int * span) / ((1 << bits) - 1) + offset


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    span = x_max - x_min
    offset = x_min
    return int((x - offset) * ((1 << bits) - 1) / span)


def pack_bits(values: List[int], bit_lengths: List[int]) -> int:
    result = 0
    current_shift = 0

    for value, bits in zip(values, bit_lengths):
        mask = (1 << bits) - 1
        result |= (value & mask) << current_shift
        current_shift += bits

    return result


def unpack_bits(value: int, bit_lengths: List[int]) -> List[int]:
    result = []
    current_value = value

    for bits in bit_lengths:
        mask = (1 << bits) - 1
        result.append(current_value & mask)
        current_value >>= bits

    return result


def pack_ex_id(ex_id: ExId) -> bytes:
    addr = (
        pack_bits(
            [ex_id.id, ex_id.data, int(ex_id.mode), ex_id.res],
            [8, 16, 5, 3],
        )
        << 3
    ) | 0x00000004
    return struct.pack(">I", addr)


def unpack_ex_id(addr: bytes) -> ExId:
    addr_int = struct.unpack(">I", addr)[0]
    addr_values = unpack_bits(addr_int >> 3, [8, 16, 5, 3])
    return ExId(
        id=addr_values[0],
        data=addr_values[1],
        mode=CanComMode(addr_values[2]),
        res=addr_values[3],
    )


def init_serial_port(port_name: str) -> serial.Serial:
    """Initialize the serial port with proper settings."""
    try:
        port = serial.Serial(
            port=port_name,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,  # Read timeout
            write_timeout=0.1,  # Write timeout
        )
        return port
    except serial.SerialException as e:
        raise IOError(f"Failed to open serial port {port_name}: {e}")


def motor_type_from_str(s: str) -> MotorType:
    try:
        return MotorType(s)
    except ValueError:
        raise ValueError(f"Invalid motor type: {s}")


def tx_packs(port: serial.Serial, packs: List[CanPack], verbose: bool = False) -> None:
    buffer = bytearray()

    for pack in packs:
        buffer.extend(b"AT")
        buffer.extend(pack_ex_id(pack.ex_id))
        buffer.append(pack.len)
        buffer.extend(pack.data[:pack.len])
        buffer.extend(b"\r\n")

    if not buffer:
        raise IOError("Empty buffer")

    if verbose:
        print(f"TX: {' '.join([f'{b:02X}' for b in buffer])}")

    port.write(buffer)
    port.flush()


def rx_unpack(port: serial.Serial, expected_count: int, verbose: bool = False) -> List[CanPack]:
    packs = []
    max_attempts = expected_count * 2  # Set a maximum number of attempts
    attempt = 0

    while len(packs) < expected_count and attempt < max_attempts:
        attempt += 1
        try:
            # First read the AT header
            header = port.read(2)
            if not header or header != b'AT':
                continue

            # Read the rest of the packet (4 bytes for ID, 1 for length, up to 8 for data, 2 for \r\n)
            ex_id_bytes = port.read(4)
            if len(ex_id_bytes) != 4:
                continue

            len_byte = port.read(1)
            if not len_byte:
                continue
            data_len = len_byte[0]

            data = port.read(data_len)
            if len(data) != data_len:
                continue

            # Read the trailing \r\n
            ending = port.read(2)
            if ending != b'\r\n':
                continue

            # Reassemble the full buffer for verbose output
            if verbose:
                full_buffer = bytearray(header + ex_id_bytes + len_byte + data + ending)
                print(f"RX: {' '.join([f'{b:02X}' for b in full_buffer])}")

            ex_id = unpack_ex_id(ex_id_bytes)
            packs.append(CanPack(ex_id=ex_id, len=data_len, data=list(data)))

        except serial.SerialTimeoutException:
            break

    port.reset_input_buffer()  # Clear any remaining data
    return packs


def unpack_raw_feedback(pack: CanPack) -> MotorFeedbackRaw:
    can_id = pack.ex_id.data & 0x00FF
    faults = (pack.ex_id.data & 0x3F00) >> 8
    mode = MotorMode((pack.ex_id.data & 0xC000) >> 14)

    if pack.ex_id.mode != CanComMode.MotorFeedback:
        return MotorFeedbackRaw(can_id=can_id, mode=mode, faults=faults)

    pos_int = (pack.data[0] << 8) | pack.data[1]
    vel_int = (pack.data[2] << 8) | pack.data[3]
    torque_int = (pack.data[4] << 8) | pack.data[5]

    return MotorFeedbackRaw(
        can_id=can_id,
        pos_int=pos_int,
        vel_int=vel_int,
        torque_int=torque_int,
        mode=mode,
        faults=faults,
    )


class Motors:
    def __init__(
        self, port_name: str, motor_infos: Dict[int, MotorType], verbose: bool = False
    ):
        if not motor_infos:
            raise ValueError("No motors to initialize")

        self.port = init_serial_port(port_name)
        self.motor_configs = {
            id: ROBSTRIDE_CONFIGS[motor_type] for id, motor_type in motor_infos.items()
        }
        self.mode = RunMode.MitMode
        self.sleep_time = 0.02  # 20ms
        self.verbose = verbose

    def send_command(self, pack: CanPack, sleep_after: bool = True) -> CanPack:
        tx_packs(self.port, [pack], self.verbose)
        if sleep_after:
            time.sleep(self.sleep_time)
        packs = rx_unpack(self.port, 1, self.verbose)
        if not packs:
            raise IOError("Failed to receive CAN packet")
        if len(packs) > 1:
            logger.warning("Received multiple CAN packets, using first one")
        return packs[0]

    def send_commands(
        self, packs: List[CanPack], sleep_after: bool = True, serial: bool = True
    ) -> List[CanPack]:
        if not packs:
            raise ValueError("No commands to send!")

        if serial:
            results = []
            for pack in packs:
                results.append(self.send_command(pack, sleep_after))
            return results
        else:
            tx_packs(self.port, packs, self.verbose)
            if sleep_after:
                time.sleep(self.sleep_time)
            return rx_unpack(self.port, len(packs), self.verbose)

    def send_get_mode(self) -> Dict[int, RunMode]:
        motor_ids = list(self.motor_configs.keys())
        modes = {}

        for id in motor_ids:
            pack = CanPack(
                ex_id=ExId(
                    id=id,
                    data=CAN_ID_DEBUG_UI,
                    mode=CanComMode.SdoRead,
                    res=0,
                ),
                len=8,
                data=[0] * 8,
            )

            index = 0x7005
            pack.data[0] = index & 0xFF
            pack.data[1] = (index >> 8) & 0xFF

            try:
                response = self.send_command(pack, False)
                mode = RunMode(response.data[4])
                modes[id] = mode
            except Exception as err:
                logger.error(f"Failed to get mode for motor {id}: {err}")

        return modes

    def send_set_mode(self, mode: RunMode) -> Dict[int, MotorFeedback]:
        if self.mode == RunMode.UnsetMode:
            read_mode = self.send_get_mode()
            if not read_mode:
                raise RuntimeError("Failed to get the current mode")

            # Check if all motors are in the same mode
            modes = list(read_mode.values())
            if all(m == modes[0] for m in modes):
                self.mode = modes[0]

        if self.mode == mode:
            return {}

        self.mode = mode
        motor_ids = list(self.motor_configs.keys())
        feedbacks = {}

        for id in motor_ids:
            pack = CanPack(
                ex_id=ExId(
                    id=id,
                    data=CAN_ID_DEBUG_UI,
                    mode=CanComMode.SdoWrite,
                    res=0,
                ),
                len=8,
                data=[0] * 8,
            )

            index = 0x7005
            pack.data[0] = index & 0xFF
            pack.data[1] = (index >> 8) & 0xFF
            pack.data[4] = int(mode)

            try:
                response_pack = self.send_command(pack, True)
                feedback = self.unpack_feedback(response_pack)
                feedbacks[id] = feedback
            except Exception as e:
                logger.error(f"Failed to set mode for motor {id}: {e}")
                continue

        return feedbacks

    def send_set_zero(self, motor_id: int) -> None:
        pack = CanPack(
            ex_id=ExId(
                id=motor_id,
                data=CAN_ID_DEBUG_UI,
                mode=CanComMode.MotorZero,
                res=0,
            ),
            len=8,
            data=[1, 0, 0, 0, 0, 0, 0, 0],
        )

        self.send_command(pack, True)

    def send_set_zeros(self, motor_ids: Optional[List[int]] = None) -> None:
        ids_to_zero = motor_ids if motor_ids is not None else list(self.motor_configs.keys())

        for id in ids_to_zero:
            if id not in self.motor_configs:
                raise ValueError(f"Invalid motor ID: {id}")

        # Zero each motor
        for id in ids_to_zero:
            self.send_set_zero(id)

    def zero_motors(self, motor_ids: List[int]) -> None:
        self.send_zero_torque(motor_ids)
        self.send_set_zeros(motor_ids)

    def read_u16_param(self, motor_id: int, index: int) -> int:
        pack = CanPack(
            ex_id=ExId(
                id=motor_id,
                data=CAN_ID_DEBUG_UI,
                mode=CanComMode.ParaRead,
                res=0,
            ),
            len=8,
            data=[0] * 8,
        )

        pack.data[0] = index & 0xFF
        pack.data[1] = (index >> 8) & 0xFF
        tx_packs(self.port, [pack], self.verbose)

        packs = rx_unpack(self.port, 1, self.verbose)
        if not packs:
            raise IOError("Failed to receive CAN packet")
        response = packs[0]

        return response.data[4] | (response.data[5] << 8)

    def read_can_timeout(self, id: int) -> float:
        if id not in self.motor_configs:
            raise ValueError(f"Motor ID {id} not found")

        config = self.motor_configs[id]
        timeout = self.read_u16_param(id, config.can_timeout_command)
        # Convert to milliseconds
        return timeout * 1000.0 / config.can_timeout_factor

    def read_can_timeouts(self) -> Dict[int, float]:
        timeouts = {}
        for id, config in self.motor_configs.items():
            timeout = self.read_u16_param(id, config.can_timeout_command)
            timeouts[id] = timeout * 1000.0 / config.can_timeout_factor
        return timeouts

    def send_can_timeout(self, timeout: float) -> None:
        # NOTE: This function may not work reliably (as noted in original code)
        for id, config in self.motor_configs.items():
            pack = CanPack(
                ex_id=ExId(
                    id=id,
                    data=CAN_ID_DEBUG_UI,
                    mode=CanComMode.ParaWrite,
                    res=0,
                ),
                len=8,
                data=[0] * 8,
            )

            index = config.can_timeout_command
            pack.data[0] = index & 0xFF
            pack.data[1] = (index >> 8) & 0xFF
            pack.data[2] = 0x04

            new_timeout = int(
                min(max(timeout * config.can_timeout_factor / 1000.0, 0), 100000)
            )
            logger.info(f"Setting CAN timeout to {new_timeout} for motor {id}")
            
            # Convert new_timeout to little-endian bytes
            pack.data[4] = new_timeout & 0xFF
            pack.data[5] = (new_timeout >> 8) & 0xFF
            pack.data[6] = (new_timeout >> 16) & 0xFF
            pack.data[7] = (new_timeout >> 24) & 0xFF

            try:
                self.send_command(pack, True)
                logger.info(f"CAN timeout set to {new_timeout}")
            except Exception as e:
                logger.error(f"Failed to set CAN timeout for motor {id}: {e}")

    def send_reset(self, id: int) -> CanPack:
        pack = CanPack(
            ex_id=ExId(
                id=id,
                data=CAN_ID_DEBUG_UI,
                mode=CanComMode.MotorReset,
                res=0,
            ),
            len=8,
            data=[0] * 8,
        )

        return self.send_command(pack, True)

    def send_resets(self) -> None:
        motor_ids = list(self.motor_configs.keys())
        for id in motor_ids:
            self.send_reset(id)

    def send_start(self, id: int) -> CanPack:
        pack = CanPack(
            ex_id=ExId(
                id=id,
                data=CAN_ID_DEBUG_UI,
                mode=CanComMode.MotorIn,
                res=0,
            ),
            len=8,
            data=[0] * 8,
        )

        return self.send_command(pack, True)

    def send_starts(self) -> None:
        motor_ids = list(self.motor_configs.keys())
        for id in motor_ids:
            self.send_start(id)

    def pack_motor_params(self, id: int, params: MotorControlParams) -> CanPack:
        if id not in self.motor_configs:
            raise ValueError(f"Motor ID {id} not found")

        config = self.motor_configs[id]
        pack = CanPack(
            ex_id=ExId(
                id=id,
                data=0,
                mode=CanComMode.MotorCtrl,
                res=0,
            ),
            len=8,
            data=[0] * 8,
        )

        pos_int_set = float_to_uint(params.position, config.p_min, config.p_max, 16)
        vel_int_set = float_to_uint(params.velocity, config.v_min, config.v_max, 16)
        kp_int_set = float_to_uint(params.kp, config.kp_min, config.kp_max, 16)
        kd_int_set = float_to_uint(params.kd, config.kd_min, config.kd_max, 16)
        torque_int_set = float_to_uint(params.torque, config.t_min, config.t_max, 16)

        pack.ex_id.data = torque_int_set

        # Convert to big-endian bytes
        pack.data[0] = (pos_int_set >> 8) & 0xFF
        pack.data[1] = pos_int_set & 0xFF
        pack.data[2] = (vel_int_set >> 8) & 0xFF
        pack.data[3] = vel_int_set & 0xFF
        pack.data[4] = (kp_int_set >> 8) & 0xFF
        pack.data[5] = kp_int_set & 0xFF
        pack.data[6] = (kd_int_set >> 8) & 0xFF
        pack.data[7] = kd_int_set & 0xFF

        return pack

    def send_motor_controls(
        self, params_map: Dict[int, MotorControlParams], serial: bool = True
    ) -> Dict[int, MotorFeedback]:
        self.send_set_mode(RunMode.MitMode)

        packs = []
        for id, params in params_map.items():
            try:
                pack = self.pack_motor_params(id, params)
                packs.append(pack)
            except ValueError:
                logger.warning(f"Skipping invalid motor ID: {id}")

        if not packs:
            raise ValueError("No valid motor control parameters provided")

        if serial:
            response_packs = []
            for pack in packs:
                try:
                    response = self.send_command(pack, False)
                    response_packs.append(response)
                except Exception as e:
                    logger.error(f"Command failed for motor {pack.ex_id.id}: {e}")
        else:
            response_packs = self.send_commands(packs, False, False)

        feedbacks = {}
        for pack in response_packs:
            try:
                feedback = self.unpack_feedback(pack)
                feedbacks[feedback.can_id] = feedback
            except Exception as e:
                logger.error(f"Failed to unpack feedback: {e}")

        return feedbacks

    def send_zero_torque(self, motor_ids: List[int]) -> None:
        params = {id: MotorControlParams() for id in motor_ids}
        self.send_motor_controls(params, True)

    def unpack_feedback(self, pack: CanPack) -> MotorFeedback:
        raw_feedback = unpack_raw_feedback(pack)
        can_id = raw_feedback.can_id

        if can_id not in self.motor_configs:
            raise ValueError(f"Motor ID {can_id} not found")

        config = self.motor_configs[can_id]
        position = uint_to_float(raw_feedback.pos_int, config.p_min, config.p_max, 16)
        velocity = uint_to_float(raw_feedback.vel_int, config.v_min, config.v_max, 16)
        torque = uint_to_float(raw_feedback.torque_int, config.t_min, config.t_max, 16)

        return MotorFeedback(
            can_id=can_id,
            position=position,
            velocity=velocity,
            torque=torque,
            mode=raw_feedback.mode,
            faults=raw_feedback.faults,
        )


class MotorsSupervisor:
    def __init__(
        self,
        port_name: str,
        motor_infos: Dict[int, MotorType],
        verbose: bool = False,
        max_update_rate: float = 50.0,
        zero_on_init: bool = False,
    ):
        # Initialize Motors
        self.motors = Motors(port_name, motor_infos, verbose)
        
        # Thread synchronization objects
        self.motors_lock = threading.Lock()
        self.target_params_lock = threading.RLock()
        self.running_lock = threading.RLock()
        self.feedback_lock = threading.RLock()
        self.zero_motors_lock = threading.Lock()
        self.paused_lock = threading.RLock()
        self.restart_lock = threading.Lock()
        self.counters_lock = threading.RLock()
        self.rate_lock = threading.RLock()
        self.serial_lock = threading.RLock()
        
        # State variables
        self.target_params = {
            id: MotorControlParams() for id in motor_infos.keys()
        }
        self.running = True
        self.latest_feedback = {}
        self.motors_to_zero = {
            id for id, config in self.motors.motor_configs.items()
            if config.zero_on_init or zero_on_init
        }
        self.paused = False
        self.restart = False
        self.total_commands = 0
        self.failed_commands = {id: 0 for id in motor_infos.keys()}
        self.max_update_rate = max_update_rate
        self.actual_update_rate = 0.0
        self.serial = True
        
        # Start the control thread
        self.start_control_thread()

    def start_control_thread(self):
        control_thread = threading.Thread(target=self._control_loop)
        control_thread.daemon = True  # Thread will exit when main program exits
        control_thread.start()

    def _control_loop(self):
        with self.motors_lock:
            # Run pre-flight checks
            try:
                self.motors.send_resets()
                self.motors.send_starts()
                logger.info("Pre-flight checks completed successfully")
                self.motors.send_set_mode(RunMode.MitMode)
            except Exception as e:
                logger.error(f"Pre-flight checks failed: {e}")
                with self.running_lock:
                    self.running = False
                return

        last_update_time = time.time()

        while True:
            # Check if we should continue running
            with self.running_lock:
                if not self.running:
                    break

            # Check if paused
            with self.paused_lock:
                if self.paused:
                    time.sleep(0.01)  # Short sleep while paused
                    continue

            # Check if restart is requested
            with self.restart_lock:
                if self.restart:
                    self.restart = False
                    with self.motors_lock:
                        try:
                            self.motors.send_resets()
                            self.motors.send_starts()
                        except Exception as e:
                            logger.error(f"Restart failed: {e}")

            loop_start_time = time.time()

            # Send zero torque commands to motors that need to be zeroed
            with self.zero_motors_lock:
                if self.motors_to_zero:
                    motor_ids = list(self.motors_to_zero)
                    try:
                        with self.motors_lock:
                            self.motors.zero_motors(motor_ids)
                        self.motors_to_zero.clear()
                    except Exception as e:
                        logger.error(f"Failed to zero motors: {e}")

            # Send control commands to motors
            with self.target_params_lock:
                target_params = self.target_params.copy()

            if target_params:
                try:
                    with self.serial_lock:
                        serial_mode = self.serial
                    
                    with self.motors_lock:
                        feedbacks = self.motors.send_motor_controls(target_params, serial_mode)
                    
                    with self.feedback_lock:
                        self.latest_feedback.update(feedbacks)
                    
                    with self.counters_lock:
                        self.total_commands += 1
                        # Update failed commands counter
                        for motor_id in target_params:
                            if motor_id not in feedbacks:
                                self.failed_commands[motor_id] = self.failed_commands.get(motor_id, 0) + 1
                
                except Exception as e:
                    logger.error(f"Failed to send motor controls: {e}")

            # Calculate actual update rate (EWMA)
            elapsed = time.time() - last_update_time
            last_update_time = loop_start_time
            current_rate = 1.0 / elapsed if elapsed > 0 else 0
            
            with self.rate_lock:
                self.actual_update_rate = self.actual_update_rate * 0.9 + current_rate * 0.1

            # Sleep to maintain desired update rate
            with self.rate_lock:
                target_duration = 1.0 / self.max_update_rate
            
            elapsed = time.time() - loop_start_time
            if target_duration > elapsed:
                time.sleep(target_duration - elapsed)
            else:
                # Yield to other threads but don't sleep for too long
                time.sleep(0.001)

        # Cleanup when loop exits
        logger.info("Control loop stopping, sending zero torque commands")
        try:
            motor_ids = list(self.motors.motor_configs.keys())
            with self.motors_lock:
                self.motors.send_zero_torque(motor_ids)
                self.motors.send_resets()
        except Exception as e:
            logger.error(f"Cleanup failed: {e}")

    def get_total_commands(self) -> int:
        with self.counters_lock:
            return self.total_commands

    def get_failed_commands(self, motor_id: int) -> int:
        with self.counters_lock:
            if motor_id not in self.failed_commands:
                raise ValueError(f"Motor ID {motor_id} not found")
            return self.failed_commands[motor_id]

    def reset_command_counters(self) -> None:
        with self.counters_lock:
            self.total_commands = 0
            self.failed_commands = {id: 0 for id in self.failed_commands.keys()}

    def set_params(self, motor_id: int, params: MotorControlParams) -> None:
        with self.target_params_lock:
            self.target_params[motor_id] = params

    def set_position(self, motor_id: int, position: float) -> float:
        with self.target_params_lock:
            if motor_id not in self.target_params:
                raise ValueError(f"Motor ID {motor_id} not found")
            params = self.target_params[motor_id]
            params.position = position
            return params.position

    def get_position(self, motor_id: int) -> float:
        with self.target_params_lock:
            if motor_id not in self.target_params:
                raise ValueError(f"Motor ID {motor_id} not found")
            return self.target_params[motor_id].position

    def set_velocity(self, motor_id: int, velocity: float) -> float:
        with self.target_params_lock:
            if motor_id not in self.target_params:
                raise ValueError(f"Motor ID {motor_id} not found")
            params = self.target_params[motor_id]
            params.velocity = velocity
            return params.velocity

    def get_velocity(self, motor_id: int) -> float:
        with self.target_params_lock:
            if motor_id not in self.target_params:
                raise ValueError(f"Motor ID {motor_id} not found")
            return self.target_params[motor_id].velocity

    def set_kp(self, motor_id: int, kp: float) -> float:
        with self.target_params_lock:
            if motor_id not in self.target_params:
                raise ValueError(f"Motor ID {motor_id} not found")
            params = self.target_params[motor_id]
            params.kp = max(0.0, kp)  # Clamp to non-negative
            return params.kp

    def get_kp(self, motor_id: int) -> float:
        with self.target_params_lock:
            if motor_id not in self.target_params:
                raise ValueError(f"Motor ID {motor_id} not found")
            return self.target_params[motor_id].kp

    def set_kd(self, motor_id: int, kd: float) -> float:
        with self.target_params_lock:
            if motor_id not in self.target_params:
                raise ValueError(f"Motor ID {motor_id} not found")
            params = self.target_params[motor_id]
            params.kd = max(0.0, kd)  # Clamp to non-negative
            return params.kd

    def get_kd(self, motor_id: int) -> float:
        with self.target_params_lock:
            if motor_id not in self.target_params:
                raise ValueError(f"Motor ID {motor_id} not found")
            return self.target_params[motor_id].kd

    def set_torque(self, motor_id: int, torque: float) -> float:
        with self.target_params_lock:
            if motor_id not in self.target_params:
                raise ValueError(f"Motor ID {motor_id} not found")
            params = self.target_params[motor_id]
            params.torque = torque
            return params.torque

    def get_torque(self, motor_id: int) -> float:
        with self.target_params_lock:
            if motor_id not in self.target_params:
                raise ValueError(f"Motor ID {motor_id} not found")
            return self.target_params[motor_id].torque

    def add_motor_to_zero(self, motor_id: int) -> None:
        # Set parameters to zero first
        with self.target_params_lock:
            if motor_id not in self.target_params:
                raise ValueError(f"Motor ID {motor_id} not found")
            params = self.target_params[motor_id]
            params.torque = 0.0
            params.position = 0.0
            params.velocity = 0.0
        
        # Add to zeroing set
        with self.zero_motors_lock:
            self.motors_to_zero.add(motor_id)

    def get_latest_feedback(self) -> Dict[int, MotorFeedback]:
        with self.feedback_lock:
            return self.latest_feedback.copy()

    def toggle_pause(self) -> None:
        with self.paused_lock:
            self.paused = not self.paused

    def reset(self) -> None:
        with self.restart_lock:
            self.restart = True

    def stop(self) -> None:
        with self.running_lock:
            self.running = False
        time.sleep(0.2)  # Give time for the control thread to clean up

    def is_running(self) -> bool:
        with self.running_lock:
            return self.running

    def set_max_update_rate(self, rate: float) -> None:
        with self.rate_lock:
            self.max_update_rate = rate

    def get_actual_update_rate(self) -> float:
        with self.rate_lock:
            return self.actual_update_rate

    def get_serial(self) -> bool:
        with self.serial_lock:
            return self.serial

    def toggle_serial(self) -> bool:
        with self.serial_lock:
            self.serial = not self.serial
            return self.serial

    def __del__(self):
        """Cleanup when object is deleted"""
        self.stop()