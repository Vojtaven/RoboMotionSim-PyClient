import math
import struct
import time
from enum import IntEnum

import zmq

_DEG2RAD = math.pi / 180.0
_RAD2DEG = 180.0 / math.pi


class MsgType(IntEnum):
    HANDSHAKE = 0x01
    HANDSHAKE_ACK = 0x02
    HEARTBEAT = 0x03
    HEARTBEAT_ACK = 0x04
    MOTOR_COUNT = 0x05
    TELEMETRY = 0x06
    COMMAND = 0x07
    CMD_ACK = 0x08
    CMD_ERROR = 0x09
    CMD_COMPLETE = 0x0A
    CLEAR_COMMAND_QUEUE = 0x0B
    DISCONNECT_ACK = 0xFE
    DISCONNECT = 0xFF


class CommandType(IntEnum):
    MOVE_BY_DISTANCE_RAW = 0x01
    MOVE_BY_TIME_RAW = 0x02
    MOVE_AT_SPEED_RAW = 0x03
    STOP = 0x04
    MOVE_AT_SPEED_MOTORS = 0x05
    RUN_MOTOR_FOR_TIME = 0x06
    RUN_MOTOR_FOR_DISTANCE = 0x07
    STOP_MOTOR = 0x08
    START_MOTOR = 0x09
    MOVE_BY_TIME = 0x0A
    MOVE_BY_DISTANCE = 0x0B
    MOVE_AT_SPEED = 0x0C
    MOVE_BY_ANGLE = 0x0D
    MOVE_BY_ANGLE_RAW = 0x0E


class RobotConnectionError(Exception):
    pass


class RobotCommandError(Exception):
    pass


# Wire formats must match the simulator's packed C structs (little-endian)
_HEADER_FMT = "<IHB"       # id(u32) + payload_size(u16) + type(u8)
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)  # 7

_TELEMETRY_ODO_FMT = "<QfffffffB"  # timestamp + 7 floats + wheel_count
_TELEMETRY_ODO_SIZE = struct.calcsize(_TELEMETRY_ODO_FMT)  # 41

_TELEMETRY_WHEEL_FMT = "<ff"       # speed + distance per wheel
_TELEMETRY_WHEEL_SIZE = struct.calcsize(_TELEMETRY_WHEEL_FMT)  # 8

_HEARTBEAT_INTERVAL = 2.0  # seconds
_CONNECT_TIMEOUT = 500000  # ms


class Robot:
    def __init__(self, command_address: str, telemetry_address: str):
        self._command_address = command_address
        self._telemetry_address = telemetry_address

        self._ctx = zmq.Context()
        self._dealer: zmq.Socket | None = None
        self._sub: zmq.Socket | None = None

        self._msg_id = 0               # monotonic counter for all outgoing messages
        self._last_cmd_id = 0          # id of the most recent movement command
        self._completed_ids: set[int] = set()  # ids confirmed via CMD_COMPLETE
        self._motor_count = 0

        self._last_heartbeat_time = 0.0

        # Telemetry cache
        self._timestamp_ms: int = 0
        self._distance_x: float = 0.0
        self._distance_y: float = 0.0
        self._velocity_x: float = 0.0
        self._velocity_y: float = 0.0
        self._front_angle: float = 0.0
        self._chassis_angle: float = 0.0
        self._chassis_angular_velocity: float = 0.0
        self._wheel_speeds: list[float] = []
        self._wheel_distances: list[float] = []

    # ------------------------------------------------------------------ #
    #  Internal helpers
    # ------------------------------------------------------------------ #

    def _next_id(self) -> int:
        self._msg_id += 1
        return self._msg_id

    def _pack_header(self, msg_id: int, payload_size: int, msg_type: MsgType) -> bytes:
        return struct.pack(_HEADER_FMT, msg_id, payload_size, int(msg_type))

    def _send(self, msg_type: MsgType, payload: bytes = b"") -> int:
        msg_id = self._next_id()
        header = self._pack_header(msg_id, len(payload), msg_type)
        self._dealer.send(header + payload)
        return msg_id

    def _send_command(self, cmd_type: CommandType, params: bytes = b"") -> int:
        # Command payload = command type (u16) + command-specific params
        cmd_header = struct.pack("<H", int(cmd_type))
        payload = cmd_header + params
        msg_id = self._send(MsgType.COMMAND, payload)
        self._last_cmd_id = msg_id
        self._completed_ids.discard(msg_id)  # reset so is_move_finished() returns False
        return msg_id

    def _parse_header(self, data: bytes) -> tuple[int, int, MsgType]:
        msg_id, payload_size, msg_type = struct.unpack_from(_HEADER_FMT, data)
        return msg_id, payload_size, MsgType(msg_type)

    def _recv_dealer(self, timeout_ms: int = 0) -> tuple[int, MsgType, bytes] | None:
        if self._dealer.poll(timeout_ms):
            data = self._dealer.recv()
            if len(data) < _HEADER_SIZE:
                return None
            msg_id, payload_size, msg_type = self._parse_header(data)
            payload = data[_HEADER_SIZE:]
            return msg_id, msg_type, payload
        return None

    def _process_dealer_message(self, msg_id: int, msg_type: MsgType, payload: bytes):
        if msg_type == MsgType.CMD_COMPLETE:
            self._completed_ids.add(msg_id)
        elif msg_type == MsgType.CMD_ERROR:
            raise RobotCommandError(f"Command {msg_id} failed")
        elif msg_type == MsgType.MOTOR_COUNT and len(payload) >= 2:
            self._motor_count = struct.unpack_from("<H", payload)[0]

    def _poll_dealer(self):
        while True:  # drain all pending messages without blocking
            result = self._recv_dealer(0)
            if result is None:
                break
            self._process_dealer_message(*result)

    def _poll_telemetry(self):
        if self._sub.poll(0):
            data = self._sub.recv()
            if len(data) < _HEADER_SIZE + _TELEMETRY_ODO_SIZE:
                return
            payload = data[_HEADER_SIZE:]
            fields = struct.unpack_from(_TELEMETRY_ODO_FMT, payload)
            self._timestamp_ms = fields[0]
            self._distance_x = fields[1]
            self._distance_y = fields[2]
            self._velocity_x = fields[3]
            self._velocity_y = fields[4]
            self._front_angle = fields[5]
            self._chassis_angle = fields[6]
            self._chassis_angular_velocity = fields[7]
            wheel_count = fields[8]

            # Per-wheel data follows the odometry header as a flat array
            offset = _TELEMETRY_ODO_SIZE
            speeds = []
            distances = []
            for _ in range(wheel_count):
                if offset + _TELEMETRY_WHEEL_SIZE > len(payload):
                    break  # truncated message, keep what we have
                speed, dist = struct.unpack_from(_TELEMETRY_WHEEL_FMT, payload, offset)
                speeds.append(speed)
                distances.append(dist)
                offset += _TELEMETRY_WHEEL_SIZE
            self._wheel_speeds = speeds
            self._wheel_distances = distances

    def _send_heartbeat_if_due(self):
        now = time.monotonic()
        if now - self._last_heartbeat_time >= _HEARTBEAT_INTERVAL:
            self._send(MsgType.HEARTBEAT)
            self._last_heartbeat_time = now

    # ------------------------------------------------------------------ #
    #  Lifecycle
    # ------------------------------------------------------------------ #

    def connect(self) -> None:
        self._dealer = self._ctx.socket(zmq.DEALER)
        self._dealer.connect(self._command_address)

        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.CONFLATE, 1)   # only keep the latest telemetry frame
        self._sub.setsockopt(zmq.SUBSCRIBE, b"") # subscribe to all messages
        self._sub.connect(self._telemetry_address)

        self._send(MsgType.HANDSHAKE)

        result = self._recv_dealer(_CONNECT_TIMEOUT)
        if result is None:
            raise RobotConnectionError("Handshake timed out")
        msg_id, msg_type, payload = result
        if msg_type != MsgType.HANDSHAKE_ACK:
            raise RobotConnectionError(f"Expected HANDSHAKE_ACK, got {msg_type!r}")

        # Server sends MOTOR_COUNT once it knows the wheel count
        result = self._recv_dealer(_CONNECT_TIMEOUT)
        if result is not None:
            self._process_dealer_message(*result)

        self._last_heartbeat_time = time.monotonic()

    def disconnect(self) -> None:
        if self._dealer is None:
            return
        self._send(MsgType.DISCONNECT)
        # Brief wait for DISCONNECT_ACK (server may not send it)
        self._recv_dealer(1000)
        self._dealer.close()
        self._sub.close()
        self._dealer = None
        self._sub = None

    def run(self) -> None:
        self._poll_dealer()
        self._poll_telemetry()
        self._send_heartbeat_if_due()

    def run_to_position(self) -> None:
        while not self.is_move_finished():
            self.run()

    # ------------------------------------------------------------------ #
    #  Status
    # ------------------------------------------------------------------ #

    def is_move_finished(self) -> bool:
        return self._last_cmd_id in self._completed_ids

    # ------------------------------------------------------------------ #
    #  Telemetry
    # ------------------------------------------------------------------ #

    def get_distance_traveled_x(self) -> float:
        return self._distance_x

    def get_distance_traveled_y(self) -> float:
        return self._distance_y

    def get_velocity_x(self) -> float:
        return self._velocity_x

    def get_velocity_y(self) -> float:
        return self._velocity_y

    # Wire protocol sends angles in radians, public API exposes degrees
    def get_front_angle(self) -> float:
        return self._front_angle * _RAD2DEG

    def get_chassis_angle(self) -> float:
        return self._chassis_angle * _RAD2DEG

    def get_chassis_angular_velocity(self) -> float:
        return self._chassis_angular_velocity * _RAD2DEG

    def get_wheel_speed(self, wheel_index: int) -> float:
        return self._wheel_speeds[wheel_index]

    def get_wheel_distance(self, wheel_index: int) -> float:
        return self._wheel_distances[wheel_index]

    # ------------------------------------------------------------------ #
    #  Commands
    # ------------------------------------------------------------------ #

    # All movement methods convert deg -> rad before sending (wire protocol is radians)
    def move_by_distance_raw(self, distance_mm: float, x_speed: float, y_speed: float,
                             rotation_speed: float, front_rotation_speed: float) -> None:
        self._send_command(CommandType.MOVE_BY_DISTANCE_RAW,
                           struct.pack("<5f", distance_mm, x_speed, y_speed,
                                       rotation_speed * _DEG2RAD,
                                       front_rotation_speed * _DEG2RAD))

    def move_by_time_raw(self, time_s: float, x_speed: float, y_speed: float,
                         rotation_speed: float, front_rotation_speed: float) -> None:
        self._send_command(CommandType.MOVE_BY_TIME_RAW,
                           struct.pack("<5f", time_s, x_speed, y_speed,
                                       rotation_speed * _DEG2RAD,
                                       front_rotation_speed * _DEG2RAD))

    def move_at_speed_raw(self, x_speed: float, y_speed: float,
                          rotation_speed: float, front_rotation_speed: float) -> None:
        self._send_command(CommandType.MOVE_AT_SPEED_RAW,
                           struct.pack("<4f", x_speed, y_speed,
                                       rotation_speed * _DEG2RAD,
                                       front_rotation_speed * _DEG2RAD))

    def stop(self) -> None:
        self._send_command(CommandType.STOP)

    def move_at_speed_motors(self, speeds: list[float]) -> None:
        count = len(speeds)
        params = struct.pack("<H", count)
        for s in speeds:
            params += struct.pack("<f", s)
        self._send_command(CommandType.MOVE_AT_SPEED_MOTORS, params)

    def run_motor_for_time(self, motor_id: int, speed: float, time_s: float) -> None:
        self._send_command(CommandType.RUN_MOTOR_FOR_TIME,
                           struct.pack("<Hff", motor_id, speed, time_s))

    def run_motor_for_distance(self, motor_id: int, speed: float, distance_mm: float) -> None:
        self._send_command(CommandType.RUN_MOTOR_FOR_DISTANCE,
                           struct.pack("<Hff", motor_id, speed, distance_mm))

    def stop_motor(self, motor_id: int) -> None:
        self._send_command(CommandType.STOP_MOTOR,
                           struct.pack("<H", motor_id))

    def start_motor(self, motor_id: int, speed: float) -> None:
        self._send_command(CommandType.START_MOTOR,
                           struct.pack("<Hf", motor_id, speed))

    def move_by_time(self, time_s: float, x_speed: float, y_speed: float,
                     rotation_speed: float, center_x_mm: float, center_y_mm: float,
                     rotate_chassis: bool) -> None:
        self._send_command(CommandType.MOVE_BY_TIME,
                           struct.pack("<6f?", time_s, x_speed, y_speed,
                                       rotation_speed * _DEG2RAD,
                                       center_x_mm, center_y_mm,
                                       rotate_chassis))

    def move_by_distance(self, distance_mm: float, x_speed: float, y_speed: float,
                         rotation_speed: float, center_x_mm: float, center_y_mm: float,
                         rotate_chassis: bool) -> None:
        self._send_command(CommandType.MOVE_BY_DISTANCE,
                           struct.pack("<6f?", distance_mm, x_speed, y_speed,
                                       rotation_speed * _DEG2RAD,
                                       center_x_mm, center_y_mm,
                                       rotate_chassis))

    def move_at_speed(self, x_speed: float, y_speed: float, rotation_speed: float,
                      center_x_mm: float, center_y_mm: float,
                      rotate_chassis: bool) -> None:
        self._send_command(CommandType.MOVE_AT_SPEED,
                           struct.pack("<5f?", x_speed, y_speed,
                                       rotation_speed * _DEG2RAD,
                                       center_x_mm, center_y_mm, rotate_chassis))

    def move_by_angle(self, x_speed: float, y_speed: float, angle_deg: float,
                      rotation_speed: float, center_x_mm: float, center_y_mm: float,
                      rotate_chassis: bool) -> None:
        self._send_command(CommandType.MOVE_BY_ANGLE,
                           struct.pack("<6f?", x_speed, y_speed,
                                       angle_deg * _DEG2RAD,
                                       rotation_speed * _DEG2RAD,
                                       center_x_mm, center_y_mm,
                                       rotate_chassis))

    def move_by_angle_raw(self, angle_deg: float, x_speed: float, y_speed: float,
                          rotation_speed: float, front_rotation_speed: float) -> None:
        self._send_command(CommandType.MOVE_BY_ANGLE_RAW,
                           struct.pack("<5f", angle_deg * _DEG2RAD, x_speed, y_speed,
                                       rotation_speed * _DEG2RAD,
                                       front_rotation_speed * _DEG2RAD))

    def clear_command_queue(self) -> None:
        self._send(MsgType.CLEAR_COMMAND_QUEUE)
