import zmq
import struct
import time
import threading
from enum import IntEnum
from prompt_toolkit import PromptSession
from prompt_toolkit.patch_stdout import patch_stdout

# --- Protocol Definitions ---
class MsgType(IntEnum):
    HANDSHAKE           = 0x01
    HANDSHAKE_ACK       = 0x02
    HEARTBEAT           = 0x03
    HEARTBEAT_ACK       = 0x04
    MOTOR_COUNT         = 0x05
    TELEMETRY           = 0x06
    COMMAND             = 0x07
    CMD_ACK             = 0x08
    CMD_ERROR           = 0x09
    CMD_COMPLETE        = 0x0A
    CLEAR_COMMAND_QUEUE = 0x0B
    DISCONNECT_ACK      = 0xFE
    DISCONNECT          = 0xFF

class CommandType(IntEnum):
    MOVE_BY_DISTANCE_RAW  = 0x01
    MOVE_BY_TIME_RAW      = 0x02
    MOVE_AT_SPEED_RAW     = 0x03
    STOP                  = 0x04
    MOVE_AT_SPEED_MOTORS  = 0x05
    RUN_MOTOR_FOR_TIME    = 0x06
    RUN_MOTOR_FOR_DISTANCE= 0x07
    STOP_MOTOR            = 0x08
    START_MOTOR           = 0x09
    MOVE_BY_TIME          = 0x0A
    MOVE_BY_DISTANCE      = 0x0B
    MOVE_AT_SPEED         = 0x0C
    MOVE_BY_ANGLE         = 0x0D
    MOVE_BY_ANGLE_RAW     = 0x0E

# Struct Formats (Little Endian, Packed)
HEADER_FMT = "<IHB"  # id(4), payload_size(2), type(1)
HEADER_SIZE = struct.calcsize(HEADER_FMT)


class RobotController:
    def __init__(self, ip="127.0.0.1"):
        self.context = zmq.Context()
        self.msg_id_counter = 1
        self.running = True

        # --- Socket Setup ---
        # PUB -> Slave's SUB (commands out)
        self.pub_cmd = self.context.socket(zmq.PUB)
        self.pub_cmd.connect(f"tcp://{ip}:5555")

        # SUB <- Slave's PUB (responses: ACKs, errors, etc.)
        self.sub_resp = self.context.socket(zmq.SUB)
        self.sub_resp.connect(f"tcp://{ip}:5556")
        self.sub_resp.setsockopt_string(zmq.SUBSCRIBE, "")

        # SUB <- Slave's PUB (high-freq telemetry)
        self.sub_telemetry = self.context.socket(zmq.SUB)
        self.sub_telemetry.connect(f"tcp://{ip}:5557")
        self.sub_telemetry.setsockopt_string(zmq.SUBSCRIBE, "")

        # --- Background Threads ---
        self.listener_thread  = threading.Thread(target=self._listen_loop,    daemon=True)
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)

    # ------------------------------------------------------------------ #
    #  Lifecycle                                                           #
    # ------------------------------------------------------------------ #

    def start(self):
        self.listener_thread.start()
        self.heartbeat_thread.start()
        print("[SYSTEM] Connected. Background threads running.")

    def close(self):
        self.running = False
        self._send_packet(MsgType.DISCONNECT)
        time.sleep(0.1)
        self.context.destroy()
        print("[SYSTEM] Disconnected.")

    # ------------------------------------------------------------------ #
    #  Sending                                                             #
    # ------------------------------------------------------------------ #

    def _send_packet(self, msg_type, payload=b""):
        header = struct.pack(HEADER_FMT, self.msg_id_counter, len(payload), int(msg_type))
        self.pub_cmd.send(header + payload)
        sent_id = self.msg_id_counter
        self.msg_id_counter += 1
        return sent_id

    def _heartbeat_loop(self):
        while self.running:
            self._send_packet(MsgType.HEARTBEAT)
            time.sleep(4.0)

    # ------------------------------------------------------------------ #
    #  Receiving                                                           #
    # ------------------------------------------------------------------ #

    def _listen_loop(self):
        poller = zmq.Poller()
        poller.register(self.sub_resp,     zmq.POLLIN)
        poller.register(self.sub_telemetry, zmq.POLLIN)

        while self.running:
            socks = dict(poller.poll(100))

            if self.sub_resp in socks:
                self._handle_response(self.sub_resp.recv())

            if self.sub_telemetry in socks:
                self._handle_telemetry(self.sub_telemetry.recv())

    def _handle_response(self, data):
        if len(data) < HEADER_SIZE:
            return
        m_id, size, m_type = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
        try:
            name = MsgType(m_type).name
        except ValueError:
            name = f"UNKNOWN(0x{m_type:02X})"

        print(f"[RESPONSE] ID:{m_id:>6}  {name}")

        # Extra info for MOTOR_COUNT
        if m_type == MsgType.MOTOR_COUNT and len(data) >= HEADER_SIZE + 4:
            count = struct.unpack_from("<I", data, HEADER_SIZE)[0]
            print(f"           Motor count: {count}")

    def _handle_telemetry(self, data):
        # TelemetryOdometry layout (packed):
        #   timestamp_ms(8) + 7x float(28) + wheelCount(1) = 37 bytes
        ODO_FMT  = "<QfffffffB"
        ODO_SIZE = struct.calcsize(ODO_FMT)
        WHEEL_FMT  = "<ff"
        WHEEL_SIZE = struct.calcsize(WHEEL_FMT)

        if len(data) < HEADER_SIZE + ODO_SIZE:
            print(f"[TELEMETRY] Short packet ({len(data)} bytes)")
            return

        offset = HEADER_SIZE
        (ts, dx, dy, vx, vy, fa, ca, cav, wc) = struct.unpack_from(ODO_FMT, data, offset)
        offset += ODO_SIZE

        print(
            f"[TELEMETRY] t={ts}ms  "
            f"dist=({dx:.3f},{dy:.3f})  "
            f"vel=({vx:.3f},{vy:.3f})  "
            f"ang={ca:.3f}rad  "
            f"wheels={wc}"
        )

        for i in range(wc):
            if offset + WHEEL_SIZE > len(data):
                break
            spd, dist = struct.unpack_from(WHEEL_FMT, data, offset)
            offset += WHEEL_SIZE
            print(f"           W{i}: speed={spd:.3f}  dist={dist:.3f}")

    # ------------------------------------------------------------------ #
    #  Commands                                                            #
    # ------------------------------------------------------------------ #

    def handshake(self):
        return self._send_packet(MsgType.HANDSHAKE)

    def move(self, x: float, y: float, rot: float, front_rot: float = 0.0):
        payload = struct.pack("<Hffff", CommandType.MOVE_AT_SPEED_RAW, x, y, rot, front_rot)
        return self._send_packet(MsgType.COMMAND, payload)

    def stop(self):
        payload = struct.pack("<H", CommandType.STOP)
        return self._send_packet(MsgType.COMMAND, payload)

    def clear_queue(self):
        return self._send_packet(MsgType.CLEAR_COMMAND_QUEUE)

    def move_by_time(self, time_s: float, x: float, y: float, rot: float, front_rot: float = 0.0):
        payload = struct.pack("<Hfffff", CommandType.MOVE_BY_TIME_RAW, time_s, x, y, rot, front_rot)
        return self._send_packet(MsgType.COMMAND, payload)

    def move_by_distance(self, dist_mm: float, x: float, y: float, rot: float, front_rot: float = 0.0):
        payload = struct.pack("<Hfffff", CommandType.MOVE_BY_DISTANCE_RAW, dist_mm, x, y, rot, front_rot)
        return self._send_packet(MsgType.COMMAND, payload)

    def run_motor(self, motor_id: int, speed: float, time_s: float):
        payload = struct.pack("<HHff", CommandType.RUN_MOTOR_FOR_TIME, motor_id, speed, time_s)
        return self._send_packet(MsgType.COMMAND, payload)

    def stop_motor(self, motor_id: int):
        payload = struct.pack("<HH", CommandType.STOP_MOTOR, motor_id)
        return self._send_packet(MsgType.COMMAND, payload)


# ------------------------------------------------------------------ #
#  CLI                                                                 #
# ------------------------------------------------------------------ #

HELP = """
Commands:
  handshake
  move <x> <y> <rot> [front_rot]
  stop
  move_time <time_s> <x> <y> <rot> [front_rot]
  move_dist <dist_mm> <x> <y> <rot> [front_rot]
  motor <id> <speed> <time_s>
  stop_motor <id>
  clear
  exit / quit
"""

def main():
    import sys

    ip = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    bot = RobotController(ip)
    bot.start()

    print(HELP)

    session = PromptSession()

    # patch_stdout makes prompt_toolkit redraw the prompt cleanly
    # whenever background threads print to stdout
    with patch_stdout():
        while True:
            try:
                line = session.prompt(">> ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not line:
                continue

            parts = line.split()
            cmd   = parts[0].lower()

            try:
                if cmd == "handshake":
                    bot.handshake()

                elif cmd == "move":
                    if len(parts) < 4:
                        print("Usage: move <x> <y> <rot> [front_rot]")
                    else:
                        args = [float(p) for p in parts[1:5]]
                        bot.move(*args)

                elif cmd == "stop":
                    bot.stop()

                elif cmd == "move_time":
                    if len(parts) < 5:
                        print("Usage: move_time <time_s> <x> <y> <rot> [front_rot]")
                    else:
                        args = [float(p) for p in parts[1:6]]
                        bot.move_by_time(*args)

                elif cmd == "move_dist":
                    if len(parts) < 5:
                        print("Usage: move_dist <dist_mm> <x> <y> <rot> [front_rot]")
                    else:
                        args = [float(p) for p in parts[1:6]]
                        bot.move_by_distance(*args)

                elif cmd == "motor":
                    if len(parts) != 4:
                        print("Usage: motor <id> <speed> <time_s>")
                    else:
                        bot.run_motor(int(parts[1]), float(parts[2]), float(parts[3]))

                elif cmd == "stop_motor":
                    if len(parts) != 2:
                        print("Usage: stop_motor <id>")
                    else:
                        bot.stop_motor(int(parts[1]))

                elif cmd == "clear":
                    bot.clear_queue()

                elif cmd in ("exit", "quit"):
                    break

                elif cmd == "help":
                    print(HELP)

                else:
                    print(f"Unknown command '{cmd}'. Type 'help' for a list.")

            except (ValueError, struct.error) as e:
                print(f"[ERROR] Bad arguments: {e}")

    bot.close()


if __name__ == "__main__":
    main()