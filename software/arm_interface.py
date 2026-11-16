import serial
import struct
import threading
import time

class RoboticArmInterface:
    def __init__(self, port: str = "socket://127.0.0.1:5555", baudrate: int = 921600):
        self.ser = serial.serial_for_url(port, baudrate=baudrate, timeout=0.05)
        self.angles = [0.0, 0.0, 0.0, 0.0]     # Live AS5600 feedback (deg)
        self.status = [0, 0, 0, 0]             # Bit 0: Enabled, Bit 1: Homing
        self.estop_ok = True
        self._running = True
        self._lock = threading.Lock()

        time.sleep(0.3)  # Allow ESP32 / socket connection to settle
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def _send_can(self, can_id: int, payload: bytes):
        payload = payload.ljust(8, b"\x00")[:8]
        body = struct.pack(">HB", can_id, len(payload)) + payload
        csum = 0
        for b in body:
            csum ^= b
        pkt = b"\xAA\x55" + body + bytes([csum, 0x0A])
        with self._lock:
            self.ser.write(pkt)

    def _rx_loop(self):
        while self._running:
            try:
                if self.ser.read(1) != b"\xAA":
                    continue
                if self.ser.read(1) != b"\x55":
                    continue
                rest = self.ser.read(13)
                if len(rest) == 13 and rest[12] == 0x0A:
                    csum = 0
                    for b in rest[:11]:
                        csum ^= b
                    if csum == rest[11]:
                        cid, _ = struct.unpack(">HB", rest[0:3])
                        if 0x200 <= cid <= 0x203:
                            nid = cid - 0x200
                            ang_cdeg, st, estop, _ = struct.unpack("<iBBH", rest[3:11])
                            self.angles[nid] = ang_cdeg / 100.0
                            self.status[nid] = st
                            self.estop_ok = bool(estop)
            except Exception:
                time.sleep(0.01)

    def enable_all(self):
        for n in range(4):
            self._send_can(0x110 + n, struct.pack("<B", 2))

    def disable_all(self):
        for n in range(4):
            self._send_can(0x110 + n, struct.pack("<B", 3))

    def home_all(self):
        for n in range(4):
            self._send_can(0x110 + n, struct.pack("<B", 1))

    def move_joint(self, node_id: int, angle_deg: float, speed_dps: int = 45):
        cdeg = int(angle_deg * 100)
        self._send_can(0x100 + node_id, struct.pack("<iHH", cdeg, speed_dps, 0))

    def move_arm(self, target_angles_deg: list[float], speed_dps: int = 45):
        """Command all 4 stepper joints simultaneously."""
        for nid in range(4):
            self.move_joint(nid, target_angles_deg[nid], speed_dps)

    def set_wrist_gripper(self, wrist1_us: int, wrist2_us: int, gripper_us: int):
        """Send 500-2500 us PWM commands to Node 4 (PB6, PB7, PA8)."""
        self._send_can(0x120, struct.pack("<HHHH", wrist1_us, wrist2_us, gripper_us, 0))

    def wait_until_reached(self, target_angles_deg: list[float], tol_deg: float = 0.5, timeout_s: float = 5.0) -> bool:
        """Block until all 4 encoders reach target within tolerance or E-Stop triggers."""
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if not self.estop_ok:
                print("\n[FAULT] Hardware E-Stop triggered! Aborting move.")
                return False
            if all(abs(self.angles[i] - target_angles_deg[i]) <= tol_deg for i in range(4)):
                return True
            time.sleep(0.02)
        return False

    def close(self):
        self._running = False
        self.ser.close()