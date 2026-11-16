import socket
import struct
import threading
import time
import sys

HOST, PORT = "127.0.0.1", 5555
STEPS_PER_DEG = [8.8889, 46.0444, 8.8889, 8.8889]

class VirtualJointNode:
    def __init__(self, node_id: int):
        self.id = node_id
        self.enabled = False
        self.homing = False
        self.current_steps = 0
        self.target_steps = 0
        self.speed_dps = 45.0
        self.zero_offset_deg = 0.0

    def step_physics(self, dt: float):
        if self.homing:
            # Simulate moving toward home switch at -30 deg/s
            delta = -30.0 * STEPS_PER_DEG[self.id] * dt
            self.current_steps += int(delta)
            if self.current_steps <= 0:
                self.current_steps = 0
                self.target_steps = 0
                self.homing = False
                print(f"[Node {self.id}] KW4 Limit Switch hit -> Homed & Zeroed")
        elif self.enabled and self.current_steps != self.target_steps:
            max_step = max(1, int(self.speed_dps * STEPS_PER_DEG[self.id] * dt))
            diff = self.target_steps - self.current_steps
            move = max(-max_step, min(max_step, diff))
            self.current_steps += move

    def get_encoder_cdeg(self) -> int:
        # Quantize to 12-bit AS5600 resolution (4096 ticks/rev = 0.0879 deg)
        deg = (self.current_steps / STEPS_PER_DEG[self.id]) - self.zero_offset_deg
        ticks = round((deg / 360.0) * 4096)
        quantized_deg = (ticks * 360.0) / 4096.0
        return int(quantized_deg * 100)

class VirtualArmRig:
    def __init__(self):
        self.nodes = [VirtualJointNode(i) for i in range(4)]
        self.servos = [1500, 1500, 1500]  # Wrist 1 (PB6), Wrist 2 (PB7), Gripper (PA8)
        self.estop_ok = True
        self.client_conn = None
        self.lock = threading.Lock()

    def build_usb_frame(self, can_id: int, payload: bytes) -> bytes:
        payload = payload.ljust(8, b"\x00")[:8]
        body = struct.pack(">HB", can_id, 8) + payload
        csum = 0
        for b in body:
            csum ^= b
        return b"\xAA\x55" + body + bytes([csum, 0x0A])

    def handle_can_rx(self, can_id: int, data: bytes):
        with self.lock:
            if 0x100 <= can_id <= 0x103:
                nid = can_id - 0x100
                cdeg, spd, _ = struct.unpack("<iHH", data[:8])
                self.nodes[nid].homing = False
                self.nodes[nid].target_steps = int((cdeg / 100.0) * STEPS_PER_DEG[nid])
                if spd > 0:
                    self.nodes[nid].speed_dps = float(spd)
                print(f"[CAN -> Node {nid}] Target: {cdeg/100.0:.2f} deg @ {spd} deg/s")

            elif 0x110 <= can_id <= 0x113:
                nid = can_id - 0x110
                cmd = data[0]
                if cmd == 1 and self.estop_ok:
                    self.nodes[nid].homing = True
                    self.nodes[nid].enabled = True
                    print(f"[CAN -> Node {nid}] CMD: Start Homing")
                elif cmd == 2 and self.estop_ok:
                    self.nodes[nid].enabled = True
                    print(f"[CAN -> Node {nid}] CMD: Driver Enabled")
                elif cmd == 3:
                    self.nodes[nid].enabled = False
                    self.nodes[nid].homing = False
                    print(f"[CAN -> Node {nid}] CMD: Driver Disabled")
                elif cmd == 4:
                    self.nodes[nid].current_steps = 0
                    self.nodes[nid].target_steps = 0
                    print(f"[CAN -> Node {nid}] CMD: Encoder Zeroed")

            elif can_id == 0x120:
                s1, s2, grip, _ = struct.unpack("<HHHH", data[:8])
                self.servos = [s1, s2, grip]
                print(f"[CAN -> Node 3 PWM] Wrist1={s1}us | Wrist2={s2}us | Gripper={grip}us")

    def physics_and_telemetry_loop(self):
        dt = 0.02  # 50 Hz
        while True:
            time.sleep(dt)
            with self.lock:
                if not self.estop_ok:
                    for n in self.nodes:
                        n.enabled = False
                        n.homing = False

                for n in self.nodes:
                    n.step_physics(dt)
                    if self.client_conn:
                        status = (0x02 if n.homing else 0x00) | (0x01 if n.enabled else 0x00)
                        payload = struct.pack("<iBBH", n.get_encoder_cdeg(), status, int(self.estop_ok), 0)
                        try:
                            self.client_conn.sendall(self.build_usb_frame(0x200 + n.id, payload))
                        except OSError:
                            self.client_conn = None

    def keyboard_loop(self):
        print("Controls: Type 'e' + Enter to toggle E-Stop | 'q' + Enter to quit\n")
        for line in sys.stdin:
            cmd = line.strip().lower()
            if cmd == "e":
                with self.lock:
                    self.estop_ok = not self.estop_ok
                    print(f"\n>>> HARDWARE E-STOP STATE: {'RELEASED (OK)' if self.estop_ok else 'PRESSED (FAULT - DRIVERS CUT)'} <<<\n")
            elif cmd == "q":
                sys.exit(0)

    def serve(self):
        threading.Thread(target=self.physics_and_telemetry_loop, daemon=True).start()
        threading.Thread(target=self.keyboard_loop, daemon=True).start()

        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((HOST, PORT))
        srv.listen(1)
        print(f"Virtual ESP32+STM32 Arm Rig listening on socket://{HOST}:{PORT} ...")

        while True:
            conn, addr = srv.accept()
            print(f"Host PC connected from {addr}")
            self.client_conn = conn
            buf = bytearray()
            try:
                while True:
                    chunk = conn.recv(256)
                    if not chunk:
                        break
                    buf.extend(chunk)
                    while len(buf) >= 15:
                        if buf[0] != 0xAA or buf[1] != 0x55:
                            buf.pop(0)
                            continue
                        pkt = bytes(buf[:15])
                        del buf[:15]
                        csum = 0
                        for b in pkt[2:13]:
                            csum ^= b
                        if pkt[14] == 0x0A and csum == pkt[13]:
                            cid, dlc = struct.unpack(">HB", pkt[2:5])
                            self.handle_can_rx(cid, pkt[5:5 + dlc])
                        else:
                            print("[ESP32 Bridge] Dropped corrupted USB frame (checksum mismatch)")
            except OSError:
                pass
            finally:
                self.client_conn = None
                print("Host PC disconnected.")

if __name__ == "__main__":
    VirtualArmRig().serve()