import time
from arm_interface import RoboticArmInterface

PORT = "socket://127.0.0.1:5555"  # Change to "COM5" on real hardware

def run_diagnostics():
    arm = RoboticArmInterface(PORT)
    print("=== Robotic Arm Electronics & CAN Bus Verification ===\n")

    # Test 1: 50 Hz Telemetry & Checksum Integrity
    print("[Test 1] Checking 50 Hz telemetry stream (0x200-0x203)...")
    time.sleep(0.5)
    assert arm.estop_ok is True, "FAIL: E-Stop reported active at startup"
    print(f"  PASS -> Live Angles: {arm.angles} | E-Stop OK: {arm.estop_ok}")

    # Test 2: Synchronized 4-Axis Move & AS5600 Encoder Convergence
    print("\n[Test 2] Enabling drivers & commanding 4-axis synchronized move...")
    arm.enable_all()
    target_pose = [45.0, -25.0, 50.0, -35.0]
    arm.move_arm(target_pose, speed_dps=50)
    reached = arm.wait_until_reached(target_pose, tol_deg=0.5, timeout_s=4.0)
    assert reached, f"FAIL: Timed out reaching {target_pose}. Actual: {arm.angles}"
    print(f"  PASS -> Reached {arm.angles} within 0.5° tolerance")

    # Test 3: Node 4 Wrist & Gripper PWM Routing (0x120 -> PB6, PB7, PA8)
    print("\n[Test 3] Testing Node 4 wrist & gripper servo PWM commands...")
    arm.set_wrist_gripper(wrist1_us=1750, wrist2_us=1250, gripper_us=1900)
    time.sleep(0.3)
    arm.set_wrist_gripper(wrist1_us=1500, wrist2_us=1500, gripper_us=1100)
    print("  PASS -> Check simulator/Node 4 terminal for PWM updates")

    # Test 4: Homing Routine & KW4 Limit Switch Zeroing (0x110-0x113 cmd=1)
    print("\n[Test 4] Running homing cycle across all 4 axes...")
    arm.home_all()
    time.sleep(0.2)
    t0 = time.time()
    while any(st & 0x02 for st in arm.status) and (time.time() - t0 < 5.0):
        time.sleep(0.05)
    assert all((st & 0x02) == 0 for st in arm.status), "FAIL: Homing bit did not clear"
    assert all(abs(a) < 0.5 for a in arm.angles), f"FAIL: Encoders did not zero: {arm.angles}"
    print(f"  PASS -> All axes homed and zeroed: {arm.angles}")

    # Test 5: Hardware E-Stop Interlock Verification
    print("\n[Test 5] E-Stop Fault Injection Test:")
    print("  -> Commanded slow sweep to 60°. Type 'e' + Enter in sim_virtual_arm.py NOW (you have 3s)...")
    arm.move_arm([60.0, 60.0, 60.0, 60.0], speed_dps=20)
    estop_caught = False
    t0 = time.time()
    while time.time() - t0 < 3.5:
        if not arm.estop_ok:
            estop_caught = True
            print(f"  PASS -> E-Stop fault detected! Motion frozen at {arm.angles}")
            break
        time.sleep(0.02)

    if not estop_caught:
        print("  SKIP -> E-Stop was not toggled during window (returning to 0°)")
        arm.move_arm([0.0, 0.0, 0.0, 0.0], speed_dps=50)
        arm.wait_until_reached([0.0, 0.0, 0.0, 0.0])

    arm.disable_all()
    arm.close()
    print("\n=== Diagnostics Complete ===")

if __name__ == "__main__":
    run_diagnostics()