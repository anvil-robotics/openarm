import time
import signal
import numpy as np

from xarm.wrapper import XArmAPI
import bota_driver
   


# =========================
# User settings
# =========================

XARM_IP = "192.168.1.11"

# Paths to your BOTA config files
BOTA_DRIVER_CONFIG = "/home/hans/projects/openarm/examples/Bota_Socket_SN001310.json"

# Control loop
CONTROL_HZ = 200.0
DT_NOMINAL = 1.0 / CONTROL_HZ

M = np.array([0.1, 0.1, 0.1], dtype=float)
B = np.array([1.0, 1.0, 1.0], dtype=float)
K = np.array([0.0, 0.0, 0.0], dtype=float)

FORCE_DEADBAND = np.array([0.2, 0.2, 0.2], dtype=float)
MAX_VEL = np.array([200.0, 200.0, 200.0], dtype=float)
LPF_ALPHA = 0.5

# Safety clamps
MAX_VEL = np.array([80.0, 80.0, 80.0], dtype=float)      # mm/s
MAX_FORCE = 40.0  # N, emergency stop if exceeded on any translational axis

# Choose whether to use BOTA payload compensation
USE_PAYLOAD_COMPENSATOR = True

# If using payload compensator, set the IMU offset correctly for your hardware
IMU_OFFSET = [0.0, 0.0, -0.0257]



CONTROL_HZ = 100.0
FORCE_DEADBAND = np.array([0.02, 0.02, 0.02], dtype=float)
LPF_ALPHA = 1.0
MAX_VEL = np.array([400.0, 400.0, 400.0], dtype=float)
GAIN = np.array([60.0, 60.0, 60.0], dtype=float)
# =========================
# Global stop flag
# =========================

stop_flag = False

def signal_handler(signum, frame):
    global stop_flag
    stop_flag = True


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)
 

def deadband(vec, band):
    """
    Apply a deadband to a vector.
    """
    out = vec.copy()
    for i in range(len(out)):
        if abs(out[i]) < band[i]:
            out[i] = 0.0
        else:
            out[i] = np.sign(out[i]) * (abs(out[i]) - band[i])
    return out

def clamp(vec, limits):
    return np.clip(vec, -limits, limits)

def lowpass(prev, new, alpha):
    return alpha * new + (1.0 - alpha) * prev


# =========================
# BOTA wrapper
# =========================

class SimpleBotaFT:
    """
    Very small wrapper around the BOTA driver/payload utilities.

    Output:
        get_force_xyz() -> np.array([Fx, Fy, Fz]) in Newtons

    Notes:
    - This uses the BOTA lifecycle pattern (configure -> activate -> update -> cached output),
      which is how BOTA documents the driver usage. If you use the payload compensator,
      it is connected as an input block to the FT sensor HWI. :contentReference[oaicite:2]{index=2}
    """

    def __init__(self, driver_config_path):
        self.driver_config_path = driver_config_path

        self.driver = None
        self.ft_hwi = None
        self.comp = None

        self.force_bias = np.zeros(3, dtype=float)
        self.filtered_force = np.zeros(3, dtype=float)

    def start(self):
        self.driver = bota_driver.BotaDriver(self.driver_config_path)
        # Transition driver from UNCONFIGURED to INACTIVE state
        if not self.driver.configure():
            raise RuntimeError("Failed to configure driver")

        # Uncomment to tare the sensor
        if not self.driver.tare():
            raise RuntimeError("Failed to tare sensor")

        # Transition driver from INACTIVE to ACTIVE state
        if not self.driver.activate():
            raise RuntimeError("Failed to activate driver")

    def stop(self):
        # Transition driver from ACTIVE to INACTIVE state
        if not self.driver.deactivate():
            raise RuntimeError("Failed to deactivate driver")
        
        # Shutdown the driver
        if not self.driver.shutdown():
            raise RuntimeError("Failed to shutdown driver")

 
    def read_force_xyz_raw(self):
        """
        Read raw force from the FT HWI cached output.
        """
        frame = self.driver.read_frame()
        # Expected shape from BOTA example style:
        # frame.force[0], frame.force[1], frame.force[2]
        return np.array([frame.force[0], frame.force[1], frame.force[2]], dtype=float)
 

 

# =========================
# xArm helper
# =========================

class SimpleXArmVelocity:
    """
    Minimal xArm Cartesian velocity interface.
    """

    def __init__(self, ip):
        self.arm = XArmAPI(ip)

    def start(self):
        self.arm.motion_enable(enable=True)
        self.arm.clean_warn()
        self.arm.clean_error()
        self.arm.set_mode(5)   # Cartesian velocity mode
        self.arm.set_state(0)
        time.sleep(0.2)

    def stop(self):
        try:
            self.send_velocity(np.zeros(3))
        except Exception:
            pass
        try:
            self.arm.set_state(0)
        except Exception:
            pass
        try:
            self.arm.disconnect()
        except Exception:
            pass

    def send_velocity(self, v_xyz_mm_s):
        speeds = [
            float(v_xyz_mm_s[0]),
            float(v_xyz_mm_s[1]),
            float(v_xyz_mm_s[2]),
            0.0, 0.0, 0.0
        ]
        code = self.arm.vc_set_cartesian_velocity(
            speeds=speeds,
            is_radian=False,
            is_tool_coord=False,
            duration=0.05,
        )
        return code

    def ok(self):
        return self.arm.connected and self.arm.state != 4


# =========================
# Main controller
# =========================

def main():
    global stop_flag

    print("Starting xArm...")
    robot = SimpleXArmVelocity(XARM_IP)
    robot.start()

    print("Starting BOTA...")
    ft = SimpleBotaFT(
        driver_config_path=BOTA_DRIVER_CONFIG,
    )
    ft.start()

    # Admittance state
    v = np.zeros(3, dtype=float)  # mm/s
    x = np.zeros(3, dtype=float)  # virtual displacement

    print("Controller running. Press Ctrl+C to stop.")
    next_t = time.perf_counter()
    last_print = time.perf_counter()

    try:
        while not stop_flag and robot.ok():
            loop_start = time.perf_counter()

        

            # 2) read force
            F_sensor = ft.read_force_xyz_raw()

            F = np.array([
                F_sensor[2],
                F_sensor[0],
                F_sensor[1],
            ], dtype=float)


            # 3) deadband
            F = deadband(F, FORCE_DEADBAND)

            # 4) emergency force stop
            if np.any(np.abs(F) > MAX_FORCE):
                print(f"Force limit exceeded: {F}")
                break

            # 5) simple admittance:
            #    M * dv + B * v + K * x = F
            #    dv = (F - B*v - K*x) / M
            # dt = DT_NOMINAL
            # a = (F - B * v - K * x) / M
            # v = v + a * dt
            # x = x + v * dt

            # 6) clamp velocity
            v_cmd = clamp(GAIN * F, MAX_VEL)
            # v_cmd = clamp(v, MAX_VEL)

            # 7) send to robot
            code = robot.send_velocity(v_cmd)
            if code != 0:
                print(f"xArm velocity command failed: {code}")
                break

            # 8) optional status print
            now = time.perf_counter()
            if now - last_print > 0.2:
                print(
                    f"F[N]=[{F[0]:6.2f}, {F[1]:6.2f}, {F[2]:6.2f}]   "
                    f"V[mm/s]=[{v_cmd[0]:6.2f}, {v_cmd[1]:6.2f}, {v_cmd[2]:6.2f}]"
                )
                last_print = now

            # 9) loop timing
            next_t += DT_NOMINAL
            sleep_t = next_t - time.perf_counter()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                # reset if behind
                next_t = time.perf_counter()

    except KeyboardInterrupt:
        pass

    finally:
        print("Stopping robot...")
        robot.stop()

        print("Stopping BOTA...")
        ft.stop()

        print("Done.")


if __name__ == "__main__":
    main()