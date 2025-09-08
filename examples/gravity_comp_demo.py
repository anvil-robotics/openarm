import argparse
import asyncio
import mujoco
import numpy as np
import platform
import sys
import time
from dataclasses import dataclass

import can
import usb.core

from openarm.bus import Bus
from openarm.damiao import ControlMode, MitControlParams, Motor, MotorType
from openarm.simulation.models import OPENARM_MODEL_PATH

@dataclass
class MotorConfig:
    """Configuration for a single motor."""
    name: str
    slave_id: int
    master_id: int
    type: MotorType

# Motor configurations
MOTOR_CONFIGS: list[MotorConfig] = [
    MotorConfig("J1", slave_id=0x01, master_id=0x11, type=MotorType.DM8009),
    MotorConfig("J2", slave_id=0x02, master_id=0x12, type=MotorType.DM8009),
    MotorConfig("J3", slave_id=0x03, master_id=0x13, type=MotorType.DM4340),
    MotorConfig("J4", slave_id=0x04, master_id=0x14, type=MotorType.DM4340),
    MotorConfig("J5", slave_id=0x05, master_id=0x15, type=MotorType.DM4310),
    MotorConfig("J6", slave_id=0x06, master_id=0x16, type=MotorType.DM4310),
    MotorConfig("J7", slave_id=0x07, master_id=0x17, type=MotorType.DM4310),
]

class MuJoCoKDL:
    """A simple class for computing inverse dynamics using MuJoCo."""

    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(OPENARM_MODEL_PATH))
        self.model.opt.gravity = np.array([0, 0, -9.81])

        self.data = mujoco.MjData(self.model)

        # Disable all collisions
        self.model.geom_contype[:] = 0
        self.model.geom_conaffinity[:] = 0

        # Disable all joint limit
        self.model.jnt_limited[:] = 0

    def compute_inverse_dynamics(self, q: np.ndarray, side: str = 'left') -> np.ndarray:
        assert side in ['left', 'right'], "side must be 'left' or 'right'"

        length = len(q)

        if side == 'left':
            # Left joints: indices 0-6
            joint_indices = slice(0, length)
        else:  # right
            # Right joints: indices 9-15, but input q is still 0-6, so we map to 9-15
            joint_indices = slice(9, 9 + length)

        # Clear all joint states first
        self.data.qpos[:] = 0

        # Set joint positions for the specified side
        self.data.qpos[joint_indices] = q

        mujoco.mj_inverse(self.model, self.data)
        return self.data.qfrc_inverse[joint_indices]

def create_can_bus(interface: str = "can0", max_attempts: int = 10) -> list[can.BusABC]:
    """Create and initialize CAN bus based on platform."""
    if platform.system() in ["Windows", "Darwin"]:  # Darwin is macOS
        print("Searching for USB CAN devices...")
        devs = usb.core.find(idVendor=0x1D50, idProduct=0x606F, find_all=True)
        if not devs:
            print("Error: No USB CAN devices found (VID:0x1D50, PID:0x606F)")
            return []

        devs = list(devs)
        print(f"Found {len(devs)} USB CAN device(s)")

        can_buses = []
        for dev in devs:
            print(f"Initializing USB CAN device: {dev.product}")

            last_error = None
            for attempt in range(max_attempts):
                try:
                    can_bus = can.Bus(
                        interface="gs_usb",
                        channel=dev.product,
                        bitrate=1000000,
                        bus=dev.bus,
                        address=dev.address
                    )
                    print(f"  CAN bus initialized successfully (attempt {attempt + 1})")
                    can_buses.append(can_bus)
                    break
                except Exception as e:
                    last_error = e
                    if attempt < max_attempts - 1:
                        time.sleep(0.1)
            else:
                print(f"  Failed to initialize CAN bus after {max_attempts} attempts: {last_error}")

        return can_buses

    # Linux
    print(f"Initializing CAN bus on {interface}...")
    try:
        return [can.Bus(channel=interface, interface="socketcan")]
    except Exception as e:
        print(f"Failed to initialize socketcan: {e}")
        return []

async def setup_motors(interface: str = "can0"):
    """Setup and initialize motors, return list of motor objects and their current positions."""
    can_buses = create_can_bus(interface)
    if not can_buses:
        return None, None

    print(f"Detected {len(can_buses)} CAN bus(es)")

    # Use first bus for simplicity
    can_bus = can_buses[0]
    print("Testing motor connectivity...")

    motors = []
    current_positions = []
    all_motors_active = True

    for config in MOTOR_CONFIGS:
        print(f"Testing motor {config.name} (ID 0x{config.slave_id:02X})...")
        bus = Bus(can_bus)
        motor = Motor(
            bus,
            slave_id=config.slave_id,
            master_id=config.master_id,
            motor_type=config.type,
        )

        # Try to enable and get status with timeout
        try:
            await asyncio.wait_for(motor.enable(), timeout=1.0)
            await asyncio.wait_for(motor.set_control_mode(ControlMode.MIT), timeout=1.0)
            state = await asyncio.wait_for(motor.refresh_status(), timeout=1.0)

            if state:
                motors.append(motor)
                current_positions.append(state.position)
                print(f"  Motor {config.name} active - Initial position: {state.position:.3f} rad")
            else:
                print(f"  Motor {config.name} inactive - No state received")
                all_motors_active = False
                motors.append(None)
                current_positions.append(0.0)

        except asyncio.TimeoutError:
            print(f"  Motor {config.name} inactive - Timeout")
            all_motors_active = False
            motors.append(None)
            current_positions.append(0.0)
        except Exception as e:
            print(f"  Motor {config.name} inactive - Error: {e}")
            all_motors_active = False
            motors.append(None)
            current_positions.append(0.0)

    # If any motor is inactive, disable all motors and return None
    if not all_motors_active:
        print("\nError: Not all motors are active. Disabling all motors...")
        for motor in motors:
            if motor is not None:
                try:
                    await motor.disable()
                except Exception:
                    pass
        return None, None

    return motors, current_positions

async def main(args: argparse.Namespace):
    """Main gravity compensation loop."""
    print("Setting up gravity compensation...")

    # Initialize dynamics computation
    kdl = MuJoCoKDL()

    # Setup motors and get initial positions
    motors, current_positions = await setup_motors(args.interface)
    if not motors:
        print("Cannot proceed: Not all motors are available")
        return

    print(f"\nStarting gravity compensation loop with {sum(1 for m in motors if m)} motors...")
    print(f"Using {args.side} arm configuration")
    print("Press Ctrl+C to stop")

    try:
        while True:
            # Get current joint positions (only for active motors)
            active_positions = []
            active_indices = []
            for i, (motor, pos) in enumerate(zip(motors, current_positions)):
                if motor is not None:
                    active_positions.append(pos)
                    active_indices.append(i)

            if not active_positions:
                print("No active motors")
                break

            # Compute gravity compensation torques using specified side
            q = np.array(active_positions)
            gravity_torques = kdl.compute_inverse_dynamics(q, side=args.side)

            # Torques scaling tuning
            gravity_torques[0] *= 0.8
            gravity_torques[1] *= 0.8
            gravity_torques[2] *= 1
            gravity_torques[3] *= 1
            gravity_torques[4] *= 1
            gravity_torques[5] *= 1
            gravity_torques[6] *= 1

            # Prepare MIT control commands for all motors
            control_tasks = []
            motor_indices = []

            for motor_idx, motor in enumerate(motors):
                if motor is not None:
                    try:
                        # Find the torque for this motor
                        active_idx = active_indices.index(motor_idx)
                        torque = gravity_torques[active_idx]

                        # MIT control with torque only (position, velocity, gains set to 0)
                        params = MitControlParams(
                            q=0,        # No position control
                            dq=0,       # No velocity control
                            kp=0,       # No position gain
                            kd=0,       # No damping gain
                            tau=torque  # Pure torque control
                        )

                        # Create control task
                        task = motor.control_mit(params)
                        control_tasks.append(task)
                        motor_indices.append(motor_idx)

                    except Exception as e:
                        print(f"Error preparing control for motor {motor_idx}: {e}")

            # Send all MIT control commands at once and await all responses
            if control_tasks:
                try:
                    states = await asyncio.gather(*control_tasks, return_exceptions=True)

                    # Update positions from all motor responses
                    for i, state in enumerate(states):
                        motor_idx = motor_indices[i]
                        if isinstance(state, Exception):
                            print(f"Error controlling motor {motor_idx}: {state}")
                        elif state:
                            current_positions[motor_idx] = state.position

                except Exception as e:
                    print(f"Error in batch control: {e}")

            # Small delay
            await asyncio.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping gravity compensation...")

        # Disable all motors
        for motor in motors:
            if motor is not None:
                try:
                    await motor.disable()
                except Exception:
                    pass

def parse_arguments() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Gravity compensation demo for OpenArm robot"
    )

    parser.add_argument(
        "--interface",
        "-i",
        default="can0",
        help="CAN interface name (default: can0, ignored on Windows/macOS)",
    )

    parser.add_argument(
        "--side",
        "-s",
        choices=["left", "right"],
        default="left",
        help="Arm side for gravity compensation (default: left)",
    )

    return parser.parse_args()


def run() -> None:
    """Entry point for the gravity compensation script."""
    args = parse_arguments()

    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        sys.exit(0)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    run()
