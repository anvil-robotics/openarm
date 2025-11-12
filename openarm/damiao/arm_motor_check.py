"""Arm motor check script for OpenArm.

This script checks all configured arm motors by moving them through their range of motion
sequentially. Each motor goes through the sequence: 0 rad → 0.15 rad → 0 rad,
with position verification at each step.

Usage:
    python -m openarm.damiao.arm_motor_check --iface INTERFACE --side {left,right}

Examples:
    python -m openarm.damiao.arm_motor_check --iface follower_l --side left
    python -m openarm.damiao.arm_motor_check --iface can0 --side right
"""

import argparse
import asyncio
import sys
import time
from dataclasses import dataclass
from math import pi
from typing import Optional

import can

from openarm.bus import Bus

from .config import MOTOR_CONFIGS
from .encoding import ControlMode, PosVelControlParams
from .motor import Motor


# Test parameters (constants)
TEST_VELOCITY = 0.2  # rad/s
POSITION_TOLERANCE = 0.01  # rad
POSITION_TIMEOUT = 10.0  # seconds
POLL_INTERVAL = 0.1  # seconds
# Safety margin to keep away from mechanical limits
SAFETY_MARGIN_RAD = 0.01

# Determine test position - try to move 0.15 rad in a valid direction
TEST_MOVEMENT_RAD = 0.15

@dataclass
class MotorTestResult:
    """Result of a motor test."""

    motor_name: str
    success: bool
    error: Optional[str] = None


async def wait_for_position(
    motor: Motor, target_position: float, timeout: float = POSITION_TIMEOUT
) -> bool:
    """Wait for motor to reach target position within tolerance.

    Args:
        motor: Motor instance to monitor
        target_position: Target position in radians
        timeout: Maximum wait time in seconds

    Returns:
        True if position reached within timeout, False otherwise
    """
    start_time = time.time()

    while time.time() - start_time < timeout:
        try:
            state = await motor.refresh_status()
            position_error = abs(state.position - target_position)

            if position_error < POSITION_TOLERANCE:
                return True

            # Brief sleep between polls
            await asyncio.sleep(POLL_INTERVAL)

        except Exception as e:
            sys.stderr.write(f"Error checking position: {e}\n")
            return False

    return False


async def test_single_motor(bus: Bus, motor_config, side: str) -> MotorTestResult:
    """Test a single motor through its movement sequence.

    Args:
        bus: Shared Bus instance for CAN communication
        motor_config: MotorConfig instance with motor parameters
        side: Arm side ('left' or 'right')

    Returns:
        MotorTestResult indicating success or failure
    """
    motor_name = motor_config.name
    slave_id = motor_config.slave_id
    master_id = motor_config.master_id
    motor_type = motor_config.type

    # Get side-specific angle limits
    min_angle = (
        motor_config.min_angle_left if side == "left" else motor_config.min_angle_right
    )
    max_angle = (
        motor_config.max_angle_left if side == "left" else motor_config.max_angle_right
    )

    # Convert limits from degrees to radians
    min_angle_rad = min_angle * pi / 180
    max_angle_rad = max_angle * pi / 180

    # Try positive direction first
    if 0.0 + TEST_MOVEMENT_RAD <= max_angle_rad - SAFETY_MARGIN_RAD:
        test_position_rad = TEST_MOVEMENT_RAD
    # Try negative direction if positive doesn't work
    elif 0.0 - TEST_MOVEMENT_RAD >= min_angle_rad + SAFETY_MARGIN_RAD:
        test_position_rad = -TEST_MOVEMENT_RAD
    # If neither ±0.15 fits, use middle of valid range
    else:
        range_center = (min_angle_rad + max_angle_rad) / 2
        # Clamp to safe bounds
        test_position_rad = max(
            min_angle_rad + SAFETY_MARGIN_RAD,
            min(max_angle_rad - SAFETY_MARGIN_RAD, range_center),
        )

    sys.stdout.write(f"\n{'='*60}\n")
    sys.stdout.write(
        f"Testing {motor_name} ({motor_type.value}, "
        f"Slave ID: {slave_id}, Master ID: {master_id})\n"
    )
    sys.stdout.write(
        f"  Range: {min_angle:+.1f}° to {max_angle:+.1f}° "
        f"({min_angle_rad:+.3f} to {max_angle_rad:+.3f} rad)\n"
    )
    sys.stdout.write(f"{'='*60}\n")

    try:
        # Create motor instance with shared bus
        motor = Motor(
            bus, slave_id=slave_id, master_id=master_id, motor_type=motor_type
        )

        # Step 1: Enable motor
        sys.stdout.write("  ➤ Enabling motor... ")
        sys.stdout.flush()
        await motor.enable()
        sys.stdout.write("✓\n")

        # Step 2: Set control mode to POS_VEL
        sys.stdout.write("  ➤ Setting control mode to POS_VEL... ")
        sys.stdout.flush()
        await motor.set_control_mode(ControlMode.POS_VEL)
        sys.stdout.write("✓\n")

        # Step 3: Move to 0.0 rad
        sys.stdout.write("  ➤ Moving to 0.0 rad... ")
        sys.stdout.flush()
        params = PosVelControlParams(position=0.0, velocity=TEST_VELOCITY)
        await motor.control_pos_vel(params)

        if await wait_for_position(motor, 0.0):
            sys.stdout.write("✓\n")
        else:
            sys.stdout.write("✗ (timeout)\n")
            await motor.disable()
            return MotorTestResult(
                motor_name, False, "Timeout waiting for position 0.0 rad"
            )

        # Step 4: Move to test position
        test_position_deg = test_position_rad * 180 / pi
        sys.stdout.write(
            f"  ➤ Moving to {test_position_rad:+.3f} rad ({test_position_deg:+.2f}°)... "
        )
        sys.stdout.flush()
        params = PosVelControlParams(position=test_position_rad, velocity=TEST_VELOCITY)
        await motor.control_pos_vel(params)

        if await wait_for_position(motor, test_position_rad):
            sys.stdout.write("✓\n")
        else:
            sys.stdout.write("✗ (timeout)\n")
            await motor.disable()
            return MotorTestResult(
                motor_name,
                False,
                f"Timeout waiting for position {test_position_rad:.3f} rad",
            )

        # Step 5: Move back to 0.0 rad
        sys.stdout.write("  ➤ Moving back to 0.0 rad... ")
        sys.stdout.flush()
        params = PosVelControlParams(position=0.0, velocity=TEST_VELOCITY)
        await motor.control_pos_vel(params)

        if await wait_for_position(motor, 0.0):
            sys.stdout.write("✓\n")
        else:
            sys.stdout.write("✗ (timeout)\n")
            await motor.disable()
            return MotorTestResult(
                motor_name, False, "Timeout waiting for final position 0.0 rad"
            )

        # Step 6: Disable motor
        sys.stdout.write("  ➤ Disabling motor... ")
        sys.stdout.flush()
        await motor.disable()
        sys.stdout.write("✓\n")

        sys.stdout.write(f"  ✓ {motor_name} test PASSED\n")
        return MotorTestResult(motor_name, True)

    except Exception as e:
        sys.stderr.write(f"\n  ✗ {motor_name} test FAILED: {e}\n")
        return MotorTestResult(motor_name, False, str(e))


async def test_all_motors(can_interface: str, side: str):
    """Test all configured motors sequentially.

    Args:
        can_interface: CAN interface name (e.g., 'follower_l', 'can0')
        side: Arm side ('left' or 'right')
    """
    sys.stdout.write("\n")
    sys.stdout.write("╔════════════════════════════════════════════════════════════╗\n")
    sys.stdout.write("║           OpenArm Motor Check Script                      ║\n")
    sys.stdout.write("╚════════════════════════════════════════════════════════════╝\n")
    sys.stdout.write(f"\nCAN Interface: {can_interface}\n")
    sys.stdout.write(f"Arm Side: {side.capitalize()}\n")
    sys.stdout.write(f"Test Velocity: {TEST_VELOCITY} rad/s\n")
    sys.stdout.write(f"Position Tolerance: {POSITION_TOLERANCE} rad\n")
    sys.stdout.write(f"Position Timeout: {POSITION_TIMEOUT} seconds\n")
    sys.stdout.write(f"\nTesting {len(MOTOR_CONFIGS)} motors...\n")

    # Create shared CAN bus instance for all motors
    can_bus = can.Bus(channel=can_interface, interface="socketcan")
    bus = Bus(can_bus)

    try:
        results = []

        for motor_config in MOTOR_CONFIGS:
            result = await test_single_motor(bus, motor_config, side)
            results.append(result)

        # Print summary
        sys.stdout.write(f"\n\n{'='*60}\n")
        sys.stdout.write("TEST SUMMARY\n")
        sys.stdout.write(f"{'='*60}\n")

        passed = sum(1 for r in results if r.success)
        failed = sum(1 for r in results if not r.success)

        for result in results:
            status = "✓ PASS" if result.success else "✗ FAIL"
            sys.stdout.write(f"  {result.motor_name:4s}: {status}")
            if result.error:
                sys.stdout.write(f" - {result.error}")
            sys.stdout.write("\n")

        sys.stdout.write(f"\n{'='*60}\n")
        sys.stdout.write(f"Total: {len(results)} motors\n")
        sys.stdout.write(f"Passed: {passed}\n")
        sys.stdout.write(f"Failed: {failed}\n")
        sys.stdout.write(f"{'='*60}\n\n")

        # Exit with error code if any tests failed
        if failed > 0:
            sys.exit(1)

    finally:
        # Clean up CAN bus resources
        can_bus.shutdown()


def main():
    """Main entry point for the script."""
    parser = argparse.ArgumentParser(
        description="Arm motor check for OpenArm - tests all motors sequentially",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--iface",
        "-i",
        required=True,
        help="CAN interface name (e.g., follower_l, follower_r, leader_l, leader_r, can0, can1)",
    )
    parser.add_argument(
        "--side",
        "-s",
        required=True,
        choices=["left", "right"],
        help="Arm side (left or right) - determines angle limits from motor configuration",
    )

    args = parser.parse_args()

    try:
        asyncio.run(test_all_motors(args.iface, args.side))
    except KeyboardInterrupt:
        sys.stderr.write("\n\nTest interrupted by user\n")
        sys.exit(1)
    except Exception as e:
        sys.stderr.write(f"\n\nFatal error: {e}\n")
        sys.exit(1)


if __name__ == "__main__":
    main()

