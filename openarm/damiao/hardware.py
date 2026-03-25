"""CAN bus discovery, motor detection, and the Arm dataclass.

Factored out of monitor.py to keep the main teleop loop focused.
"""

from __future__ import annotations

import asyncio
import logging
import sys
from dataclasses import dataclass, field

import can

from openarm.bus import Bus

from .config import MOTOR_CONFIGS
from .detect import detect_motors
from .encoding import ControlMode
from .motor import Motor

logger = logging.getLogger(__name__)

RED = "\033[91m"
GREEN = "\033[92m"
RESET = "\033[0m"


# ---------------------------------------------------------------------------
# Arm dataclass
# ---------------------------------------------------------------------------

@dataclass
class Arm:
    """Represents a single robotic arm with its motors and configuration."""

    position: str  # "left" or "right"
    can_bus: can.BusABC
    channel: str  # e.g. "can0", "can1"
    motors: list[Motor | None] = field(default_factory=list)
    states: list = field(default_factory=list)
    is_master: bool = False
    is_slave: bool = False
    mirror_mode: bool = False
    follows: str | None = None  # channel name of master (for slaves)

    @property
    def active_motors(self) -> list[Motor]:
        return [m for m in self.motors if m is not None]

    @property
    def active_count(self) -> int:
        return len(self.active_motors)

    def get_positions(self) -> list[float]:
        """Return current joint positions, 0.0 for missing motors/states."""
        return [
            st.position if m is not None and st is not None else 0.0
            for m, st in zip(self.motors, self.states)
        ]

    async def disable_all_motors(self) -> None:
        for motor in self.motors:
            if motor is not None:
                try:
                    await motor.disable()
                except Exception as e:  # noqa: BLE001
                    logger.debug("Failed to disable motor: %s", e)

    async def enable_all_motors(self, control_mode: ControlMode) -> None:
        for idx, motor in enumerate(self.motors):
            if motor is not None:
                try:
                    await motor.enable()
                    await motor.set_control_mode(control_mode)
                    logger.info("Motor %d: Enabled", idx + 1)
                    sys.stdout.write(f"    Motor {idx + 1}: Enabled\n")
                except Exception as e:
                    logger.exception("Motor %d: Error", idx + 1)
                    sys.stderr.write(f"{RED}    Motor {idx + 1}: Error - {e}{RESET}\n")

    async def refresh_states(self) -> None:
        new_states = []
        for motor in self.motors:
            if motor:
                try:
                    state = await motor.refresh_status()
                    new_states.append(state)
                except Exception as e:  # noqa: BLE001
                    logger.debug("Failed to refresh motor status: %s", e)
                    new_states.append(None)
            else:
                new_states.append(None)
        self.states = new_states


# ---------------------------------------------------------------------------
# CAN bus helpers
# ---------------------------------------------------------------------------

def open_can_buses() -> list[can.BusABC]:
    """Detect and open all available SocketCAN buses."""
    try:
        if hasattr(can, "detect_available_configs"):
            configs = can.detect_available_configs("socketcan")
        else:
            import subprocess
            result = subprocess.run(
                ["/opt/iproute2-root/bin/ip", "link", "show"],
                capture_output=True, text=True,
            )
            configs = []
            lines = result.stdout.splitlines()
            for i, line in enumerate(lines):
                if i + 1 < len(lines) and "link/can" in lines[i + 1]:
                    iface = line.split(":")[1].strip().split("@")[0]
                    configs.append({"channel": iface, "interface": "socketcan"})
        print(f"detect_available_configs: {configs}")
        return [
            can.Bus(channel=c["channel"], interface=c["interface"]) for c in configs
        ]
    except Exception as e:  # noqa: BLE001
        print(f"Exception: {e}")
        return []


async def detect_and_disable_motors(
    can_buses: list[can.BusABC],
) -> tuple[list[list[Motor | None]], list[list]] | None:
    """Scan every CAN bus for expected motors, disable them, and return
    ``(all_bus_motors, all_state_results)`` or ``None`` on failure."""
    all_bus_motors: list[list[Motor | None]] = []
    has_missing = False

    for bus_idx, can_bus in enumerate(can_buses):
        sys.stdout.write(f"\nScanning for motors on bus {bus_idx + 1}...\n")
        slave_ids = [config.slave_id for config in MOTOR_CONFIGS]
        detected = list(detect_motors(can_bus, slave_ids, timeout=0.01))
        sys.stdout.write(f"\nBus {bus_idx + 1} Motor Status:\n")
        detected_lookup = {info.slave_id: info for info in detected}

        bus_motors: list[Motor | None] = []
        for config in MOTOR_CONFIGS:
            if config.slave_id not in detected_lookup:
                sys.stderr.write(
                    f"  {RED}✗{RESET} {config.name}: ID 0x{config.slave_id:02X} "
                    f"(Master: 0x{config.master_id:02X}) {RED}[NOT DETECTED]{RESET}\n"
                )
                bus_motors.append(None)
                has_missing = True
            elif detected_lookup[config.slave_id].master_id != config.master_id:
                detected_info = detected_lookup[config.slave_id]
                sys.stderr.write(
                    f"  {RED}✗{RESET} {config.name}: ID 0x{config.slave_id:02X} "
                    f"{RED}[MASTER ID MISMATCH: Expected 0x{config.master_id:02X}, "
                    f"Got 0x{detected_info.master_id:02X}]{RESET}\n"
                )
                bus_motors.append(None)
                has_missing = True
            else:
                sys.stdout.write(
                    f"  {GREEN}✓{RESET} {config.name}: ID 0x{config.slave_id:02X} "
                    f"(Master: 0x{config.master_id:02X})\n"
                )
                bus = Bus(can_bus)
                motor = Motor(
                    bus,
                    slave_id=config.slave_id,
                    master_id=config.master_id,
                    motor_type=config.type,
                )
                bus_motors.append(motor)
        all_bus_motors.append(bus_motors)

    if has_missing:
        sys.stderr.write(
            f"\n{RED}Error: Not all motors are detected or configured "
            f"correctly. Exiting.{RESET}\n"
        )
        return None

    total = sum(1 for bm in all_bus_motors for m in bm if m is not None)
    if total == 0:
        sys.stderr.write(f"\n{RED}Error: No motors detected on any bus.{RESET}\n")
        return None

    sys.stdout.write(
        f"\n{GREEN}Total {total} motors detected across "
        f"{len(can_buses)} bus(es){RESET}\n"
    )

    sys.stdout.write("\nDisabling all motors...\n")
    all_state_results: list[list] = []
    for bus_idx, bus_motors in enumerate(all_bus_motors):
        bus_states = []
        for motor in bus_motors:
            if motor:
                try:
                    state = await motor.disable()
                    bus_states.append(state)
                except Exception as e:
                    logger.exception("Error disabling motor on bus %d", bus_idx + 1)
                    sys.stderr.write(
                        f"{RED}Error disabling motor on bus {bus_idx + 1}: {e}{RESET}\n"
                    )
                    bus_states.append(None)
            else:
                bus_states.append(None)
        all_state_results.append(bus_states)

    return all_bus_motors, all_state_results


# ---------------------------------------------------------------------------
# Standalone motor monitor (non-teleop mode)
# ---------------------------------------------------------------------------

async def monitor_motors(
    can_buses: list[can.BusABC],
    all_bus_motors: list[list[Motor | None]],
    all_state_results: list[list],
) -> None:
    """Monitor motor angles continuously and display them in a table format."""
    sys.stdout.write("\nContinuously monitoring motor angles (Ctrl+C to stop):\n\n")

    header = "  Motor"
    for bus_idx in range(len(can_buses)):
        header += f"        Bus {bus_idx + 1}     "
    sys.stdout.write(header + "\n")
    sys.stdout.write("  " + "-" * (len(header) - 2) + "\n")

    for config in MOTOR_CONFIGS:
        line = f"  {config.name:<12}"
        for _ in range(len(can_buses)):
            line += "  Initializing...  "
        sys.stdout.write(line + "\n")

    num_motors = len(MOTOR_CONFIGS)
    all_current_states = all_state_results  # noqa: F841

    try:
        while True:
            await asyncio.sleep(0.1)

            new_all_states = []
            for bus_motors in all_bus_motors:
                bus_states = []
                for motor in bus_motors:
                    if motor:
                        try:
                            state = await motor.refresh_status()
                            bus_states.append(state)
                        except Exception as e:  # noqa: BLE001
                            logger.debug("Failed to refresh motor status: %s", e)
                            bus_states.append(None)
                    else:
                        bus_states.append(None)
                new_all_states.append(bus_states)
            all_current_states = new_all_states  # noqa: F841

    except KeyboardInterrupt:
        sys.stdout.write(f"\033[{num_motors}B\n")
        sys.stdout.write("\nMonitoring stopped.\n")
