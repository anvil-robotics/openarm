"""CAN bus initialization utilities for different platforms."""
# ruff: noqa: BLE001

import platform
import time

import can
import usb.core


def create_can_bus(interface: str = "can0", max_attempts: int = 10) -> list[can.BusABC]:  # noqa: ARG001
    """Create and initialize CAN bus based on platform.

    Args:
        interface: CAN interface name for Linux (ignored on Windows/macOS)
        max_attempts: Maximum connection attempts for USB devices

    Returns:
        List of initialized CAN buses (empty if failed)

    """
    if platform.system() in ["Windows", "Darwin"]:  # Darwin is macOS
        devs = usb.core.find(idVendor=0x1D50, idProduct=0x606F, find_all=True)
        if not devs:
            return []

        devs = list(devs)  # Convert to list

        can_buses = []
        for dev in devs:
            # Retry connection for USB devices
            for attempt in range(max_attempts):
                try:
                    can_bus = can.Bus(
                        interface="gs_usb",
                        channel=dev.product,
                        bitrate=1000000,
                        bus=dev.bus,
                        address=dev.address,
                    )
                    can_buses.append(can_bus)
                    break
                except Exception:
                    if attempt < max_attempts - 1:
                        time.sleep(0.1)
            else:
                pass

        return can_buses

    # Linux
    try:
        configs = can.detect_available_configs("socketcan")
        return [
            can.Bus(channel=config["channel"], interface=config["interface"])
            for config in configs
        ]
    except Exception:
        return []
