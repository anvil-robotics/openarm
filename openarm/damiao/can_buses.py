import platform
import time

import can
import usb.core


def create_can_bus(interface: str = "can0", max_attempts: int = 10) -> list[can.BusABC]:
    """Create and initialize CAN bus based on platform.

    Args:
        interface: CAN interface name for Linux (ignored on Windows/macOS)
        max_attempts: Maximum connection attempts for USB devices

    Returns:
        List of initialized CAN buses (empty if failed)
    """
    if platform.system() in ["Windows", "Darwin"]:  # Darwin is macOS
        print("Searching for USB CAN devices...")
        devs = usb.core.find(idVendor=0x1D50, idProduct=0x606F, find_all=True)
        if not devs:
            print("Error: No USB CAN devices found (VID:0x1D50, PID:0x606F)")
            return []

        devs = list(devs)  # Convert to list
        print(f"Found {len(devs)} USB CAN device(s)")

        can_buses = []
        for dev in devs:
            print(f"Initializing USB CAN device: {dev.product}")

            # Retry connection for USB devices
            last_error = None
            for attempt in range(max_attempts):
                try:
                    can_bus = can.Bus(
                        interface="gs_usb",
                        channel=dev.product,
                        bitrate=1000000,
                        bus=dev.bus,
                        address=dev.address,
                    )
                    print(f"  CAN bus initialized successfully (attempt {attempt + 1})")
                    can_buses.append(can_bus)
                    break
                except Exception as e:
                    last_error = e
                    if attempt < max_attempts - 1:
                        time.sleep(0.1)
            else:
                print(
                    f"  Failed to initialize CAN bus after {max_attempts} attempts: {last_error}"
                )

        return can_buses

    # Linux
    print(f"Initializing CAN bus on {interface}...")
    try:
        configs = can.detect_available_configs("socketcan")
        return [
            can.Bus(channel=config["channel"], interface=config["interface"])
            for config in configs
        ]
    except Exception as e:
        print(f"Failed to initialize socketcan: {e}")
        return []
