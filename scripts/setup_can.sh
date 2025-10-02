#!/bin/bash

# =============================================================================
# Usage: sudo ./setup_can.sh
#
# Description:
#   This script configures four CAN devices (leader_left, leader_right,
#   follower_left, and follower_right) using systemd-networkd with a fixed
#   bitrate (1 Mbps) and ensures they always have consistent interface names
#   across reboots or replugging. If no udev rule exists yet, the script will
#   detect the device on plug-in, rename it, and append the rule to
#   /etc/udev/rules.d/90-can.rules.
#
# Requirements:
#   - Must be run as root
#   - systemd-networkd service
#   - udevadm available on the system
#
# Behavior:
#   1. Checks if the target CAN interface already exists.
#   2. If not, waits for the user to plug in a CAN device and monitors udev.
#   3. Detects the device, renames it to the expected interface name, and
#      writes a persistent udev rule based on the device serial number.
#   4. Reloads and triggers udev to apply the new rule.
#   5. Creates systemd network configuration files for each CAN interface
#      with bitrate 1000000 (1 Mbps) and enables systemd-networkd.
#
# Notes:
#   - Once rules are written, future runs won’t prompt for plugging in devices.
#   - To reset, remove /etc/udev/rules.d/90-can.rules and systemd network files.
# =============================================================================

set -e

if [[ $EUID -ne 0 ]]; then
  echo "This script must be run as root" >&2
  exit 1
fi

function setup_can {
  if ! ip link show "$1" &>/dev/null; then
    echo "→ Please plug in CAN device..."

    read_step=header
    udevadm monitor --subsystem-match=net --property | \
    while read -r line; do
      case "$read_step" in
        header)
            if [[ "$line" =~ UDEV.*add ]]; then
              read_step=interface
            else
              read_step=skip
            fi
          ;;

        interface)
          if [[ "$line" =~ INTERFACE= ]]; then
            interface=$(echo "$line" | cut -d= -f2)
            if [ "$interface" = "$1" ]; then
              echo "→ CAN device detected"
              break
            else
              read_step=serial
            fi
          elif [ "$line" = "" ]; then
            read_step=header
          fi
          ;;

        serial)
          if [[ "$line" =~ ID_SERIAL_SHORT= ]]; then
            serial=$(echo "$line" | cut -d= -f2)
            echo "→ CAN device detected: $interface [Serial: $serial]"

            echo "→ Renaming interface: $interface → $1"
            ip link set "$interface" name "$1"

            echo "→ Updating udev rules"
            echo "ACTION==\"add\", SUBSYSTEM==\"net\", KERNEL==\"can*\", ATTRS{serial}==\"$serial\", NAME=\"$1\"" >> /etc/udev/rules.d/90-can.rules
            udevadm control --reload-rules
            udevadm trigger
            break
          elif [ "$line" = "" ]; then
            read_step=header
          fi
          ;;

        skip)
          if [ "$line" = "" ]; then
            read_step=header
          fi
          ;;
      esac
    done
  fi

  echo "→ Updating systemd-networkd configuration"
  cat > "/etc/systemd/network/10-${1//_/-}.network" << EOF
[Match]
Name=$1

[CAN]
BitRate=1000000
EOF

  echo "→ Enabling systemd-networkd"
  systemctl enable systemd-networkd
  systemctl restart systemd-networkd
}

echo "[LEADER LEFT]"
setup_can leader_left

echo ""
echo "[LEADER RIGHT]"
setup_can leader_right

echo ""
echo "[FOLLOWER LEFT]"
setup_can follower_left

echo ""
echo "[FOLLOWER RIGHT]"
setup_can follower_right
