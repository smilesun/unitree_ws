#!/usr/bin/env bash
set -euo pipefail  # Exit on error, unset vars, or failed pipe segment.

IFACE="${1:-}"  # Optional interface name (e.g., enp0s31f6).
HOST_IP_CIDR="${2:-192.168.123.7/24}"  # Host IP/mask for the interface.
ROBOT_IP="${3:-192.168.123.110}"  # Robot IP to ping for sanity check.

if [[ $EUID -ne 0 ]]; then  # Require root to change network settings.
  echo "This script must be run as root (use sudo)." >&2  # Print error to stderr.
  exit 1  # Stop if not running as root.
fi

if [[ -z "$IFACE" ]]; then  # If interface not provided, try to detect one.
  # Try to auto-detect a wired interface (prefer "en*" names).
  IFACE="$(ip -o link show | awk -F': ' '{print $2}' | grep -E '^en' | head -n 1 || true)"  # Pick first en*.
fi

if [[ -z "$IFACE" ]]; then  # If still empty, bail out with guidance.
  echo "Could not auto-detect a wired interface. Please pass one explicitly." >&2  # Error message.
  echo "Example: sudo $0 enp0s31f6 192.168.123.7/24 192.168.123.110" >&2  # Usage hint.
  exit 1  # Stop because no interface is available.
fi

sudo ifconfig "$IFACE" down  # Bring interface down before changing IP.
sudo ifconfig "$IFACE" "$HOST_IP_CIDR"  # Set the interface IP and netmask.
sudo ifconfig "$IFACE" up  # Bring interface back up.

echo "Pinging $ROBOT_IP to verify connectivity..."  # Announce ping check.
ping -c 4 "$ROBOT_IP"  # Send 4 pings to confirm connectivity.
