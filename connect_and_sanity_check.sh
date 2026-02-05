#!/usr/bin/env bash
set -euo pipefail  # Exit on error, unset vars, or failed pipe segment.

IFACE="${1:-}"  # Optional interface name (e.g., enp0s31f6).
HOST_IP_CIDR="${2:-192.168.123.50/24}"  # Host (your PC) IP on robot subnet; default matches Unitree 192.168.123.0/24 with a non-conflicting IP. /24 = 255.255.255.0. Other safe choices: 192.168.123.2/24, 192.168.123.7/24, 192.168.123.200/24 (avoid robot IP, .0, .255).
ROBOT_IP="${3:-192.168.123.110}"  # Robot IP to ping for sanity check.

if [[ $EUID -ne 0 ]]; then  # Require root to change network settings.
  echo "This script must be run as root (use sudo)." >&2  # Print error to stderr.
  exit 1  # Stop if not running as root.
fi

echo "Available interfaces:"  # Show all interfaces before auto-selection.
ip link  # Print interface list.
echo

if [[ -z "$IFACE" ]]; then  # If interface not provided, try to detect one.
  # Prefer USB Ethernet (enx*) with link up.
  IFACE="$(ip -o link show | awk -F': ' '{print $2 " " $9}' | grep -E '^enx' | grep -v 'NO-CARRIER' | awk '{print $1}' | head -n 1 || true)"  # Pick first linked enx*.
fi
if [[ -z "$IFACE" ]]; then  # If still empty, try wired en* with link up.
  # Try to auto-detect a wired interface (prefer "en*" names) with link up.
  IFACE="$(ip -o link show | awk -F': ' '{print $2 " " $9}' | grep -E '^en' | grep -v 'NO-CARRIER' | awk '{print $1}' | head -n 1 || true)"  # Pick first linked en*.
fi
if [[ -z "$IFACE" ]]; then  # If still empty, fall back to any en*.
  # Fall back to the first wired interface (en*), even if not linked.
  IFACE="$(ip -o link show | awk -F': ' '{print $2}' | grep -E '^en' | head -n 1 || true)"  # Pick first en*.
fi

if [[ -z "$IFACE" ]]; then  # If still empty, bail out with guidance.
  echo "Could not auto-detect a wired interface. Please pass one explicitly." >&2  # Error message.
  echo "Example: sudo $0 enp0s31f6 192.168.123.7/24 192.168.123.110" >&2  # Usage hint.
  exit 1  # Stop because no interface is available.
fi

echo "Using interface: $IFACE"  # Print selected interface.
echo "Current IPs on $IFACE:"  # Show current IP before reconfiguring.
ip -o -4 addr show "$IFACE" || true  # Print IPv4 addresses if present.

sudo ifconfig "$IFACE" down  # Bring interface down before changing IP.
sudo ifconfig "$IFACE" "$HOST_IP_CIDR"  # Set the interface IP and netmask.
sudo ifconfig "$IFACE" up  # Bring interface back up.

echo "Pinging $ROBOT_IP to verify connectivity..."  # Announce ping check.
ping -c 4 "$ROBOT_IP"  # Send 4 pings to confirm connectivity.
